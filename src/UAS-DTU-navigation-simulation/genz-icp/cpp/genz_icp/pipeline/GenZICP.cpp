// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss.
// Modified by Daehan Lee, Hyungtae Lim, and Soohee Han, 2024
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "GenZICP.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "genz_icp/core/Deskew.hpp"
#include "genz_icp/core/Preprocessing.hpp"
#include "genz_icp/core/Registration.hpp"
#include "genz_icp/core/VoxelHashMap.hpp"

namespace {
constexpr size_t kStationaryWarmupFrames = 5;
constexpr size_t kMaxStationarySkippedFrames = 7;
constexpr size_t kRegistrationQualityWarmupAcceptedFrames = 3;
constexpr double kStartupPoseVariance = 1e-6;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

inline double Square(double value) { return value * value; }

struct YawSearchCandidateScore {
    double offset_deg = 0.0;
    double rmse = std::numeric_limits<double>::infinity();
    double weighted_rmse = std::numeric_limits<double>::infinity();
    double score = std::numeric_limits<double>::infinity();
    size_t correspondence_count = 0;
    Sophus::SE3d initial_guess;
    bool valid = false;
};

Eigen::Matrix<double, 4, 1> ComputeFrameSignature(const std::vector<Eigen::Vector3d> &frame) {
    Eigen::Matrix<double, 4, 1> signature = Eigen::Matrix<double, 4, 1>::Zero();
    if (frame.empty()) return signature;

    for (const auto &point : frame) {
        signature.head<3>() += point;
        signature[3] += point.norm();
    }

    signature /= static_cast<double>(frame.size());
    return signature;
}

bool IsStableFrameSignature(const Eigen::Matrix<double, 4, 1> &current,
                            const Eigen::Matrix<double, 4, 1> &reference,
                            double voxel_size) {
    const double centroid_epsilon = std::max(0.05, 0.10 * voxel_size);
    const double range_epsilon = std::max(0.05, 0.10 * voxel_size);
    return (current.head<3>() - reference.head<3>()).norm() < centroid_epsilon &&
           std::abs(current[3] - reference[3]) < range_epsilon;
}

bool IsSmallSolvedMotion(const Sophus::SE3d &model_deviation, double min_motion_threshold) {
    const double translation = model_deviation.translation().norm();
    const double rotation = Eigen::AngleAxisd(model_deviation.rotationMatrix()).angle();
    return translation < std::max(0.01, 0.25 * min_motion_threshold) && rotation < 0.005;
}

bool IsFinitePose(const Sophus::SE3d &pose) {
    return pose.matrix().allFinite();
}

double DeltaYawDegrees(const Sophus::SE3d &delta) {
    const Eigen::Matrix3d rotation = delta.rotationMatrix();
    const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    return std::abs(yaw) * kRadToDeg;
}

Eigen::Matrix<double, 6, 6> StartupCovariance() {
    return Eigen::Matrix<double, 6, 6>::Identity() * kStartupPoseVariance;
}

std::string FormatDouble(double value) {
    if (!std::isfinite(value)) return "nan";
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(4) << value;
    return stream.str();
}

std::string ToLower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

Eigen::Vector3d ParseVerticalAxis(const std::string &axis_name) {
    const auto normalized = ToLower(axis_name);
    if (normalized == "x" || normalized == "+x") return Eigen::Vector3d::UnitX();
    if (normalized == "-x") return -Eigen::Vector3d::UnitX();
    if (normalized == "y" || normalized == "+y") return Eigen::Vector3d::UnitY();
    if (normalized == "-y") return -Eigen::Vector3d::UnitY();
    if (normalized == "-z") return -Eigen::Vector3d::UnitZ();
    return Eigen::Vector3d::UnitZ();
}

std::vector<Eigen::Vector3d> TransformFrame(const Sophus::SE3d &transform,
                                            const std::vector<Eigen::Vector3d> &frame) {
    std::vector<Eigen::Vector3d> transformed(frame.size());
    std::transform(frame.cbegin(), frame.cend(), transformed.begin(),
                   [&](const auto &point) { return transform * point; });
    return transformed;
}

YawSearchCandidateScore ScoreYawCandidate(
    const std::vector<Eigen::Vector3d> &source,
    const genz_icp::VoxelHashMap &local_map,
    const Sophus::SE3d &candidate_guess,
    double offset_deg,
    double max_correspondence_distance,
    double kernel,
    size_t min_correspondences,
    bool use_weighted_rmse) {
    YawSearchCandidateScore score;
    score.offset_deg = offset_deg;
    score.initial_guess = candidate_guess;

    const auto transformed_source = TransformFrame(candidate_guess, source);
    const auto &[src_planar,
                 tgt_planar,
                 normals,
                 src_non_planar,
                 tgt_non_planar,
                 planar_count,
                 non_planar_count] =
        local_map.GetCorrespondences(transformed_source, max_correspondence_distance);

    const size_t correspondence_count = planar_count + non_planar_count;
    score.correspondence_count = correspondence_count;
    if (correspondence_count == 0) {
        return score;
    }

    const double alpha = static_cast<double>(planar_count) /
                         static_cast<double>(correspondence_count);
    const double kernel_squared = kernel * kernel;
    const auto robust_weight = [&](double residual_squared) {
        return kernel_squared / Square(kernel + residual_squared);
    };

    double unweighted_error = 0.0;
    double weighted_error = 0.0;
    double weighted_sum = 0.0;

    for (size_t i = 0; i < src_planar.size(); ++i) {
        const double residual = (src_planar[i] - tgt_planar[i]).dot(normals[i]);
        const double residual_squared = residual * residual;
        const double weight = alpha * robust_weight(residual_squared);
        unweighted_error += residual_squared;
        weighted_error += weight * residual_squared;
        weighted_sum += weight;
    }

    for (size_t i = 0; i < src_non_planar.size(); ++i) {
        const double residual_squared = (src_non_planar[i] - tgt_non_planar[i]).squaredNorm();
        const double weight = (1.0 - alpha) * robust_weight(residual_squared);
        unweighted_error += residual_squared;
        weighted_error += weight * residual_squared;
        weighted_sum += weight;
    }

    score.rmse =
        std::sqrt(unweighted_error / static_cast<double>(correspondence_count));
    if (weighted_sum > 0.0) {
        score.weighted_rmse = std::sqrt(weighted_error / weighted_sum);
    }
    score.score = use_weighted_rmse && std::isfinite(score.weighted_rmse)
                      ? score.weighted_rmse
                      : score.rmse;
    score.valid = correspondence_count >= min_correspondences &&
                  std::isfinite(score.score) &&
                  score.initial_guess.matrix().allFinite();
    return score;
}

std::optional<YawSearchCandidateScore> SelectYawSearchInitialGuess(
    const std::vector<Eigen::Vector3d> &source,
    const genz_icp::VoxelHashMap &local_map,
    const Sophus::SE3d &base_initial_guess,
    const genz_icp::pipeline::GenZConfig &config,
    double kernel) {
    if (!config.enable_yaw_search_initializer || local_map.Empty()) {
        return std::nullopt;
    }

    const Eigen::Vector3d yaw_axis = ParseVerticalAxis(config.yaw_search_vertical_axis);
    const std::vector<double> yaw_offsets =
        config.yaw_search_degrees.empty()
            ? std::vector<double>{0.0}
            : config.yaw_search_degrees;
    const size_t min_correspondences =
        static_cast<size_t>(std::max(0, config.yaw_search_min_correspondences));
    const double max_correspondence_distance =
        std::max(0.0, config.yaw_search_score_max_correspondence_distance);
    const double safe_kernel = std::max(1e-6, kernel);

    std::optional<YawSearchCandidateScore> best;
    for (const double offset_deg : yaw_offsets) {
        const Sophus::SE3d yaw_delta(
            Sophus::SO3d::exp(yaw_axis * offset_deg * kDegToRad),
            Eigen::Vector3d::Zero());
        const Sophus::SE3d candidate_guess = base_initial_guess * yaw_delta;
        const auto candidate = ScoreYawCandidate(source,
                                                 local_map,
                                                 candidate_guess,
                                                 offset_deg,
                                                 max_correspondence_distance,
                                                 safe_kernel,
                                                 min_correspondences,
                                                 config.yaw_search_use_weighted_rmse);

        if (config.yaw_search_debug) {
            std::cout << "Yaw search candidate: offset_deg=" << FormatDouble(offset_deg)
                      << ", correspondences=" << candidate.correspondence_count
                      << ", rmse=" << FormatDouble(candidate.rmse)
                      << ", weighted_rmse=" << FormatDouble(candidate.weighted_rmse)
                      << ", score=" << FormatDouble(candidate.score)
                      << "\n";
        }

        if (candidate.valid && (!best || candidate.score < best->score)) {
            best = candidate;
        }
    }

    if (config.yaw_search_debug) {
        if (best) {
            std::cout << "Yaw search selected: offset_deg="
                      << FormatDouble(best->offset_deg)
                      << ", score=" << FormatDouble(best->score)
                      << ", correspondences=" << best->correspondence_count
                      << "\n";
        } else {
            std::cout << "Yaw search selected: none; keeping base initial guess\n";
        }
    }

    return best;
}

std::optional<genz_icp::RegistrationMotionPriorConfig> BuildMotionPriorConfig(
    const genz_icp::pipeline::GenZConfig &config,
    size_t consecutive_registration_rejections,
    bool recovery_mode) {
    if (!config.enable_motion_prior) {
        return std::nullopt;
    }

    const bool recovery_active = recovery_mode || consecutive_registration_rejections > 0;
    if (recovery_active && !config.motion_prior_apply_during_recovery) {
        return std::nullopt;
    }

    genz_icp::RegistrationMotionPriorConfig prior;
    prior.enabled = true;
    prior.translation_sigma = std::max(1e-6, config.motion_prior_translation_sigma);
    prior.z_sigma = std::max(1e-6, config.motion_prior_z_sigma);
    prior.roll_pitch_sigma_rad =
        std::max(1e-6, config.motion_prior_roll_pitch_sigma_deg * kDegToRad);
    prior.yaw_sigma_rad =
        std::max(1e-6, config.motion_prior_yaw_sigma_deg * kDegToRad);
    prior.weight = std::max(0.0, config.motion_prior_weight);
    prior.debug = config.motion_prior_debug;
    if (prior.weight <= 0.0) {
        return std::nullopt;
    }
    return prior;
}
}  // namespace

namespace genz_icp::pipeline {

GenZICP::RegistrationTuple GenZICP::RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                                                  const std::vector<double> &timestamps) {
    return RegisterFrame(frame, timestamps, std::nullopt);
}

GenZICP::RegistrationTuple GenZICP::RegisterFrame(
    const std::vector<Eigen::Vector3d> &frame,
    const std::vector<double> &timestamps,
    const std::optional<Sophus::SO3d> &rotation_prediction) {
    const auto &deskew_frame = [&]() -> std::vector<Eigen::Vector3d> {
        if (!config_.deskew || timestamps.empty()) return frame;
        // TODO(Nacho) Add some asserts here to sanitize the timestamps

        //  If not enough poses for the estimation, do not de-skew
        const size_t N = poses().size();
        if (N <= 2) return frame;

        // Estimate linear and angular velocities
        const auto &start_pose = poses_[N - 2];
        const auto &finish_pose = poses_[N - 1];

        return DeSkewScan(frame, timestamps, start_pose, finish_pose);
        
    }();
    return RegisterFrame(deskew_frame, rotation_prediction);
}

GenZICP::RegistrationTuple GenZICP::RegisterFrame(const std::vector<Eigen::Vector3d> &frame) {
    return RegisterFrame(frame, std::nullopt);
}

GenZICP::RegistrationTuple GenZICP::RegisterFrame(
    const std::vector<Eigen::Vector3d> &frame,
    const std::optional<Sophus::SO3d> &rotation_prediction) {
    last_frame_accepted_ = false;

    // Preprocess the input cloud
    const auto &cropped_frame = Preprocess(frame, config_.max_range, config_.min_range);

    // Adapt voxel size based on LOCUS 2.0's adaptive voxel grid filter
    const auto source_tmp = genz_icp::VoxelDownsample(cropped_frame, adaptive_voxel_size_);
    const double desired_points = static_cast<double>(std::max(1, config_.desired_num_voxelized_points));
    double adaptive_voxel_size = genz_icp::Clamp(
        adaptive_voxel_size_ * static_cast<double>(source_tmp.size()) / desired_points, 0.02, 2.0);

    // Re-voxelize using the adaptive voxel size
    const auto &[source, frame_downsample] = Voxelize(cropped_frame, adaptive_voxel_size);
    adaptive_voxel_size_ = adaptive_voxel_size; // Save for the next frame

    const auto frame_signature = ComputeFrameSignature(source);
    if (local_map_.Empty()) {
        const Sophus::SE3d seed_pose = !poses_.empty() ? poses_.back() : Sophus::SE3d();
        local_map_.Update(frame_downsample, seed_pose);
        ++frame_index_;
        if (!has_initial_pose_) {
            initial_pose_ = seed_pose;
            has_initial_pose_ = true;
        }
        PushPose(seed_pose);
        last_covariance_ = StartupCovariance();
        last_registered_signature_ = frame_signature;
        has_last_registered_signature_ = true;
        stationary_frame_count_ = 1;
        skipped_stationary_frames_ = 0;
        last_frame_accepted_ = true;
        return std::make_tuple(Vector3dVector{}, Vector3dVector{}, last_covariance_);
    }

    // Get motion prediction and adaptive_threshold
    const double sigma = GetAdaptiveThreshold();

    if (!poses_.empty() &&
        has_last_registered_signature_ &&
        stationary_frame_count_ >= kStationaryWarmupFrames &&
        skipped_stationary_frames_ < kMaxStationarySkippedFrames &&
        IsStableFrameSignature(frame_signature, last_registered_signature_, adaptive_voxel_size)) {
        PushPose(poses_.back());
        ++skipped_stationary_frames_;
        last_frame_accepted_ = true;
        return std::make_tuple(Vector3dVector{}, Vector3dVector{}, last_covariance_);
    }

    // Compute initial_guess for ICP
    const auto normal_prediction = GetPredictionModel();
    const Sophus::SE3d prediction =
        rotation_prediction
            ? Sophus::SE3d(*rotation_prediction, normal_prediction.translation())
            : normal_prediction;
    const auto last_pose = !poses_.empty() ? poses_.back() : Sophus::SE3d();
    const auto base_initial_guess = last_pose * prediction;
    Sophus::SE3d initial_guess = base_initial_guess;
    bool yaw_search_selected = false;
    std::optional<VoxelHashMap> registration_map_with_tentative;
    const VoxelHashMap *registration_map = &local_map_;
    if (config_.enable_tentative_map_gating && config_.use_tentative_points_for_icp) {
        registration_map_with_tentative = BuildRegistrationMap();
        registration_map = &(*registration_map_with_tentative);
    }
    if (config_.enable_yaw_search_initializer) {
        const auto yaw_search_result =
            SelectYawSearchInitialGuess(source,
                                        *registration_map,
                                        base_initial_guess,
                                        config_,
                                        sigma / 3.0);
        if (yaw_search_result) {
            initial_guess = yaw_search_result->initial_guess;
            yaw_search_selected = true;
        }
    }

    if (config_.enable_yaw_search_initializer && config_.yaw_search_debug) {
        std::cout << "ICP initial guess source: "
                  << (rotation_prediction ? "imu_prediction" : "normal")
                  << (yaw_search_selected ? "+yaw_search" : "")
                  << "\n";
    }

    const auto motion_prior =
        BuildMotionPriorConfig(config_,
                               consecutive_registration_rejections_,
                               registration_recovery_mode_);
    const auto robust_icp =
        config_.enable_robust_icp_outlier_handling
            ? std::optional<genz_icp::RegistrationRobustICPConfig>(BuildRobustICPConfig())
            : std::nullopt;

    // Run GenZ-ICP and evaluate the complete candidate before committing it.
    auto registration_result = registration_.RegisterFrameWithQuality(source,         //
                                                                      *registration_map, //
                                                                      initial_guess,  //
                                                                      3.0 * sigma,    //
                                                                      sigma / 3.0,    //
                                                                      motion_prior,
                                                                      robust_icp);
    const auto &new_pose = registration_result.pose;
    const auto &covariance = registration_result.covariance;

    auto quality = registration_result.quality;
    const auto accepted_delta = last_pose.inverse() * new_pose;
    quality.translation_delta = accepted_delta.translation().norm();
    quality.rotation_delta = accepted_delta.so3().log().norm();
    quality.finite = quality.finite &&
                     IsFinitePose(new_pose) &&
                     accepted_delta.matrix().allFinite() &&
                     covariance.allFinite() &&
                     std::isfinite(quality.translation_delta) &&
                     std::isfinite(quality.rotation_delta);

    bool log_accept = false;
    if (config_.enable_registration_quality_gate) {
        std::string rejection_reason;
        if (!IsRegistrationAcceptable(quality, rejection_reason)) {
            ++consecutive_registration_rejections_;
            if (config_.max_consecutive_registration_rejections > 0 &&
                consecutive_registration_rejections_ >=
                    static_cast<size_t>(config_.max_consecutive_registration_rejections)) {
                registration_recovery_mode_ = true;
                rejection_reason +=
                    "; recovery mode active: odometry frozen and map update skipped";
            }
            LogRegistrationDecision(false, quality, rejection_reason, true);
            return std::make_tuple(Vector3dVector{}, Vector3dVector{}, last_covariance_);
        }

        log_accept =
            terminal_status_enabled_ || consecutive_registration_rejections_ > 0 ||
            registration_recovery_mode_;
    }

    std::string map_update_rejection_reason;
    const bool update_map =
        IsMapUpdateAcceptable(quality, map_update_rejection_reason);
    LogMapUpdateDecision(update_map,
                         quality,
                         map_update_rejection_reason,
                         config_.map_update_debug && (!update_map || terminal_status_enabled_));

    UpdateRegistrationQualityEma(quality);
    LogRegistrationDecision(true, quality, "", log_accept);
    consecutive_registration_rejections_ = 0;
    registration_recovery_mode_ = false;

    const auto model_deviation = initial_guess.inverse() * new_pose;
    adaptive_threshold_.UpdateModelDeviation(model_deviation);
    if (update_map) {
        if (config_.enable_tentative_map_gating) {
            UpdateMapWithTentativeGating(frame_downsample,
                                         new_pose,
                                         DeltaYawDegrees(accepted_delta));
        } else {
            local_map_.Update(frame_downsample, new_pose);
            ++frame_index_;
        }
    }
    if (!has_initial_pose_) {
        initial_pose_ = new_pose;
        has_initial_pose_ = true;
    }
    PushPose(new_pose);
    last_covariance_ = covariance;
    last_registered_signature_ = frame_signature;
    has_last_registered_signature_ = true;
    skipped_stationary_frames_ = 0;
    if (IsSmallSolvedMotion(model_deviation, config_.min_motion_th)) {
        ++stationary_frame_count_;
    } else {
        stationary_frame_count_ = 0;
    }
    last_frame_accepted_ = true;

    return std::make_tuple(std::move(registration_result.planar_points),
                           std::move(registration_result.non_planar_points),
                           covariance);
}

GenZICP::Vector3dVectorTuple GenZICP::Voxelize(const std::vector<Eigen::Vector3d> &frame, double adaptive_voxel_size) const {
    const auto frame_downsample = genz_icp::VoxelDownsample(frame, std::max(adaptive_voxel_size * 0.5, 0.02)); // localmap update
    const auto source = genz_icp::VoxelDownsample(frame_downsample, adaptive_voxel_size * 1.0); // registration
    return {source, frame_downsample};
}

double GenZICP::GetAdaptiveThreshold() {
    if (!HasMoved()) {
        return config_.initial_threshold;
    }
    return adaptive_threshold_.ComputeThreshold();
}

Sophus::SE3d GenZICP::GetPredictionModel() const {
    Sophus::SE3d pred = Sophus::SE3d();
    const size_t N = poses_.size();
    if (N < 2) return pred;
    return poses_[N - 2].inverse() * poses_[N - 1];
}

bool GenZICP::HasMoved() {
    if (!has_initial_pose_ || poses_.empty()) return false;
    const double motion = (initial_pose_.inverse() * poses_.back()).translation().norm();
    return motion > 5.0 * config_.min_motion_th;
}

bool GenZICP::IsRegistrationAcceptable(const genz_icp::RegistrationQuality &quality,
                                       std::string &reason) const {
    if (!quality.finite) {
        reason = "non-finite registration output";
        return false;
    }

    const size_t min_correspondences =
        static_cast<size_t>(std::max(0, config_.min_registration_correspondences));
    if (quality.correspondence_count < min_correspondences) {
        reason = "too few correspondences";
        return false;
    }

    if (config_.max_registration_translation_per_frame >= 0.0 &&
        quality.translation_delta > config_.max_registration_translation_per_frame) {
        reason = "translation jump exceeds limit";
        return false;
    }

    const double max_rotation_rad =
        std::max(0.0, config_.max_registration_rotation_per_frame_deg) * kDegToRad;
    if (quality.rotation_delta > max_rotation_rad) {
        reason = "rotation jump exceeds limit";
        return false;
    }

    const bool rmse_gate_ready =
        registration_rmse_ema_initialized_ &&
        accepted_registration_count_ >= kRegistrationQualityWarmupAcceptedFrames;
    if (rmse_gate_ready) {
        const double ratio = std::max(1.0, config_.registration_rmse_reject_ratio);
        const double absolute_limit = std::max(0.0, config_.absolute_registration_rmse_limit);
        const double adaptive_limit = std::max(absolute_limit, ratio * registration_rmse_ema_);
        if (quality.rmse > adaptive_limit) {
            reason = "registration RMSE exceeds adaptive limit";
            return false;
        }
    }

    return true;
}

bool GenZICP::IsMapUpdateAcceptable(const genz_icp::RegistrationQuality &quality,
                                    std::string &reason) const {
    if (!config_.enable_map_update_quality_gate) {
        return true;
    }

    const size_t min_correspondences =
        static_cast<size_t>(std::max(0, config_.map_update_min_correspondences));
    if (quality.correspondence_count < min_correspondences) {
        reason = "too few correspondences for map update";
        return false;
    }

    if (config_.map_update_max_rmse >= 0.0 &&
        quality.rmse > config_.map_update_max_rmse) {
        reason = "RMSE exceeds map update limit";
        return false;
    }

    if (config_.map_update_max_translation_delta >= 0.0 &&
        quality.translation_delta > config_.map_update_max_translation_delta) {
        reason = "translation delta exceeds map update limit";
        return false;
    }

    const double max_rotation_rad =
        std::max(0.0, config_.map_update_max_rotation_delta_deg) * kDegToRad;
    if (config_.map_update_max_rotation_delta_deg >= 0.0 &&
        quality.rotation_delta > max_rotation_rad) {
        reason = "rotation delta exceeds map update limit";
        return false;
    }

    if (registration_weighted_rmse_ema_initialized_ &&
        config_.map_update_max_weighted_rmse_ratio > 0.0 &&
        std::isfinite(quality.weighted_rmse)) {
        const double weighted_rmse_limit =
            config_.map_update_max_weighted_rmse_ratio * registration_weighted_rmse_ema_;
        if (quality.weighted_rmse > weighted_rmse_limit) {
            reason = "weighted RMSE exceeds map update ratio";
            return false;
        }
    }

    return true;
}

void GenZICP::UpdateRegistrationQualityEma(const genz_icp::RegistrationQuality &quality) {
    if (!std::isfinite(quality.rmse)) return;

    if (!registration_rmse_ema_initialized_) {
        registration_rmse_ema_ = quality.rmse;
        registration_rmse_ema_initialized_ = true;
    } else {
        const double alpha = std::clamp(config_.registration_rmse_ema_alpha, 0.0, 1.0);
        registration_rmse_ema_ = alpha * quality.rmse + (1.0 - alpha) * registration_rmse_ema_;
    }

    if (std::isfinite(quality.weighted_rmse)) {
        if (!registration_weighted_rmse_ema_initialized_) {
            registration_weighted_rmse_ema_ = quality.weighted_rmse;
            registration_weighted_rmse_ema_initialized_ = true;
        } else {
            const double alpha = std::clamp(config_.registration_rmse_ema_alpha, 0.0, 1.0);
            registration_weighted_rmse_ema_ =
                alpha * quality.weighted_rmse + (1.0 - alpha) * registration_weighted_rmse_ema_;
        }
    }
    ++accepted_registration_count_;
}

void GenZICP::LogRegistrationDecision(bool accepted,
                                      const genz_icp::RegistrationQuality &quality,
                                      const std::string &reason,
                                      bool force) const {
    if (!force) return;

    const std::string ema = registration_rmse_ema_initialized_
                                ? FormatDouble(registration_rmse_ema_)
                                : std::string("uninitialized");
    const double rotation_delta_deg = quality.rotation_delta * kRadToDeg;

    std::ostream &stream = accepted ? std::cout : std::cerr;
    stream << "Registration " << (accepted ? "accepted" : "rejected") << ": ";
    if (!accepted) {
        stream << "reason=" << reason << ", ";
    }
    stream << "rmse=" << FormatDouble(quality.rmse)
           << ", weighted_rmse=" << FormatDouble(quality.weighted_rmse)
           << ", ema=" << ema
           << ", correspondences=" << quality.correspondence_count
           << ", trans_delta=" << FormatDouble(quality.translation_delta)
           << ", rot_delta_deg=" << FormatDouble(rotation_delta_deg)
           << ", consecutive_rejections=" << consecutive_registration_rejections_
           << "\n";
}

void GenZICP::LogMapUpdateDecision(bool accepted,
                                   const genz_icp::RegistrationQuality &quality,
                                   const std::string &reason,
                                   bool force) const {
    if (!force || !config_.enable_map_update_quality_gate) return;

    const std::string ema = registration_weighted_rmse_ema_initialized_
                                ? FormatDouble(registration_weighted_rmse_ema_)
                                : std::string("uninitialized");
    const double rotation_delta_deg = quality.rotation_delta * kRadToDeg;

    if (accepted) {
        std::cout << "Map update accepted: rmse=" << FormatDouble(quality.rmse)
                  << ", weighted_rmse=" << FormatDouble(quality.weighted_rmse)
                  << ", ema=" << ema
                  << ", correspondences=" << quality.correspondence_count
                  << ", trans_delta=" << FormatDouble(quality.translation_delta)
                  << ", rot_delta_deg=" << FormatDouble(rotation_delta_deg)
                  << "\n";
        return;
    }

    std::cout << "Map update skipped: reason=" << reason
              << ", rmse=" << FormatDouble(quality.rmse)
              << ", weighted_rmse=" << FormatDouble(quality.weighted_rmse)
              << ", ema=" << ema
              << ", correspondences=" << quality.correspondence_count
              << ", trans_delta=" << FormatDouble(quality.translation_delta)
              << ", rot_delta_deg=" << FormatDouble(rotation_delta_deg)
              << "\n";
}

RegistrationRobustICPConfig GenZICP::BuildRobustICPConfig() const {
    RegistrationRobustICPConfig robust;
    robust.enabled = config_.enable_robust_icp_outlier_handling;
    robust.max_correspondence_distance =
        std::max(0.0, config_.robust_max_correspondence_distance);
    robust.residual_threshold =
        std::max(1e-6, config_.robust_residual_threshold);
    robust.loss_type = config_.robust_loss_type;
    robust.trimmed_icp_enabled = config_.trimmed_icp_enabled;
    robust.trimmed_icp_keep_ratio =
        std::clamp(config_.trimmed_icp_keep_ratio, 0.01, 1.0);
    robust.min_correspondences =
        std::max(0, config_.robust_min_correspondences);
    robust.debug = config_.robust_icp_debug;
    return robust;
}

VoxelHashMap GenZICP::BuildRegistrationMap() const {
    VoxelHashMap registration_map = local_map_;
    if (config_.enable_tentative_map_gating && config_.use_tentative_points_for_icp) {
        registration_map.AddPoints(TentativePointcloud());
    }
    return registration_map;
}

GenZICP::Vector3dVector GenZICP::TentativePointcloud() const {
    Vector3dVector points;
    points.reserve(tentative_map_.size());
    for (const auto &[voxel, tentative] : tentative_map_) {
        (void)voxel;
        if (tentative.centroid.allFinite()) {
            points.push_back(tentative.centroid);
        }
    }
    return points;
}

void GenZICP::UpdateMapWithTentativeGating(
    const std::vector<Eigen::Vector3d> &frame_downsample,
    const Sophus::SE3d &pose,
    double delta_yaw_deg) {
    ++frame_index_;

    TentativeMapStats stats;
    stats.current_frame_points = frame_downsample.size();
    stats.delta_yaw_deg = delta_yaw_deg;
    stats.gating_mode = TentativeModeForYaw(delta_yaw_deg);

    const size_t stable_map_points = local_map_.PointCount();
    const size_t min_stable_map_points =
        static_cast<size_t>(std::max(0, config_.map_update_min_stable_map_points));
    if (config_.map_update_allow_new_points_when_map_is_small &&
        stable_map_points < min_stable_map_points) {
        local_map_.Update(frame_downsample, pose);
        stats.stable_map_insertions = frame_downsample.size();
        if (config_.tentative_map_debug) {
            LogTentativeMapStats(stats);
        }
        return;
    }

    const auto transformed_points = TransformFrame(pose, frame_downsample);
    std::vector<Eigen::Vector3d> stable_insertions;
    stable_insertions.reserve(transformed_points.size());
    std::vector<Eigen::Vector3d> promoted_points;

    const double support_radius =
        std::max(0.0, config_.tentative_stable_support_radius);
    for (const auto &point : transformed_points) {
        if (!point.allFinite()) continue;

        if (local_map_.HasNeighborWithin(point, support_radius)) {
            ++stats.stable_supported_points;
            stable_insertions.push_back(point);
            continue;
        }

        if (config_.insert_new_points_as_tentative) {
            UpdateTentativeVoxel(point, frame_index_, promoted_points, stats);
        }
    }

    stable_insertions.insert(stable_insertions.end(),
                             promoted_points.begin(),
                             promoted_points.end());
    stats.stable_map_insertions = stable_insertions.size();
    local_map_.Update(stable_insertions, pose.translation());

    if (stats.gating_mode == TentativeGatingMode::Normal) {
        ExpireTentativeVoxels(frame_index_, stats);
    }

    if (config_.tentative_map_debug) {
        LogTentativeMapStats(stats);
    }
}

void GenZICP::UpdateTentativeVoxel(const Eigen::Vector3d &point,
                                   size_t frame_index,
                                   std::vector<Eigen::Vector3d> &promoted_points,
                                   TentativeMapStats &stats) {
    const auto voxel = TentativeVoxelForPoint(point);
    auto search = tentative_map_.find(voxel);
    if (search == tentative_map_.end()) {
        TentativeVoxel tentative;
        tentative.centroid = point;
        tentative.observation_count = 1;
        tentative.first_seen_frame = frame_index;
        tentative.last_seen_frame = frame_index;
        search = tentative_map_.insert({voxel, tentative}).first;
    } else {
        auto &tentative = search.value();
        if (tentative.last_seen_frame != frame_index) {
            ++tentative.observation_count;
            tentative.last_seen_frame = frame_index;
            const double alpha = 1.0 / static_cast<double>(
                                           std::max(1, tentative.observation_count));
            tentative.centroid = (1.0 - alpha) * tentative.centroid + alpha * point;
        } else {
            tentative.centroid = 0.5 * (tentative.centroid + point);
        }
    }
    ++stats.tentative_updated_points;

    const bool calm_enough =
        stats.gating_mode == TentativeGatingMode::Normal ||
        (!config_.promote_tentative_only_when_motion_is_calm &&
         stats.gating_mode == TentativeGatingMode::Relaxed);
    const int required_observations =
        std::max(1, config_.tentative_required_observations);
    if (!calm_enough ||
        search->second.observation_count < required_observations) {
        return;
    }

    promoted_points.push_back(search->second.centroid);
    tentative_map_.erase(search);
    ++stats.tentative_promoted_points;
}

void GenZICP::ExpireTentativeVoxels(size_t frame_index, TentativeMapStats &stats) {
    const size_t max_age_frames =
        static_cast<size_t>(std::max(1, config_.tentative_max_age_frames));
    for (auto it = tentative_map_.begin(); it != tentative_map_.end();) {
        const size_t age =
            frame_index > it->second.last_seen_frame
                ? frame_index - it->second.last_seen_frame
                : 0;
        if (age > max_age_frames) {
            it = tentative_map_.erase(it);
            ++stats.tentative_expired_points;
        } else {
            ++it;
        }
    }
}

VoxelHashMap::Voxel GenZICP::TentativeVoxelForPoint(const Eigen::Vector3d &point) const {
    const double voxel_size = std::max(1e-6, config_.tentative_voxel_size);
    return VoxelHashMap::Voxel(
        static_cast<int>(std::floor(point.x() / voxel_size)),
        static_cast<int>(std::floor(point.y() / voxel_size)),
        static_cast<int>(std::floor(point.z() / voxel_size)));
}

GenZICP::TentativeGatingMode GenZICP::TentativeModeForYaw(double delta_yaw_deg) const {
    const double normal_limit =
        std::max(0.0, config_.dynamic_enable_max_delta_yaw_deg);
    const double relaxed_limit =
        std::max(normal_limit, config_.dynamic_relax_max_delta_yaw_deg);
    const double abs_yaw = std::abs(delta_yaw_deg);
    if (abs_yaw <= normal_limit) {
        return TentativeGatingMode::Normal;
    }
    if (abs_yaw <= relaxed_limit) {
        return TentativeGatingMode::Relaxed;
    }
    return TentativeGatingMode::Frozen;
}

const char *GenZICP::TentativeModeName(TentativeGatingMode mode) const {
    switch (mode) {
        case TentativeGatingMode::Normal:
            return "normal";
        case TentativeGatingMode::Relaxed:
            return "relaxed";
        case TentativeGatingMode::Frozen:
            return "frozen";
    }
    return "normal";
}

void GenZICP::LogTentativeMapStats(const TentativeMapStats &stats) const {
    std::cout << "Tentative map: current_frame_points=" << stats.current_frame_points
              << ", stable_supported_points=" << stats.stable_supported_points
              << ", tentative_updated_points=" << stats.tentative_updated_points
              << ", tentative_promoted_points=" << stats.tentative_promoted_points
              << ", tentative_expired_points=" << stats.tentative_expired_points
              << ", stable_map_insertions=" << stats.stable_map_insertions
              << ", delta_yaw_deg=" << FormatDouble(stats.delta_yaw_deg)
              << ", gating_mode=" << TentativeModeName(stats.gating_mode)
              << ", tentative_voxels=" << tentative_map_.size()
              << "\n";
}

void GenZICP::PushPose(const Sophus::SE3d &pose) {
    poses_.push_back(pose);
    const size_t max_pose_history = std::max<size_t>(2, config_.max_pose_history);
    while (poses_.size() > max_pose_history) {
        poses_.pop_front();
    }
}

}  // namespace genz_icp::pipeline
