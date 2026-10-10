#include "Registration.hpp"

#include <Eigen/Cholesky>
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <numeric>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <string>
#include <tuple>
#include <iostream>
#include <utility>

namespace Eigen {
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
}  // namespace Eigen

namespace {

inline double square(double x) { return x * x; }

constexpr double kHighUncertainty = 1.0;
constexpr double kMinInformationPivot = 1e-12;
constexpr double kRelativeRegularization = 1e-9;
constexpr double kMinResidualVariance = 1e-6;
constexpr double kMaxResidualVariance = 1e3;
constexpr double kErrorConvergenceRelativeTolerance = 1e-5;
constexpr double kMinMotionPriorSigma = 1e-6;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

Eigen::Matrix6d HighUncertaintyCovariance() {
    return Eigen::Matrix6d::Identity() * kHighUncertainty;
}

Eigen::Matrix6d CovarianceFromInformation(const Eigen::Matrix6d &information,
                                          double weighted_error,
                                          size_t residual_count) {
    if (!information.allFinite()) {
        return HighUncertaintyCovariance();
    }

    Eigen::Matrix6d sym_information = 0.5 * (information + information.transpose());
    Eigen::LDLT<Eigen::Matrix6d> ldlt(sym_information);

    if (ldlt.info() != Eigen::Success ||
        (ldlt.vectorD().array() <= kMinInformationPivot).any()) {
        const double max_diagonal = sym_information.diagonal().cwiseAbs().maxCoeff();
        const double regularization =
            std::max(kMinInformationPivot, kRelativeRegularization * max_diagonal);

        sym_information.diagonal().array() += regularization;
        ldlt.compute(sym_information);
    }

    if (ldlt.info() != Eigen::Success ||
        (ldlt.vectorD().array() <= kMinInformationPivot).any()) {
        return HighUncertaintyCovariance();
    }

    const double dof = static_cast<double>(residual_count > 6 ? residual_count - 6 : 1);
    const double residual_variance =
        std::clamp(weighted_error / dof, kMinResidualVariance, kMaxResidualVariance);

    Eigen::Matrix6d covariance = residual_variance * ldlt.solve(Eigen::Matrix6d::Identity());
    if (!covariance.allFinite()) {
        return HighUncertaintyCovariance();
    }

    return 0.5 * (covariance + covariance.transpose());
}

bool IsFinitePose(const Sophus::SE3d &pose) {
    return pose.matrix().allFinite();
}

void TransformPoints(const Sophus::SE3d &T, std::vector<Eigen::Vector3d> &points) {
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const auto &point) { return T * point; });
}

struct LinearSystemResult {
    Eigen::Matrix6d JTJ = Eigen::Matrix6d::Zero();
    Eigen::Vector6d JTr = Eigen::Vector6d::Zero();
    double unweighted_error = 0.0;
    double weighted_error = 0.0;
    double weighted_sum = 0.0;
    size_t residual_count = 0;
    size_t correspondence_count = 0;
};

struct RobustResidualEntry {
    bool planar = false;
    size_t index = 0;
    double residual = 0.0;
    double correspondence_distance = 0.0;
};

struct RobustICPSelection {
    std::vector<uint8_t> planar_mask;
    std::vector<uint8_t> non_planar_mask;
    size_t input_correspondences = 0;
    size_t rejected_by_max_distance = 0;
    size_t kept_after_trimmed_icp = 0;
    size_t final_correspondences = 0;
    size_t final_planar_count = 0;
    size_t final_non_planar_count = 0;
    double mean_residual = 0.0;
    double median_residual = 0.0;
    bool fallback_to_normal_set = false;
};

bool MotionPriorIsActive(const genz_icp::RegistrationMotionPriorConfig &prior) {
    return prior.enabled &&
           prior.weight > 0.0 &&
           std::isfinite(prior.weight) &&
           std::isfinite(prior.translation_sigma) &&
           std::isfinite(prior.z_sigma) &&
           std::isfinite(prior.roll_pitch_sigma_rad) &&
           std::isfinite(prior.yaw_sigma_rad);
}

Eigen::Matrix6d MotionPriorInformation(const genz_icp::RegistrationMotionPriorConfig &prior) {
    Eigen::Matrix6d information = Eigen::Matrix6d::Zero();
    const double sigma_xy = std::max(kMinMotionPriorSigma, prior.translation_sigma);
    const double sigma_z = std::max(kMinMotionPriorSigma, prior.z_sigma);
    const double sigma_roll_pitch = std::max(kMinMotionPriorSigma, prior.roll_pitch_sigma_rad);
    const double sigma_yaw = std::max(kMinMotionPriorSigma, prior.yaw_sigma_rad);

    information(0, 0) = prior.weight / square(sigma_xy);
    information(1, 1) = prior.weight / square(sigma_xy);
    information(2, 2) = prior.weight / square(sigma_z);
    information(3, 3) = prior.weight / square(sigma_roll_pitch);
    information(4, 4) = prior.weight / square(sigma_roll_pitch);
    information(5, 5) = prior.weight / square(sigma_yaw);
    return information;
}

std::string ToLower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

double RobustLossWeight(const genz_icp::RegistrationRobustICPConfig &config,
                        double residual) {
    const double delta = std::max(1e-6, config.residual_threshold);
    residual = std::abs(residual);
    const std::string loss_type = ToLower(config.loss_type);

    if (loss_type == "none") {
        return 1.0;
    }
    if (loss_type == "huber") {
        return residual <= delta ? 1.0 : delta / residual;
    }
    if (loss_type == "tukey") {
        if (residual >= delta) return 0.0;
        const double ratio = residual / delta;
        return square(1.0 - ratio * ratio);
    }

    // Default to Cauchy for unknown values because it is conservative and smooth.
    const double ratio = residual / delta;
    return 1.0 / (1.0 + ratio * ratio);
}

RobustICPSelection SelectRobustCorrespondences(
    const std::vector<Eigen::Vector3d> &src_planar,
    const std::vector<Eigen::Vector3d> &tgt_planar,
    const std::vector<Eigen::Vector3d> &normals,
    const std::vector<Eigen::Vector3d> &src_non_planar,
    const std::vector<Eigen::Vector3d> &tgt_non_planar,
    const genz_icp::RegistrationRobustICPConfig &config) {
    RobustICPSelection selection;
    selection.input_correspondences = src_planar.size() + src_non_planar.size();
    selection.planar_mask.assign(src_planar.size(), 0);
    selection.non_planar_mask.assign(src_non_planar.size(), 0);

    std::vector<RobustResidualEntry> entries;
    entries.reserve(selection.input_correspondences);
    std::vector<double> residuals;
    residuals.reserve(selection.input_correspondences);

    for (size_t i = 0; i < src_planar.size(); ++i) {
        const double residual = std::abs((src_planar[i] - tgt_planar[i]).dot(normals[i]));
        const double correspondence_distance = (src_planar[i] - tgt_planar[i]).norm();
        entries.push_back({true, i, residual, correspondence_distance});
        residuals.push_back(residual);
    }
    for (size_t i = 0; i < src_non_planar.size(); ++i) {
        const Eigen::Vector3d residual = src_non_planar[i] - tgt_non_planar[i];
        const double residual_norm = residual.norm();
        entries.push_back({false, i, residual_norm, residual_norm});
        residuals.push_back(residual_norm);
    }

    if (!residuals.empty()) {
        const double residual_sum =
            std::accumulate(residuals.cbegin(), residuals.cend(), 0.0);
        selection.mean_residual = residual_sum / static_cast<double>(residuals.size());
        std::sort(residuals.begin(), residuals.end());
        selection.median_residual = residuals[residuals.size() / 2];
    }

    std::vector<RobustResidualEntry> candidates;
    candidates.reserve(entries.size());
    const bool use_max_distance =
        std::isfinite(config.max_correspondence_distance) &&
        config.max_correspondence_distance > 0.0;
    for (const auto &entry : entries) {
        if (use_max_distance &&
            entry.correspondence_distance > config.max_correspondence_distance) {
            ++selection.rejected_by_max_distance;
            continue;
        }
        candidates.push_back(entry);
    }

    if (config.trimmed_icp_enabled && !candidates.empty()) {
        const double keep_ratio = std::clamp(config.trimmed_icp_keep_ratio, 0.01, 1.0);
        size_t keep_count =
            static_cast<size_t>(std::ceil(keep_ratio * static_cast<double>(candidates.size())));
        const size_t min_correspondences =
            static_cast<size_t>(std::max(0, config.min_correspondences));
        keep_count = std::max(keep_count, std::min(min_correspondences, candidates.size()));
        keep_count = std::min(keep_count, candidates.size());

        std::sort(candidates.begin(), candidates.end(),
                  [](const RobustResidualEntry &lhs, const RobustResidualEntry &rhs) {
                      return lhs.residual < rhs.residual;
                  });
        candidates.resize(keep_count);
    }
    selection.kept_after_trimmed_icp = candidates.size();

    const size_t min_correspondences =
        static_cast<size_t>(std::max(0, config.min_correspondences));
    if (min_correspondences > 0 && candidates.size() < min_correspondences) {
        selection.fallback_to_normal_set = true;
        candidates = std::move(entries);
    }

    for (const auto &entry : candidates) {
        if (entry.planar) {
            selection.planar_mask[entry.index] = 1;
            ++selection.final_planar_count;
        } else {
            selection.non_planar_mask[entry.index] = 1;
            ++selection.final_non_planar_count;
        }
    }
    selection.final_correspondences =
        selection.final_planar_count + selection.final_non_planar_count;
    return selection;
}

Eigen::Vector6d MotionPriorError(const Sophus::SE3d &prior_pose,
                                 const Sophus::SE3d &current_pose) {
    return (prior_pose.inverse() * current_pose).log();
}

void AddMotionPrior(LinearSystemResult &linear_system,
                    const Eigen::Matrix6d &prior_information,
                    const Sophus::SE3d &prior_pose,
                    const Sophus::SE3d &current_pose) {
    const Eigen::Vector6d error = MotionPriorError(prior_pose, current_pose);
    if (!error.allFinite()) return;

    // Identity-Jacobian prior in the same [translation, rotation] tangent order
    // used by the ICP point residuals. With dx = -H^-1 g, +W*e pulls e toward 0.
    linear_system.JTJ.noalias() += prior_information;
    linear_system.JTr.noalias() += prior_information * error;
}

void LogMotionPriorConfig(const genz_icp::RegistrationMotionPriorConfig &prior) {
    std::cout << "Motion prior enabled: sigma_xy=" << prior.translation_sigma
              << ", sigma_z=" << prior.z_sigma
              << ", sigma_rp_deg=" << prior.roll_pitch_sigma_rad * kRadToDeg
              << ", sigma_yaw_deg=" << prior.yaw_sigma_rad * kRadToDeg
              << ", weight=" << prior.weight
              << "\n";
}

void LogMotionPriorDeviation(const std::string &prefix, const Eigen::Vector6d &error) {
    const double trans_norm = error.head<3>().norm();
    const double rot_deg = error.tail<3>().norm() * kRadToDeg;
    const double roll_pitch_deg = error.segment<2>(3).norm() * kRadToDeg;
    const double yaw_deg = std::abs(error[5]) * kRadToDeg;
    std::cout << prefix
              << " trans_norm=" << trans_norm
              << ", z=" << error[2]
              << ", rot_deg=" << rot_deg
              << ", roll_pitch_deg=" << roll_pitch_deg
              << ", yaw_deg=" << yaw_deg
              << "\n";
}

//Build the linear system for the GenZ-ICP
LinearSystemResult BuildLinearSystem(
    const std::vector<Eigen::Vector3d> &src_planar,
    const std::vector<Eigen::Vector3d> &tgt_planar,
    const std::vector<Eigen::Vector3d> &normals,
    const std::vector<Eigen::Vector3d> &src_non_planar,
    const std::vector<Eigen::Vector3d> &tgt_non_planar,
    double kernel,
    double alpha,
    const genz_icp::RegistrationRobustICPConfig *robust_icp = nullptr,
    const std::vector<uint8_t> *planar_mask = nullptr,
    const std::vector<uint8_t> *non_planar_mask = nullptr) {

    struct ResultTuple {
        Eigen::Matrix6d JTJ;
        Eigen::Vector6d JTr;
        double unweighted_error = 0.0;
        double weighted_error = 0.0;
        double weighted_sum = 0.0;
        size_t residual_count = 0;
        size_t correspondence_count = 0;

        ResultTuple() : JTJ(Eigen::Matrix6d::Zero()), JTr(Eigen::Vector6d::Zero()) {}

        ResultTuple operator+(const ResultTuple &other) const {
            ResultTuple result;
            result.JTJ = JTJ + other.JTJ;
            result.JTr = JTr + other.JTr;
            result.unweighted_error = unweighted_error + other.unweighted_error;
            result.weighted_error = weighted_error + other.weighted_error;
            result.weighted_sum = weighted_sum + other.weighted_sum;
            result.residual_count = residual_count + other.residual_count;
            result.correspondence_count = correspondence_count + other.correspondence_count;
            return result;
        }
    };

    // Point-to-Plane Jacobian and Residual
    auto compute_jacobian_and_residual_planar = [&](auto i) {
        double r_planar = (src_planar[i] - tgt_planar[i]).dot(normals[i]); // residual
        Eigen::Matrix<double, 1, 6> J_planar; // Jacobian matrix
        J_planar.block<1, 3>(0, 0) = normals[i].transpose(); 
        J_planar.block<1, 3>(0, 3) = (src_planar[i].cross(normals[i])).transpose();
        return std::make_tuple(J_planar, r_planar);
    };

    // Point-to-Point Jacobian and Residual
    auto compute_jacobian_and_residual_non_planar = [&](auto i) {
        const Eigen::Vector3d r_non_planar = src_non_planar[i] - tgt_non_planar[i];
        Eigen::Matrix3_6d J_non_planar;
        J_non_planar.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_non_planar.block<3, 3>(0, 3) = -1.0 * Sophus::SO3d::hat(src_non_planar[i]);
        return std::make_tuple(J_non_planar, r_non_planar);
    };

    double kernel_squared = kernel * kernel;
    auto compute = [&](const tbb::blocked_range<size_t> &r, ResultTuple J) -> ResultTuple {
        auto Weight = [&](double residual_squared) {
            return kernel_squared / square(kernel + residual_squared);
        };
        for (size_t i = r.begin(); i < r.end(); ++i) {
            if (i < src_planar.size()) { // Point-to-Plane
                if (planar_mask != nullptr && !(*planar_mask)[i]) continue;
                const auto &[J_planar, r_planar] = compute_jacobian_and_residual_planar(i);
                const double residual_squared = r_planar * r_planar;
                double w_planar = Weight(residual_squared);
                if (robust_icp != nullptr && robust_icp->enabled) {
                    w_planar *= RobustLossWeight(*robust_icp, std::abs(r_planar));
                }
                const double weighted_alpha = alpha * w_planar;
                J.JTJ.noalias() += weighted_alpha * J_planar.transpose() * J_planar;
                J.JTr.noalias() += weighted_alpha * J_planar.transpose() * r_planar;
                J.unweighted_error += residual_squared;
                J.weighted_error += weighted_alpha * residual_squared;
                J.weighted_sum += weighted_alpha;
                J.residual_count += 1;
                J.correspondence_count += 1;
            } else { // Point-to-Point
                size_t index = i - src_planar.size();
                if (index < src_non_planar.size()) {
                    if (non_planar_mask != nullptr && !(*non_planar_mask)[index]) continue;
                    const auto &[J_non_planar, r_non_planar] = compute_jacobian_and_residual_non_planar(index);
                    const double residual_squared = r_non_planar.squaredNorm();
                    double w_non_planar = Weight(residual_squared);
                    if (robust_icp != nullptr && robust_icp->enabled) {
                        w_non_planar *= RobustLossWeight(*robust_icp, std::sqrt(residual_squared));
                    }
                    const double weighted_alpha = (1 - alpha) * w_non_planar;
                    J.JTJ.noalias() += weighted_alpha * J_non_planar.transpose() * J_non_planar;
                    J.JTr.noalias() += weighted_alpha * J_non_planar.transpose() * r_non_planar;
                    J.unweighted_error += residual_squared;
                    J.weighted_error += weighted_alpha * residual_squared;
                    J.weighted_sum += weighted_alpha;
                    J.residual_count += 3;
                    J.correspondence_count += 1;
                }
            }
        }
        return J;
    };


    size_t total_size = src_planar.size() + src_non_planar.size();
    const auto result = tbb::parallel_reduce(
        tbb::blocked_range<size_t>(0, total_size),
        ResultTuple(),
        compute,
        [](const ResultTuple &a, const ResultTuple &b) {
            return a + b;
        });

    return {result.JTJ,
            result.JTr,
            result.unweighted_error,
            result.weighted_error,
            result.weighted_sum,
            result.residual_count,
            result.correspondence_count};
}

void VisualizeStatus(size_t planar_count, size_t non_planar_count, double alpha) {
    const int bar_width = 52;
    const std::string planar_color = "\033[1;38;2;0;119;187m";
    const std::string non_planar_color = "\033[1;38;2;238;51;119m";
    const std::string alpha_color = "\033[1;32m";

    printf("\033[2J\033[1;1H"); // Clear terminal
    std::cout << "====================== GenZ-ICP ======================\n";
    std::cout << non_planar_color << "# of non-planar points: " << non_planar_count << ", ";
    std::cout << planar_color << "# of planar points: " << planar_count << "\033[0m\n";

    std::cout << "Unstructured  <-----  ";
    std::cout << alpha_color << "alpha: " << std::fixed << std::setprecision(3) << alpha << "\033[0m";
    std::cout << "  ----->  Structured\n";

    const int alpha_location = static_cast<int>(bar_width * alpha); 
    std::cout << "[";
    for (int i = 0; i < bar_width; ++i) {
        if (i == alpha_location) {
            std::cout << "\033[1;32m█\033[0m"; 
        } else {
            std::cout << "-"; 
        }
    }
    std::cout << "]\n";
    std::cout.flush();
}
}  // namespace

namespace genz_icp {

Registration::Registration(int max_num_iteration, double convergence_criterion)
    : max_num_iterations_(max_num_iteration), 
      convergence_criterion_(convergence_criterion) {}

RegistrationResult Registration::RegisterFrameWithQuality(
    const std::vector<Eigen::Vector3d> &frame,
    const VoxelHashMap &voxel_map,
    const Sophus::SE3d &initial_guess,
    double max_correspondence_distance,
    double kernel,
    const std::optional<RegistrationMotionPriorConfig> &motion_prior,
    const std::optional<RegistrationRobustICPConfig> &robust_icp) {
    RegistrationResult result;
    result.pose = initial_guess;
    result.covariance = HighUncertaintyCovariance();

    if (voxel_map.Empty()) return result;

    const bool motion_prior_active =
        motion_prior && MotionPriorIsActive(*motion_prior);
    const bool robust_icp_active = robust_icp && robust_icp->enabled;
    const Eigen::Matrix6d motion_prior_information =
        motion_prior_active ? MotionPriorInformation(*motion_prior) : Eigen::Matrix6d::Zero();
    if (motion_prior_active && motion_prior->debug) {
        LogMotionPriorConfig(*motion_prior);
        LogMotionPriorDeviation("Motion prior residual before ICP:",
                                MotionPriorError(initial_guess, initial_guess));
    }

    std::vector<Eigen::Vector3d> source = frame;
    TransformPoints(initial_guess, source);

    Eigen::Matrix6d final_information = Eigen::Matrix6d::Zero();
    double final_unweighted_error = 0.0;
    double final_weighted_error = 0.0;
    double final_weighted_sum = 0.0;
    size_t final_residual_count = 0;
    size_t final_correspondence_count = 0;
    bool have_final_information = false;
    double previous_normalized_error = std::numeric_limits<double>::infinity();

    // GenZ-ICP-loop
    Sophus::SE3d T_icp = Sophus::SE3d();
    for (int j = 0; j < max_num_iterations_; ++j) {
        const auto &[src_planar, tgt_planar, normals, src_non_planar, tgt_non_planar, planar_count, non_planar_count] = voxel_map.GetCorrespondences(source, max_correspondence_distance);
        const size_t correspondence_count = planar_count + non_planar_count;
        if (correspondence_count == 0) {
            break;
        }

        RobustICPSelection robust_selection;
        size_t used_planar_count = planar_count;
        size_t used_non_planar_count = non_planar_count;
        size_t used_correspondence_count = correspondence_count;
        const std::vector<uint8_t> *planar_mask = nullptr;
        const std::vector<uint8_t> *non_planar_mask = nullptr;
        if (robust_icp_active) {
            robust_selection =
                SelectRobustCorrespondences(src_planar,
                                            tgt_planar,
                                            normals,
                                            src_non_planar,
                                            tgt_non_planar,
                                            *robust_icp);
            used_planar_count = robust_selection.final_planar_count;
            used_non_planar_count = robust_selection.final_non_planar_count;
            used_correspondence_count = robust_selection.final_correspondences;
            planar_mask = &robust_selection.planar_mask;
            non_planar_mask = &robust_selection.non_planar_mask;

            if (robust_icp->debug) {
                std::cout << "Robust ICP iter=" << j
                          << ": input_correspondences="
                          << robust_selection.input_correspondences
                          << ", rejected_by_max_distance="
                          << robust_selection.rejected_by_max_distance
                          << ", kept_by_trimmed_icp="
                          << robust_selection.kept_after_trimmed_icp
                          << ", mean_residual="
                          << robust_selection.mean_residual
                          << ", median_residual="
                          << robust_selection.median_residual
                          << ", final_used_correspondences="
                          << robust_selection.final_correspondences
                          << (robust_selection.fallback_to_normal_set
                                  ? ", fallback_to_normal_set=true"
                                  : "")
                          << "\n";
            }
        }

        if (used_correspondence_count == 0) {
            break;
        }

        double alpha = static_cast<double>(used_planar_count) /
                       static_cast<double>(used_correspondence_count);
        auto linear_system =
            BuildLinearSystem(src_planar,
                              tgt_planar,
                              normals,
                              src_non_planar,
                              tgt_non_planar,
                              kernel,
                              alpha,
                              robust_icp_active ? &(*robust_icp) : nullptr,
                              planar_mask,
                              non_planar_mask);
        if (motion_prior_active) {
            const Sophus::SE3d current_pose = T_icp * initial_guess;
            AddMotionPrior(linear_system, motion_prior_information, initial_guess, current_pose);
        }
        Eigen::LDLT<Eigen::Matrix6d> ldlt(linear_system.JTJ);
        if (ldlt.info() != Eigen::Success) {
            break;
        }

        const Eigen::Vector6d dx = ldlt.solve(-linear_system.JTr);
        if (!dx.allFinite()) {
            break;
        }

        const Sophus::SE3d estimation = Sophus::SE3d::exp(dx);
        TransformPoints(estimation, source);
        // Update iterations
        T_icp = estimation * T_icp;
        // Termination criteria
        const double normalized_error =
            linear_system.weighted_error /
            std::max<double>(1.0, static_cast<double>(linear_system.residual_count));
        const bool error_converged =
            std::isfinite(previous_normalized_error) &&
            std::abs(previous_normalized_error - normalized_error) <=
                kErrorConvergenceRelativeTolerance * std::max(1.0, previous_normalized_error);
        previous_normalized_error = normalized_error;

        if (dx.norm() < convergence_criterion_ || error_converged || j == max_num_iterations_ - 1) {
            if (terminal_status_enabled_) {
                VisualizeStatus(used_planar_count, used_non_planar_count, alpha);
            }
            result.planar_points = src_planar;
            result.non_planar_points = src_non_planar;
            final_information = linear_system.JTJ;
            final_unweighted_error = linear_system.unweighted_error;
            final_weighted_error = linear_system.weighted_error;
            final_weighted_sum = linear_system.weighted_sum;
            final_residual_count = linear_system.residual_count;
            final_correspondence_count = linear_system.correspondence_count;
            have_final_information = true;
            break;
        }
    }

    result.pose = T_icp * initial_guess;
    if (motion_prior_active && motion_prior->debug) {
        LogMotionPriorDeviation("Motion prior final deviation:",
                                MotionPriorError(initial_guess, result.pose));
    }
    result.covariance =
        have_final_information
            ? CovarianceFromInformation(final_information, final_weighted_error, final_residual_count)
            : HighUncertaintyCovariance();

    if (have_final_information && final_correspondence_count > 0) {
        result.quality.correspondence_count = final_correspondence_count;
        // Quality RMSE is correspondence-level: planar uses r^2 and non-planar
        // uses ||r||^2, then the total is normalized by correspondence count.
        result.quality.rmse =
            std::sqrt(final_unweighted_error / static_cast<double>(final_correspondence_count));
        if (final_weighted_sum > 0.0) {
            result.quality.weighted_rmse =
                std::sqrt(final_weighted_error / final_weighted_sum);
        }
        result.quality.finite =
            IsFinitePose(result.pose) &&
            result.covariance.allFinite() &&
            final_information.allFinite() &&
            std::isfinite(result.quality.rmse) &&
            std::isfinite(result.quality.weighted_rmse);
    }

    return result;
}

std::tuple<Sophus::SE3d,
           std::vector<Eigen::Vector3d>,
           std::vector<Eigen::Vector3d>,
           Eigen::Matrix<double, 6, 6>>
Registration::RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                            const VoxelHashMap &voxel_map,
                            const Sophus::SE3d &initial_guess,
                            double max_correspondence_distance,
                            double kernel) {
    auto result = RegisterFrameWithQuality(
        frame, voxel_map, initial_guess, max_correspondence_distance, kernel);
    return std::make_tuple(result.pose,
                           std::move(result.planar_points),
                           std::move(result.non_planar_points),
                           result.covariance);
}

}  // namespace genz_icp
