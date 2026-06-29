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
#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <deque>
#include <optional>
#include <sophus/so3.hpp>
#include <string>
#include <tuple>
#include <vector>

#include "genz_icp/core/Deskew.hpp"
#include "genz_icp/core/Threshold.hpp"
#include "genz_icp/core/VoxelHashMap.hpp"
#include "genz_icp/core/Registration.hpp"

namespace genz_icp::pipeline {

struct GenZConfig {
    // map params
    double max_range = 100.0;
    double min_range = 0.5;
    double map_cleanup_radius = 400.0;
    int max_points_per_voxel = 1;

    // voxelize params
    double voxel_size = 0.25;
    int desired_num_voxelized_points = 2000;

    // th parms
    double min_motion_th = 0.1;
    double initial_threshold = 2.0;
    double planarity_threshold = 0.1;

    // Motion compensation
    bool deskew = false;

    // registration params
    int max_num_iterations = 150;
    double convergence_criterion = 0.0001;
    size_t max_pose_history = 2000;

    // Registration quality gate
    bool enable_registration_quality_gate = true;
    int min_registration_correspondences = 300;
    double registration_rmse_reject_ratio = 2.5;
    double registration_rmse_ema_alpha = 0.05;
    double max_registration_translation_per_frame = 1.0;
    double max_registration_rotation_per_frame_deg = 45.0;
    int max_consecutive_registration_rejections = 5;
    double absolute_registration_rmse_limit = 1.0;

    // Optional pre-ICP yaw search initializer
    bool enable_yaw_search_initializer = false;
    std::vector<double> yaw_search_degrees = {-12.0, -8.0, -4.0, 0.0, 4.0, 8.0, 12.0};
    double yaw_search_score_max_correspondence_distance = 1.5;
    int yaw_search_min_correspondences = 500;
    bool yaw_search_use_weighted_rmse = true;
    std::string yaw_search_vertical_axis = "z";
    bool yaw_search_debug = true;

    // Optional soft motion prior inside ICP registration
    bool enable_motion_prior = false;
    double motion_prior_translation_sigma = 0.35;
    double motion_prior_z_sigma = 0.12;
    double motion_prior_roll_pitch_sigma_deg = 6.0;
    double motion_prior_yaw_sigma_deg = 30.0;
    double motion_prior_weight = 1.0;
    bool motion_prior_apply_during_recovery = true;
    bool motion_prior_debug = true;

    // Optional gate for inserting accepted scans into the local map
    bool enable_map_update_quality_gate = false;
    double map_update_max_weighted_rmse_ratio = 1.25;
    double map_update_max_rmse = 0.35;
    double map_update_max_translation_delta = 0.75;
    double map_update_max_rotation_delta_deg = 10.0;
    int map_update_min_correspondences = 2500;
    bool map_update_debug = true;
};

class GenZICP {
public:
    using Vector3dVector = std::vector<Eigen::Vector3d>;
    using Vector3dVectorTuple = std::tuple<Vector3dVector, Vector3dVector>;
    using RegistrationTuple = std::tuple<Vector3dVector, Vector3dVector, Eigen::Matrix<double, 6, 6>>;

public:
    explicit GenZICP(const GenZConfig &config)
        : config_(config),
          adaptive_voxel_size_(config.voxel_size),
          registration_(config.max_num_iterations, config.convergence_criterion),
          local_map_(config.voxel_size, config.max_range, config.map_cleanup_radius, config.planarity_threshold, config.max_points_per_voxel),
          adaptive_threshold_(config.initial_threshold, config.min_motion_th, config.max_range) {}

    GenZICP() : GenZICP(GenZConfig{}) {}

public:
    RegistrationTuple RegisterFrame(const std::vector<Eigen::Vector3d> &frame);
    RegistrationTuple RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                                    const std::optional<Sophus::SO3d> &rotation_prediction);
    RegistrationTuple RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                                    const std::vector<double> &timestamps);
    RegistrationTuple RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                                    const std::vector<double> &timestamps,
                                    const std::optional<Sophus::SO3d> &rotation_prediction);
    void SetTerminalStatusEnabled(bool enabled) {
        terminal_status_enabled_ = enabled;
        registration_.SetTerminalStatusEnabled(enabled);
    }
    Vector3dVectorTuple Voxelize(const std::vector<Eigen::Vector3d> &frame, double voxel_size) const;
    double GetAdaptiveThreshold();
    Sophus::SE3d GetPredictionModel() const;
    bool HasMoved();

public:
    // Extra C++ API to facilitate ROS debugging
    std::vector<Eigen::Vector3d> LocalMap() const { return local_map_.Pointcloud(); };
    const std::deque<Sophus::SE3d> &poses() const { return poses_; };
    bool LastFrameAccepted() const { return last_frame_accepted_; }
    size_t ConsecutiveRegistrationRejections() const { return consecutive_registration_rejections_; }

private:
    void PushPose(const Sophus::SE3d &pose);
    bool IsRegistrationAcceptable(const genz_icp::RegistrationQuality &quality,
                                  std::string &reason) const;
    bool IsMapUpdateAcceptable(const genz_icp::RegistrationQuality &quality,
                               std::string &reason) const;
    void UpdateRegistrationQualityEma(const genz_icp::RegistrationQuality &quality);
    void LogRegistrationDecision(bool accepted,
                                 const genz_icp::RegistrationQuality &quality,
                                 const std::string &reason,
                                 bool force = false) const;
    void LogMapUpdateDecision(bool accepted,
                              const genz_icp::RegistrationQuality &quality,
                              const std::string &reason,
                              bool force = false) const;

    // GenZ-ICP pipeline modules
    std::deque<Sophus::SE3d> poses_;
    Sophus::SE3d initial_pose_;
    bool has_initial_pose_ = false;
    Eigen::Matrix<double, 6, 6> last_covariance_ = Eigen::Matrix<double, 6, 6>::Identity() * 1e6;
    Eigen::Matrix<double, 4, 1> last_registered_signature_ = Eigen::Matrix<double, 4, 1>::Zero();
    bool has_last_registered_signature_ = false;
    size_t stationary_frame_count_ = 0;
    size_t skipped_stationary_frames_ = 0;
    bool terminal_status_enabled_ = true;
    bool registration_rmse_ema_initialized_ = false;
    double registration_rmse_ema_ = 0.0;
    bool registration_weighted_rmse_ema_initialized_ = false;
    double registration_weighted_rmse_ema_ = 0.0;
    size_t accepted_registration_count_ = 0;
    size_t consecutive_registration_rejections_ = 0;
    bool registration_recovery_mode_ = false;
    bool last_frame_accepted_ = false;
    GenZConfig config_;
    double adaptive_voxel_size_;
    Registration registration_;
    VoxelHashMap local_map_;
    AdaptiveThreshold adaptive_threshold_;
};

}  // namespace genz_icp::pipeline
