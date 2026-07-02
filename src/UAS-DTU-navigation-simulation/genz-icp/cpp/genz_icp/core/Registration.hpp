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
#include <sophus/se3.hpp>
#include <iomanip>
#include <limits>
#include <optional>
#include <tuple>
#include <vector>

#include "VoxelHashMap.hpp"

namespace genz_icp {

struct RegistrationQuality {
    double rmse = std::numeric_limits<double>::infinity();
    double weighted_rmse = std::numeric_limits<double>::infinity();
    size_t correspondence_count = 0;
    double translation_delta = 0.0;
    double rotation_delta = 0.0;
    bool finite = false;
};

struct RegistrationResult {
    Sophus::SE3d pose;
    std::vector<Eigen::Vector3d> planar_points;
    std::vector<Eigen::Vector3d> non_planar_points;
    Eigen::Matrix<double, 6, 6> covariance =
        Eigen::Matrix<double, 6, 6>::Identity();
    RegistrationQuality quality;
};

struct RegistrationMotionPriorConfig {
    bool enabled = false;
    double translation_sigma = 0.35;
    double z_sigma = 0.12;
    double roll_pitch_sigma_rad = 6.0 * 3.14159265358979323846 / 180.0;
    double yaw_sigma_rad = 30.0 * 3.14159265358979323846 / 180.0;
    double weight = 1.0;
    bool debug = false;
};

struct RegistrationRobustICPConfig {
    bool enabled = false;
    double max_correspondence_distance = 1.0;
    double residual_threshold = 0.35;
    std::string loss_type = "cauchy";
    bool trimmed_icp_enabled = false;
    double trimmed_icp_keep_ratio = 0.80;
    int min_correspondences = 500;
    bool debug = false;
};
    
struct Registration {
    explicit Registration(int max_num_iteration, double convergence_criterion);
    void SetTerminalStatusEnabled(bool enabled) { terminal_status_enabled_ = enabled; }

    RegistrationResult RegisterFrameWithQuality(const std::vector<Eigen::Vector3d> &frame,
                                                const VoxelHashMap &voxel_map,
                                                const Sophus::SE3d &initial_guess,
                                                double max_correspondence_distance,
                                                double kernel,
                                                const std::optional<RegistrationMotionPriorConfig> &motion_prior = std::nullopt,
                                                const std::optional<RegistrationRobustICPConfig> &robust_icp = std::nullopt);

    std::tuple<Sophus::SE3d, std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>, Eigen::Matrix<double, 6, 6>> RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                                                                                                       const VoxelHashMap &voxel_map,
                                                                                                       const Sophus::SE3d &initial_guess,
                                                                                                       double max_correspondence_distance,
                                                                                                       double kernel);

    int max_num_iterations_;
    double convergence_criterion_;
    bool terminal_status_enabled_ = true;
};
}  // namespace genz_icp
