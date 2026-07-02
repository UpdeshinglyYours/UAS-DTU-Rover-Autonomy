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
#include <Eigen/Geometry>
#include <sophus/so3.hpp>

// GenZ-ICP
#include "genz_icp/pipeline/GenZICP.hpp"

// ROS 2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cstddef>
#include <deque>
#include <limits>
#include <mutex>
#include <optional>
#include <string>

namespace genz_icp_ros {

enum class ImuPredictionTimeUnit { Seconds, Milliseconds, Microseconds, Nanoseconds };
enum class ImuPredictionScanLocation { Start, Middle, End };

struct ImuPredictionSample {
    double stamp{0.0};
    Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
};

struct ImuPredictionResult {
    Sophus::SO3d rotation{Sophus::SO3d()};
    double dt{0.0};
    size_t sample_count{0};
};

class OdometryServer : public rclcpp::Node {
public:
    /// OdometryServer constructor
    OdometryServer() = delete;
    explicit OdometryServer(const rclcpp::NodeOptions &options);

private:
    /// Register new frame
    void RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
    void ImuPredictionCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

    /// Stream the estimated pose to ROS
    /// Stream the estimated pose to ROS (with covariance matrix included)
    void PublishOdometry(const Sophus::SE3d &pose,
                         const rclcpp::Time &stamp,
                         const std::string &cloud_frame_id,
                         const Eigen::Matrix<double, 6, 6> &covariance);
    void FillTwist(nav_msgs::msg::Odometry &odom_msg,
                   const Sophus::SE3d &pose,
                   const rclcpp::Time &stamp);

    /// Stream the debugging point clouds for visualization (if required)
    void PublishClouds(const rclcpp::Time &stamp,
                       const std::string &cloud_frame_id,
                       const std::vector<Eigen::Vector3d> &planar_points,
                       const std::vector<Eigen::Vector3d> &non_planar_points);
    bool HasDebugCloudSubscribers() const;

    /// Utility function to compute transformation using tf tree
    Sophus::SE3d LookupTransform(const std::string &target_frame,
                                 const std::string &source_frame) const;

    std::optional<double> ComputeScanReferenceTime(const sensor_msgs::msg::PointCloud2 &msg,
                                                   std::string *reason) const;
    std::optional<ImuPredictionResult> BuildImuMotionPrediction(double current_reference_time,
                                                                std::string *reason) const;
    void UpdateImuPredictionGyroBiasCalibration(double stamp,
                                                const Eigen::Vector3d &raw_angular_velocity,
                                                const Eigen::Vector3d &scaled_angular_velocity);
    void InsertImuPredictionSample(const ImuPredictionSample &sample);

private:
    /// Tools for broadcasting TFs.
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
    bool publish_odom_tf_;
    bool publish_debug_clouds_;
    bool terminal_status_enabled_{false};
    size_t max_path_length_{2000};

    /// Twist estimation from consecutive published odometry poses.
    bool publish_twist_{true};
    bool twist_in_child_frame_{true};
    double twist_smoothing_alpha_{1.0};
    double twist_min_dt_{0.001};
    double twist_max_dt_{1.0};
    bool twist_debug_{false};
    double twist_linear_covariance_{0.25};
    double twist_angular_covariance_{0.25};
    bool has_previous_twist_pose_{false};
    bool has_smoothed_twist_{false};
    rclcpp::Time previous_twist_stamp_;
    Eigen::Vector3d previous_twist_position_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond previous_twist_orientation_{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d smoothed_linear_velocity_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d smoothed_angular_velocity_{Eigen::Vector3d::Zero()};

    /// Data subscribers.
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_prediction_sub_;

    /// Data publishers.
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr planar_points_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr non_planar_points_publisher_;

    /// Path publisher
    nav_msgs::msg::Path path_msg_;
    std::deque<geometry_msgs::msg::PoseStamped> path_poses_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_publisher_;

    /// GenZ-ICP
    genz_icp::pipeline::GenZICP odometry_;
    genz_icp::pipeline::GenZConfig config_;

    /// Optional IMU gyro prediction for ICP initialization.
    bool enable_imu_motion_prediction_{false};
    std::string imu_topic_{"/bf_lidar/imu_out"};
    std::string imu_prediction_point_time_field_{"auto"};
    ImuPredictionTimeUnit imu_prediction_point_time_unit_{ImuPredictionTimeUnit::Nanoseconds};
    ImuPredictionScanLocation imu_prediction_cloud_stamp_location_{ImuPredictionScanLocation::End};
    ImuPredictionScanLocation imu_prediction_deskew_reference_{ImuPredictionScanLocation::Middle};
    double imu_angular_velocity_scale_{0.017453292519943295};
    bool enable_imu_prediction_gyro_bias_calibration_{true};
    double imu_prediction_gyro_bias_calibration_seconds_{2.0};
    int imu_prediction_gyro_bias_min_samples_{50};
    double imu_prediction_max_gap_seconds_{0.06};
    double imu_prediction_max_age_seconds_{0.25};
    double imu_prediction_max_rejected_frame_age_seconds_{1.0};
    double imu_prediction_buffer_seconds_{2.0};
    bool imu_prediction_rotation_only_{true};
    bool imu_prediction_debug_{true};
    Eigen::Vector3d imu_prediction_gyro_bias_{Eigen::Vector3d::Zero()};
    bool imu_prediction_gyro_bias_ready_{true};
    bool imu_prediction_gyro_bias_manual_override_{false};
    double imu_prediction_gyro_bias_calibration_start_{std::numeric_limits<double>::quiet_NaN()};
    Eigen::Vector3d imu_prediction_gyro_bias_raw_sum_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d imu_prediction_gyro_bias_scaled_sum_{Eigen::Vector3d::Zero()};
    size_t imu_prediction_gyro_bias_sample_count_{0};
    double latest_imu_prediction_stamp_{-std::numeric_limits<double>::infinity()};
    std::deque<ImuPredictionSample> imu_prediction_buffer_;
    mutable std::mutex imu_prediction_mutex_;
    std::optional<double> previous_accepted_scan_reference_time_;

    /// Global/map coordinate frame.
    std::string odom_frame_{"odom"};
    std::string base_frame_{};
};

}  // namespace genz_icp_ros
