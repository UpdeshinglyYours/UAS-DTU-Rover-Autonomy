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
#include <Eigen/Core>
#include <algorithm>
#include <exception>
#include <memory>
#include <sophus/se3.hpp>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>

// GenZ-ICP-ROS
#include "OdometryServer.hpp"
#include "Utils.hpp"

// GenZ-ICP
#include "genz_icp/pipeline/GenZICP.hpp"

// ROS 2 headers
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rcpputils/filesystem_helper.hpp"

namespace genz_icp_ros {

using utils::EigenToPointCloud2;
using utils::GetTimestamps;
using utils::PointCloud2ToEigen;

OdometryServer::OdometryServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("odometry_node", options) {
    // clang-format off
    base_frame_ = declare_parameter<std::string>("base_frame", base_frame_);
    odom_frame_ = declare_parameter<std::string>("odom_frame", odom_frame_);
    publish_odom_tf_ = declare_parameter<bool>("publish_odom_tf", publish_odom_tf_);
    publish_debug_clouds_ = declare_parameter<bool>("visualize", publish_debug_clouds_);
    terminal_status_enabled_ = declare_parameter<bool>("terminal_status", terminal_status_enabled_);
    declare_parameter<int>("max_path_length", static_cast<int>(max_path_length_));
    declare_parameter<double>("max_range", config_.max_range);
    declare_parameter<double>("min_range", config_.min_range);
    declare_parameter<bool>("deskew", config_.deskew);
    declare_parameter<double>("voxel_size", config_.max_range / 100.0);
    declare_parameter<double>("map_cleanup_radius", config_.map_cleanup_radius);
    declare_parameter<double>("planarity_threshold", config_.planarity_threshold);
    declare_parameter<int>("max_points_per_voxel", config_.max_points_per_voxel);
    declare_parameter<int>("desired_num_voxelized_points", config_.desired_num_voxelized_points);
    declare_parameter<int>("max_num_iterations", config_.max_num_iterations);
    declare_parameter<double>("convergence_criterion", config_.convergence_criterion);
    declare_parameter<int>("max_pose_history", static_cast<int>(config_.max_pose_history));
    declare_parameter<double>("initial_threshold", config_.initial_threshold);
    declare_parameter<double>("min_motion_th", config_.min_motion_th);
    declare_parameter<std::string>("config_file", "");

    const bool launch_deskew_requested = get_parameter("deskew").as_bool();

    // Load the configuration file
    std::string config_file = get_parameter("config_file").as_string();
    if (!config_file.empty()) {
        rcpputils::fs::path path(config_file);
        if (!path.is_absolute()) {
            path = rcpputils::fs::path(ament_index_cpp::get_package_share_directory("genz_icp")) / "config" / path;
        }
        YAML::Node yaml = YAML::LoadFile(path.string());

        std::vector<rclcpp::Parameter> overrides;
        for (const auto &param : yaml) {
            const auto &name = param.first.as<std::string>();
            const auto &value = param.second;

            if (value.IsScalar()) {
                // Get the declared type of the parameter
                rcl_interfaces::msg::ParameterDescriptor descriptor;
                descriptor = this->describe_parameter(name);

                using ParamType = rcl_interfaces::msg::ParameterType;
                switch (descriptor.type) {
                    case ParamType::PARAMETER_DOUBLE:
                        overrides.emplace_back(name, value.as<double>());
                        break;
                    case ParamType::PARAMETER_INTEGER:
                        overrides.emplace_back(name, value.as<int>());
                        break;
                    case ParamType::PARAMETER_BOOL:
                        overrides.emplace_back(name, value.as<bool>());
                        break;
                    case ParamType::PARAMETER_STRING:
                        overrides.emplace_back(name, value.as<std::string>());
                        break;
                    default:
                        break;
                }
            }
        }
        set_parameters(overrides);
        if (launch_deskew_requested) {
            set_parameter(rclcpp::Parameter("deskew", true));
        }
    }

    config_.max_range = get_parameter("max_range").as_double();
    config_.min_range = get_parameter("min_range").as_double();
    config_.deskew = get_parameter("deskew").as_bool();
    config_.voxel_size = get_parameter("voxel_size").as_double();
    config_.map_cleanup_radius = get_parameter("map_cleanup_radius").as_double();
    config_.planarity_threshold = get_parameter("planarity_threshold").as_double();
    config_.max_points_per_voxel = get_parameter("max_points_per_voxel").as_int();
    config_.desired_num_voxelized_points = get_parameter("desired_num_voxelized_points").as_int();
    config_.max_num_iterations = get_parameter("max_num_iterations").as_int();
    config_.convergence_criterion = get_parameter("convergence_criterion").as_double();
    config_.max_pose_history =
        static_cast<size_t>(std::max<int64_t>(2, get_parameter("max_pose_history").as_int()));
    config_.initial_threshold = get_parameter("initial_threshold").as_double();
    config_.min_motion_th = get_parameter("min_motion_th").as_double();
    terminal_status_enabled_ = get_parameter("terminal_status").as_bool();
    max_path_length_ =
        static_cast<size_t>(std::max<int64_t>(0, get_parameter("max_path_length").as_int()));
    if (max_path_length_ > 0) {
        path_msg_.poses.reserve(max_path_length_);
    }
    if (config_.max_range < config_.min_range) {
        RCLCPP_WARN(get_logger(), "[WARNING] max_range is smaller than min_range, settng min_range to 0.0");
        config_.min_range = 0.0;
    }
    // clang-format on

    // Construct the main GenZ-ICP odometry node
    odometry_ = genz_icp::pipeline::GenZICP(config_);
    odometry_.SetTerminalStatusEnabled(terminal_status_enabled_);
    RCLCPP_INFO(this->get_logger(), "LiDAR deskew is %s", config_.deskew ? "enabled" : "disabled");

    // Initialize subscribers
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud_topic", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        std::bind(&OdometryServer::RegisterFrame, this, std::placeholders::_1));

    // Initialize publishers
    rclcpp::QoS qos((rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile()));
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/genz/odometry", qos);
    traj_publisher_ = create_publisher<nav_msgs::msg::Path>("/genz/trajectory", qos);
    path_msg_.header.frame_id = odom_frame_;
    if (publish_debug_clouds_) {
        map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/genz/local_map", qos);
        planar_points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/genz/planar_points", qos);
        non_planar_points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/genz/non_planar_points", qos);
    }

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf2_buffer_->setUsingDedicatedThread(true);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

    RCLCPP_INFO(this->get_logger(), "GenZ-ICP ROS 2 odometry node initialized");
}

Sophus::SE3d OdometryServer::LookupTransform(const std::string &target_frame,
                                             const std::string &source_frame) const {
    std::string err_msg;
    if (tf2_buffer_->_frameExists(source_frame) &&  //
        tf2_buffer_->_frameExists(target_frame) &&  //
        tf2_buffer_->canTransform(target_frame, source_frame, tf2::TimePointZero, &err_msg)) {
        try {
            auto tf = tf2_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
            return tf2::transformToSophus(tf);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        }
    }
    RCLCPP_WARN(this->get_logger(), "Failed to find tf. Reason=%s", err_msg.c_str());
    return {};
}

void OdometryServer::RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    const auto cloud_frame_id = msg->header.frame_id;
    const auto points = PointCloud2ToEigen(msg);
    std::vector<double> timestamps;
    if (config_.deskew) {
        try {
            timestamps = GetTimestamps(msg);
            if (timestamps.size() != points.size()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Deskew enabled but timestamp count (%zu) does not match point count (%zu); skipping deskew for this scan",
                                     timestamps.size(), points.size());
                timestamps.clear();
            }
        } catch (const std::exception &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Deskew enabled but timestamp extraction failed: %s; skipping deskew for this scan",
                                 ex.what());
        }
    }
    const auto egocentric_estimation = (base_frame_.empty() || base_frame_ == cloud_frame_id);

    // Register frame, main entry point to GenZ-ICP pipeline
    const auto &[planar_points, non_planar_points, covariance] = odometry_.RegisterFrame(points, timestamps);

    // Compute the pose using GenZ, ego-centric to the LiDAR
    const Sophus::SE3d genz_pose = odometry_.poses().back();

    // If necessary, transform the ego-centric pose to the specified base_link/base_footprint frame
    const auto pose = [&]() -> Sophus::SE3d {
        if (egocentric_estimation) return genz_pose;
        const Sophus::SE3d cloud2base = LookupTransform(base_frame_, cloud_frame_id);
        return cloud2base * genz_pose * cloud2base.inverse();
    }();

    // Spit the current estimated pose to ROS msgs
    PublishOdometry(pose, msg->header.stamp, cloud_frame_id, covariance);
    // Publishing this clouds is a bit costly, so do it only if we are debugging
    if (publish_debug_clouds_ && HasDebugCloudSubscribers()) {
        PublishClouds(msg->header.stamp, cloud_frame_id, planar_points, non_planar_points);
    }
}

void OdometryServer::PublishOdometry(const Sophus::SE3d &pose,
                                     const rclcpp::Time &stamp,
                                     const std::string &cloud_frame_id,
                                     const Eigen::Matrix<double, 6, 6> &covariance) {

    // Broadcast the tf ---
    if (publish_odom_tf_) {
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = stamp;
        transform_msg.header.frame_id = odom_frame_;
        transform_msg.child_frame_id = base_frame_.empty() ? cloud_frame_id : base_frame_;
        transform_msg.transform = tf2::sophusToTransform(pose);
        tf_broadcaster_->sendTransform(transform_msg);
    }

    const auto trajectory_subscribers =
        traj_publisher_->get_subscription_count() +
        traj_publisher_->get_intra_process_subscription_count();
    if (max_path_length_ > 0 && trajectory_subscribers > 0) {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = odom_frame_;
        pose_msg.pose = tf2::sophusToPose(pose);
        path_poses_.push_back(pose_msg);
        while (path_poses_.size() > max_path_length_) {
            path_poses_.pop_front();
        }

        path_msg_.poses.assign(path_poses_.begin(), path_poses_.end());
        traj_publisher_->publish(path_msg_);
    } else if (!path_msg_.poses.empty() || !path_poses_.empty()) {
        std::deque<geometry_msgs::msg::PoseStamped>().swap(path_poses_);
        std::vector<geometry_msgs::msg::PoseStamped>().swap(path_msg_.poses);
    }

    // publish odometry msg
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_;

    odom_msg.child_frame_id = base_frame_.empty() ? cloud_frame_id : base_frame_;

    odom_msg.pose.pose = tf2::sophusToPose(pose);

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            odom_msg.pose.covariance[i * 6 + j] = covariance(i, j);
        }
    }

    odom_publisher_->publish(std::move(odom_msg));
}

void OdometryServer::PublishClouds(const rclcpp::Time &stamp,
                                   const std::string &cloud_frame_id,
                                   const std::vector<Eigen::Vector3d> &planar_points,
                                   const std::vector<Eigen::Vector3d> &non_planar_points) {
    const auto map_subscribers =
        map_publisher_->get_subscription_count() +
        map_publisher_->get_intra_process_subscription_count();
    const auto planar_subscribers =
        planar_points_publisher_->get_subscription_count() +
        planar_points_publisher_->get_intra_process_subscription_count();
    const auto non_planar_subscribers =
        non_planar_points_publisher_->get_subscription_count() +
        non_planar_points_publisher_->get_intra_process_subscription_count();

    std_msgs::msg::Header odom_header;
    odom_header.stamp = stamp;
    odom_header.frame_id = odom_frame_;

    if (!publish_odom_tf_) {
        // debugging happens in an egocentric world
        std_msgs::msg::Header cloud_header;
        cloud_header.stamp = stamp;
        cloud_header.frame_id = cloud_frame_id;

        if (map_subscribers > 0) {
            const auto genz_map = odometry_.LocalMap();
            map_publisher_->publish(std::move(EigenToPointCloud2(genz_map, odom_header)));
        }
        if (planar_subscribers > 0) {
            planar_points_publisher_->publish(std::move(EigenToPointCloud2(planar_points, cloud_header)));
        }
        if (non_planar_subscribers > 0) {
            non_planar_points_publisher_->publish(std::move(EigenToPointCloud2(non_planar_points, cloud_header)));
        }

        return;
    }

    // If transmitting to tf tree we know where the clouds are exactly
    if (planar_subscribers > 0) {
        planar_points_publisher_->publish(std::move(EigenToPointCloud2(planar_points, odom_header)));
    }
    if (non_planar_subscribers > 0) {
        non_planar_points_publisher_->publish(std::move(EigenToPointCloud2(non_planar_points, odom_header)));
    }

    if (map_subscribers > 0) {
        const auto genz_map = odometry_.LocalMap();
        if (!base_frame_.empty()) {
            const Sophus::SE3d cloud2base = LookupTransform(base_frame_, cloud_frame_id);
            map_publisher_->publish(std::move(EigenToPointCloud2(genz_map, cloud2base, odom_header)));
        } else {
            map_publisher_->publish(std::move(EigenToPointCloud2(genz_map, odom_header)));
        }
    }
}

bool OdometryServer::HasDebugCloudSubscribers() const {
    const auto has_subscribers = [](const auto &publisher) {
        return publisher &&
               (publisher->get_subscription_count() +
               publisher->get_intra_process_subscription_count()) > 0;
    };

    return has_subscribers(map_publisher_) ||
           has_subscribers(planar_points_publisher_) ||
           has_subscribers(non_planar_points_publisher_);
}
}  // namespace genz_icp_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(genz_icp_ros::OdometryServer)
