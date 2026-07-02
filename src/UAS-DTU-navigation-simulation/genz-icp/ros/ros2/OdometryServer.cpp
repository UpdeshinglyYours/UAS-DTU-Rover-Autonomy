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
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <exception>
#include <functional>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
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
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rcpputils/filesystem_helper.hpp"

namespace genz_icp_ros {

using utils::EigenToPointCloud2;
using utils::GetTimestamps;
using utils::PointCloud2ToEigen;

namespace {
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;

constexpr double kAbsoluteUnixTimeThreshold = 1.0e8;
constexpr double kTimeEpsilon = 1.0e-9;
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

std::string ToLower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

std::string Trim(const std::string &value) {
    const auto begin = value.find_first_not_of(" \t\n\r");
    if (begin == std::string::npos) return "";
    const auto end = value.find_last_not_of(" \t\n\r");
    return value.substr(begin, end - begin + 1);
}

std::vector<std::string> SplitCandidates(const std::string &value) {
    std::vector<std::string> candidates;
    std::stringstream stream(value);
    std::string token;
    while (std::getline(stream, token, ',')) {
        token = Trim(token);
        if (!token.empty()) candidates.push_back(token);
    }
    return candidates;
}

std::optional<ImuPredictionTimeUnit> ParseImuPredictionTimeUnit(const std::string &unit) {
    const auto normalized = ToLower(Trim(unit));
    if (normalized == "seconds" || normalized == "second" || normalized == "sec" ||
        normalized == "s") {
        return ImuPredictionTimeUnit::Seconds;
    }
    if (normalized == "milliseconds" || normalized == "millisecond" || normalized == "msec" ||
        normalized == "ms") {
        return ImuPredictionTimeUnit::Milliseconds;
    }
    if (normalized == "microseconds" || normalized == "microsecond" || normalized == "usec" ||
        normalized == "us") {
        return ImuPredictionTimeUnit::Microseconds;
    }
    if (normalized == "nanoseconds" || normalized == "nanosecond" || normalized == "nsec" ||
        normalized == "ns") {
        return ImuPredictionTimeUnit::Nanoseconds;
    }
    return std::nullopt;
}

std::optional<ImuPredictionScanLocation> ParseImuPredictionScanLocation(
    const std::string &location) {
    const auto normalized = ToLower(Trim(location));
    if (normalized == "start" || normalized == "begin" || normalized == "beginning") {
        return ImuPredictionScanLocation::Start;
    }
    if (normalized == "middle" || normalized == "mid" || normalized == "midpoint" ||
        normalized == "center" || normalized == "centre") {
        return ImuPredictionScanLocation::Middle;
    }
    if (normalized == "end" || normalized == "finish" || normalized == "last") {
        return ImuPredictionScanLocation::End;
    }
    return std::nullopt;
}

double UnitScale(const ImuPredictionTimeUnit unit) {
    switch (unit) {
        case ImuPredictionTimeUnit::Seconds:
            return 1.0;
        case ImuPredictionTimeUnit::Milliseconds:
            return 1.0e-3;
        case ImuPredictionTimeUnit::Microseconds:
            return 1.0e-6;
        case ImuPredictionTimeUnit::Nanoseconds:
            return 1.0e-9;
    }
    return 1.0;
}

double LocationOffset(const ImuPredictionScanLocation location, const double duration) {
    switch (location) {
        case ImuPredictionScanLocation::Start:
            return 0.0;
        case ImuPredictionScanLocation::Middle:
            return 0.5 * duration;
        case ImuPredictionScanLocation::End:
            return duration;
    }
    return 0.0;
}

std::string FieldTypeName(const std::uint8_t datatype) {
    switch (datatype) {
        case PointField::INT8:
            return "INT8";
        case PointField::UINT8:
            return "UINT8";
        case PointField::INT16:
            return "INT16";
        case PointField::UINT16:
            return "UINT16";
        case PointField::INT32:
            return "INT32";
        case PointField::UINT32:
            return "UINT32";
        case PointField::FLOAT32:
            return "FLOAT32";
        case PointField::FLOAT64:
            return "FLOAT64";
        default:
            return "UNKNOWN";
    }
}

std::string AvailableFieldsString(const PointCloud2 &cloud) {
    std::ostringstream stream;
    for (size_t i = 0; i < cloud.fields.size(); ++i) {
        const auto &field = cloud.fields[i];
        if (i > 0) stream << ", ";
        stream << field.name << ":" << FieldTypeName(field.datatype) << "@" << field.offset;
        if (field.count != 1) stream << "[" << field.count << "]";
    }
    return stream.str();
}

const PointField *FindField(const PointCloud2 &cloud, const std::string &name) {
    const auto it = std::find_if(cloud.fields.begin(), cloud.fields.end(),
                                 [&](const PointField &field) {
                                     return field.name == name && field.count > 0;
                                 });
    return it == cloud.fields.end() ? nullptr : &(*it);
}

template <typename T>
std::vector<double> ExtractTypedFieldSeconds(const PointCloud2 &cloud,
                                             const std::string &field_name,
                                             const double unit_scale) {
    const size_t n_points = static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
    std::vector<double> values;
    values.reserve(n_points);

    sensor_msgs::PointCloud2ConstIterator<T> it(cloud, field_name);
    for (size_t i = 0; i < n_points; ++i, ++it) {
        values.push_back(static_cast<double>(*it) * unit_scale);
    }
    return values;
}

std::vector<double> ExtractFieldSeconds(const PointCloud2 &cloud,
                                        const PointField &field,
                                        const ImuPredictionTimeUnit unit) {
    const double unit_scale = UnitScale(unit);
    switch (field.datatype) {
        case PointField::INT8:
            return ExtractTypedFieldSeconds<std::int8_t>(cloud, field.name, unit_scale);
        case PointField::UINT8:
            return ExtractTypedFieldSeconds<std::uint8_t>(cloud, field.name, unit_scale);
        case PointField::INT16:
            return ExtractTypedFieldSeconds<std::int16_t>(cloud, field.name, unit_scale);
        case PointField::UINT16:
            return ExtractTypedFieldSeconds<std::uint16_t>(cloud, field.name, unit_scale);
        case PointField::INT32:
            return ExtractTypedFieldSeconds<std::int32_t>(cloud, field.name, unit_scale);
        case PointField::UINT32:
            return ExtractTypedFieldSeconds<std::uint32_t>(cloud, field.name, unit_scale);
        case PointField::FLOAT32:
            return ExtractTypedFieldSeconds<float>(cloud, field.name, unit_scale);
        case PointField::FLOAT64:
            return ExtractTypedFieldSeconds<double>(cloud, field.name, unit_scale);
        default:
            throw std::runtime_error("unsupported point time field datatype: " +
                                     FieldTypeName(field.datatype));
    }
}

bool AllFinite(const std::vector<double> &values) {
    return std::all_of(values.begin(), values.end(), [](const double value) {
        return std::isfinite(value);
    });
}

std::string FormatVector(const Eigen::Vector3d &value) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6)
           << "[" << value.x() << ", " << value.y() << ", " << value.z() << "]";
    return stream.str();
}
}  // namespace

OdometryServer::OdometryServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("odometry_node", options) {
    // clang-format off
    base_frame_ = declare_parameter<std::string>("base_frame", base_frame_);
    odom_frame_ = declare_parameter<std::string>("odom_frame", odom_frame_);
    publish_odom_tf_ = declare_parameter<bool>("publish_odom_tf", publish_odom_tf_);
    publish_debug_clouds_ = declare_parameter<bool>("visualize", publish_debug_clouds_);
    terminal_status_enabled_ = declare_parameter<bool>("terminal_status", terminal_status_enabled_);
    publish_twist_ = declare_parameter<bool>("publish_twist", publish_twist_);
    twist_in_child_frame_ = declare_parameter<bool>("twist_in_child_frame", twist_in_child_frame_);
    twist_smoothing_alpha_ = declare_parameter<double>("twist_smoothing_alpha", twist_smoothing_alpha_);
    twist_min_dt_ = declare_parameter<double>("twist_min_dt", twist_min_dt_);
    twist_max_dt_ = declare_parameter<double>("twist_max_dt", twist_max_dt_);
    twist_debug_ = declare_parameter<bool>("twist_debug", twist_debug_);
    twist_linear_covariance_ = declare_parameter<double>("twist_linear_covariance", twist_linear_covariance_);
    twist_angular_covariance_ = declare_parameter<double>("twist_angular_covariance", twist_angular_covariance_);
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
    declare_parameter<bool>("enable_registration_quality_gate", config_.enable_registration_quality_gate);
    declare_parameter<int>("min_registration_correspondences", config_.min_registration_correspondences);
    declare_parameter<double>("registration_rmse_reject_ratio", config_.registration_rmse_reject_ratio);
    declare_parameter<double>("registration_rmse_ema_alpha", config_.registration_rmse_ema_alpha);
    declare_parameter<double>("max_registration_translation_per_frame", config_.max_registration_translation_per_frame);
    declare_parameter<double>("max_registration_rotation_per_frame_deg", config_.max_registration_rotation_per_frame_deg);
    declare_parameter<int>("max_consecutive_registration_rejections", config_.max_consecutive_registration_rejections);
    declare_parameter<double>("absolute_registration_rmse_limit", config_.absolute_registration_rmse_limit);
    enable_imu_motion_prediction_ =
        declare_parameter<bool>("enable_imu_motion_prediction", enable_imu_motion_prediction_);
    imu_topic_ = declare_parameter<std::string>("imu_topic", imu_topic_);
    imu_prediction_point_time_field_ =
        declare_parameter<std::string>("imu_prediction_point_time_field", imu_prediction_point_time_field_);
    declare_parameter<std::string>("imu_prediction_point_time_unit", "nanoseconds");
    declare_parameter<std::string>("imu_prediction_cloud_stamp_location", "end");
    declare_parameter<std::string>("imu_prediction_deskew_reference", "middle");
    imu_angular_velocity_scale_ =
        declare_parameter<double>("imu_angular_velocity_scale", imu_angular_velocity_scale_);
    enable_imu_prediction_gyro_bias_calibration_ =
        declare_parameter<bool>("enable_imu_prediction_gyro_bias_calibration",
                                enable_imu_prediction_gyro_bias_calibration_);
    imu_prediction_gyro_bias_calibration_seconds_ =
        declare_parameter<double>("imu_prediction_gyro_bias_calibration_seconds",
                                  imu_prediction_gyro_bias_calibration_seconds_);
    imu_prediction_gyro_bias_min_samples_ =
        declare_parameter<int>("imu_prediction_gyro_bias_min_samples",
                               imu_prediction_gyro_bias_min_samples_);
    declare_parameter<std::vector<double>>("imu_prediction_gyro_bias", std::vector<double>{});
    imu_prediction_max_gap_seconds_ =
        declare_parameter<double>("imu_prediction_max_gap_seconds", imu_prediction_max_gap_seconds_);
    imu_prediction_max_age_seconds_ =
        declare_parameter<double>("imu_prediction_max_age_seconds", imu_prediction_max_age_seconds_);
    imu_prediction_max_rejected_frame_age_seconds_ =
        declare_parameter<double>("imu_prediction_max_rejected_frame_age_seconds",
                                  imu_prediction_max_rejected_frame_age_seconds_);
    imu_prediction_rotation_only_ =
        declare_parameter<bool>("imu_prediction_rotation_only", imu_prediction_rotation_only_);
    imu_prediction_debug_ =
        declare_parameter<bool>("imu_prediction_debug", imu_prediction_debug_);
    declare_parameter<bool>("enable_yaw_search_initializer", config_.enable_yaw_search_initializer);
    declare_parameter<std::vector<double>>("yaw_search_degrees", config_.yaw_search_degrees);
    declare_parameter<double>("yaw_search_score_max_correspondence_distance",
                              config_.yaw_search_score_max_correspondence_distance);
    declare_parameter<int>("yaw_search_min_correspondences", config_.yaw_search_min_correspondences);
    declare_parameter<bool>("yaw_search_use_weighted_rmse", config_.yaw_search_use_weighted_rmse);
    declare_parameter<std::string>("yaw_search_vertical_axis", config_.yaw_search_vertical_axis);
    declare_parameter<bool>("yaw_search_debug", config_.yaw_search_debug);
    declare_parameter<bool>("enable_motion_prior", config_.enable_motion_prior);
    declare_parameter<double>("motion_prior_translation_sigma", config_.motion_prior_translation_sigma);
    declare_parameter<double>("motion_prior_z_sigma", config_.motion_prior_z_sigma);
    declare_parameter<double>("motion_prior_roll_pitch_sigma_deg", config_.motion_prior_roll_pitch_sigma_deg);
    declare_parameter<double>("motion_prior_yaw_sigma_deg", config_.motion_prior_yaw_sigma_deg);
    declare_parameter<double>("motion_prior_weight", config_.motion_prior_weight);
    declare_parameter<bool>("motion_prior_apply_during_recovery", config_.motion_prior_apply_during_recovery);
    declare_parameter<bool>("motion_prior_debug", config_.motion_prior_debug);
    declare_parameter<bool>("enable_map_update_quality_gate", config_.enable_map_update_quality_gate);
    declare_parameter<double>("map_update_max_weighted_rmse_ratio", config_.map_update_max_weighted_rmse_ratio);
    declare_parameter<double>("map_update_max_rmse", config_.map_update_max_rmse);
    declare_parameter<double>("map_update_max_translation_delta", config_.map_update_max_translation_delta);
    declare_parameter<double>("map_update_max_rotation_delta_deg", config_.map_update_max_rotation_delta_deg);
    declare_parameter<int>("map_update_min_correspondences", config_.map_update_min_correspondences);
    declare_parameter<bool>("map_update_debug", config_.map_update_debug);
    declare_parameter<bool>("enable_robust_icp_outlier_handling", config_.enable_robust_icp_outlier_handling);
    declare_parameter<double>("robust_max_correspondence_distance", config_.robust_max_correspondence_distance);
    declare_parameter<double>("robust_residual_threshold", config_.robust_residual_threshold);
    declare_parameter<std::string>("robust_loss_type", config_.robust_loss_type);
    declare_parameter<bool>("trimmed_icp_enabled", config_.trimmed_icp_enabled);
    declare_parameter<double>("trimmed_icp_keep_ratio", config_.trimmed_icp_keep_ratio);
    declare_parameter<int>("robust_min_correspondences", config_.robust_min_correspondences);
    declare_parameter<bool>("robust_icp_debug", config_.robust_icp_debug);
    declare_parameter<bool>("enable_tentative_map_gating", config_.enable_tentative_map_gating);
    declare_parameter<double>("tentative_voxel_size", config_.tentative_voxel_size);
    declare_parameter<int>("tentative_required_observations", config_.tentative_required_observations);
    declare_parameter<int>("tentative_max_age_frames", config_.tentative_max_age_frames);
    declare_parameter<double>("tentative_stable_support_radius", config_.tentative_stable_support_radius);
    declare_parameter<bool>("use_tentative_points_for_icp", config_.use_tentative_points_for_icp);
    declare_parameter<bool>("insert_new_points_as_tentative", config_.insert_new_points_as_tentative);
    declare_parameter<bool>("promote_tentative_only_when_motion_is_calm", config_.promote_tentative_only_when_motion_is_calm);
    declare_parameter<double>("dynamic_enable_max_delta_yaw_deg", config_.dynamic_enable_max_delta_yaw_deg);
    declare_parameter<double>("dynamic_relax_max_delta_yaw_deg", config_.dynamic_relax_max_delta_yaw_deg);
    declare_parameter<bool>("map_update_allow_new_points_when_map_is_small", config_.map_update_allow_new_points_when_map_is_small);
    declare_parameter<int>("map_update_min_stable_map_points", config_.map_update_min_stable_map_points);
    declare_parameter<bool>("tentative_map_debug", config_.tentative_map_debug);
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
            } else if (value.IsSequence()) {
                rcl_interfaces::msg::ParameterDescriptor descriptor;
                descriptor = this->describe_parameter(name);

                using ParamType = rcl_interfaces::msg::ParameterType;
                switch (descriptor.type) {
                    case ParamType::PARAMETER_DOUBLE_ARRAY: {
                        std::vector<double> values;
                        values.reserve(value.size());
                        for (const auto &entry : value) {
                            values.push_back(entry.as<double>());
                        }
                        overrides.emplace_back(name, values);
                        break;
                    }
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
    config_.enable_registration_quality_gate =
        get_parameter("enable_registration_quality_gate").as_bool();
    config_.min_registration_correspondences =
        static_cast<int>(std::max<int64_t>(0, get_parameter("min_registration_correspondences").as_int()));
    config_.registration_rmse_reject_ratio =
        get_parameter("registration_rmse_reject_ratio").as_double();
    config_.registration_rmse_ema_alpha =
        get_parameter("registration_rmse_ema_alpha").as_double();
    config_.max_registration_translation_per_frame =
        get_parameter("max_registration_translation_per_frame").as_double();
    config_.max_registration_rotation_per_frame_deg =
        get_parameter("max_registration_rotation_per_frame_deg").as_double();
    config_.max_consecutive_registration_rejections =
        static_cast<int>(std::max<int64_t>(0, get_parameter("max_consecutive_registration_rejections").as_int()));
    config_.absolute_registration_rmse_limit =
        get_parameter("absolute_registration_rmse_limit").as_double();
    enable_imu_motion_prediction_ =
        get_parameter("enable_imu_motion_prediction").as_bool();
    imu_topic_ = get_parameter("imu_topic").as_string();
    imu_prediction_point_time_field_ =
        get_parameter("imu_prediction_point_time_field").as_string();
    const auto imu_prediction_point_time_unit_param =
        get_parameter("imu_prediction_point_time_unit").as_string();
    const auto imu_prediction_cloud_stamp_location_param =
        get_parameter("imu_prediction_cloud_stamp_location").as_string();
    const auto imu_prediction_deskew_reference_param =
        get_parameter("imu_prediction_deskew_reference").as_string();
    imu_angular_velocity_scale_ =
        get_parameter("imu_angular_velocity_scale").as_double();
    enable_imu_prediction_gyro_bias_calibration_ =
        get_parameter("enable_imu_prediction_gyro_bias_calibration").as_bool();
    imu_prediction_gyro_bias_calibration_seconds_ =
        get_parameter("imu_prediction_gyro_bias_calibration_seconds").as_double();
    imu_prediction_gyro_bias_min_samples_ =
        static_cast<int>(std::max<int64_t>(1, get_parameter("imu_prediction_gyro_bias_min_samples").as_int()));
    const auto imu_prediction_gyro_bias_override =
        get_parameter("imu_prediction_gyro_bias").as_double_array();
    imu_prediction_max_gap_seconds_ =
        get_parameter("imu_prediction_max_gap_seconds").as_double();
    imu_prediction_max_age_seconds_ =
        get_parameter("imu_prediction_max_age_seconds").as_double();
    imu_prediction_max_rejected_frame_age_seconds_ =
        get_parameter("imu_prediction_max_rejected_frame_age_seconds").as_double();
    imu_prediction_rotation_only_ =
        get_parameter("imu_prediction_rotation_only").as_bool();
    imu_prediction_debug_ =
        get_parameter("imu_prediction_debug").as_bool();
    config_.enable_yaw_search_initializer =
        get_parameter("enable_yaw_search_initializer").as_bool();
    config_.yaw_search_degrees =
        get_parameter("yaw_search_degrees").as_double_array();
    config_.yaw_search_score_max_correspondence_distance =
        get_parameter("yaw_search_score_max_correspondence_distance").as_double();
    config_.yaw_search_min_correspondences =
        static_cast<int>(std::max<int64_t>(0, get_parameter("yaw_search_min_correspondences").as_int()));
    config_.yaw_search_use_weighted_rmse =
        get_parameter("yaw_search_use_weighted_rmse").as_bool();
    config_.yaw_search_vertical_axis =
        get_parameter("yaw_search_vertical_axis").as_string();
    config_.yaw_search_debug =
        get_parameter("yaw_search_debug").as_bool();
    config_.enable_motion_prior =
        get_parameter("enable_motion_prior").as_bool();
    config_.motion_prior_translation_sigma =
        get_parameter("motion_prior_translation_sigma").as_double();
    config_.motion_prior_z_sigma =
        get_parameter("motion_prior_z_sigma").as_double();
    config_.motion_prior_roll_pitch_sigma_deg =
        get_parameter("motion_prior_roll_pitch_sigma_deg").as_double();
    config_.motion_prior_yaw_sigma_deg =
        get_parameter("motion_prior_yaw_sigma_deg").as_double();
    config_.motion_prior_weight =
        get_parameter("motion_prior_weight").as_double();
    config_.motion_prior_apply_during_recovery =
        get_parameter("motion_prior_apply_during_recovery").as_bool();
    config_.motion_prior_debug =
        get_parameter("motion_prior_debug").as_bool();
    config_.enable_map_update_quality_gate =
        get_parameter("enable_map_update_quality_gate").as_bool();
    config_.map_update_max_weighted_rmse_ratio =
        get_parameter("map_update_max_weighted_rmse_ratio").as_double();
    config_.map_update_max_rmse =
        get_parameter("map_update_max_rmse").as_double();
    config_.map_update_max_translation_delta =
        get_parameter("map_update_max_translation_delta").as_double();
    config_.map_update_max_rotation_delta_deg =
        get_parameter("map_update_max_rotation_delta_deg").as_double();
    config_.map_update_min_correspondences =
        static_cast<int>(std::max<int64_t>(0, get_parameter("map_update_min_correspondences").as_int()));
    config_.map_update_debug =
        get_parameter("map_update_debug").as_bool();
    config_.enable_robust_icp_outlier_handling =
        get_parameter("enable_robust_icp_outlier_handling").as_bool();
    config_.robust_max_correspondence_distance =
        get_parameter("robust_max_correspondence_distance").as_double();
    config_.robust_residual_threshold =
        get_parameter("robust_residual_threshold").as_double();
    config_.robust_loss_type =
        ToLower(Trim(get_parameter("robust_loss_type").as_string()));
    config_.trimmed_icp_enabled =
        get_parameter("trimmed_icp_enabled").as_bool();
    config_.trimmed_icp_keep_ratio =
        get_parameter("trimmed_icp_keep_ratio").as_double();
    config_.robust_min_correspondences =
        static_cast<int>(std::max<int64_t>(0, get_parameter("robust_min_correspondences").as_int()));
    config_.robust_icp_debug =
        get_parameter("robust_icp_debug").as_bool();
    config_.enable_tentative_map_gating =
        get_parameter("enable_tentative_map_gating").as_bool();
    config_.tentative_voxel_size =
        get_parameter("tentative_voxel_size").as_double();
    config_.tentative_required_observations =
        static_cast<int>(std::max<int64_t>(1, get_parameter("tentative_required_observations").as_int()));
    config_.tentative_max_age_frames =
        static_cast<int>(std::max<int64_t>(1, get_parameter("tentative_max_age_frames").as_int()));
    config_.tentative_stable_support_radius =
        get_parameter("tentative_stable_support_radius").as_double();
    config_.use_tentative_points_for_icp =
        get_parameter("use_tentative_points_for_icp").as_bool();
    config_.insert_new_points_as_tentative =
        get_parameter("insert_new_points_as_tentative").as_bool();
    config_.promote_tentative_only_when_motion_is_calm =
        get_parameter("promote_tentative_only_when_motion_is_calm").as_bool();
    config_.dynamic_enable_max_delta_yaw_deg =
        get_parameter("dynamic_enable_max_delta_yaw_deg").as_double();
    config_.dynamic_relax_max_delta_yaw_deg =
        get_parameter("dynamic_relax_max_delta_yaw_deg").as_double();
    config_.map_update_allow_new_points_when_map_is_small =
        get_parameter("map_update_allow_new_points_when_map_is_small").as_bool();
    config_.map_update_min_stable_map_points =
        static_cast<int>(std::max<int64_t>(0, get_parameter("map_update_min_stable_map_points").as_int()));
    config_.tentative_map_debug =
        get_parameter("tentative_map_debug").as_bool();
    terminal_status_enabled_ = get_parameter("terminal_status").as_bool();
    publish_twist_ = get_parameter("publish_twist").as_bool();
    twist_in_child_frame_ = get_parameter("twist_in_child_frame").as_bool();
    twist_smoothing_alpha_ = get_parameter("twist_smoothing_alpha").as_double();
    twist_min_dt_ = get_parameter("twist_min_dt").as_double();
    twist_max_dt_ = get_parameter("twist_max_dt").as_double();
    twist_debug_ = get_parameter("twist_debug").as_bool();
    twist_linear_covariance_ = get_parameter("twist_linear_covariance").as_double();
    twist_angular_covariance_ = get_parameter("twist_angular_covariance").as_double();
    max_path_length_ =
        static_cast<size_t>(std::max<int64_t>(0, get_parameter("max_path_length").as_int()));
    if (max_path_length_ > 0) {
        path_msg_.poses.reserve(max_path_length_);
    }
    if (config_.max_range < config_.min_range) {
        RCLCPP_WARN(get_logger(), "[WARNING] max_range is smaller than min_range, settng min_range to 0.0");
        config_.min_range = 0.0;
    }

    if (const auto unit = ParseImuPredictionTimeUnit(imu_prediction_point_time_unit_param)) {
        imu_prediction_point_time_unit_ = *unit;
    } else {
        RCLCPP_WARN(get_logger(), "Invalid imu_prediction_point_time_unit '%s'; using nanoseconds",
                    imu_prediction_point_time_unit_param.c_str());
    }

    if (const auto location =
            ParseImuPredictionScanLocation(imu_prediction_cloud_stamp_location_param)) {
        imu_prediction_cloud_stamp_location_ = *location;
    } else {
        RCLCPP_WARN(get_logger(),
                    "Invalid imu_prediction_cloud_stamp_location '%s'; using end",
                    imu_prediction_cloud_stamp_location_param.c_str());
    }

    if (const auto reference =
            ParseImuPredictionScanLocation(imu_prediction_deskew_reference_param)) {
        imu_prediction_deskew_reference_ = *reference;
    } else {
        RCLCPP_WARN(get_logger(),
                    "Invalid imu_prediction_deskew_reference '%s'; using middle",
                    imu_prediction_deskew_reference_param.c_str());
    }

    if (!std::isfinite(imu_angular_velocity_scale_)) {
        RCLCPP_WARN(get_logger(), "Invalid imu_angular_velocity_scale; using 1.0");
        imu_angular_velocity_scale_ = 1.0;
    }
    imu_prediction_gyro_bias_calibration_seconds_ =
        std::max(0.0, imu_prediction_gyro_bias_calibration_seconds_);
    imu_prediction_max_gap_seconds_ = std::max(0.0, imu_prediction_max_gap_seconds_);
    imu_prediction_max_age_seconds_ = std::max(0.0, imu_prediction_max_age_seconds_);
    imu_prediction_max_rejected_frame_age_seconds_ =
        std::max(0.0, imu_prediction_max_rejected_frame_age_seconds_);
    imu_prediction_buffer_seconds_ =
        std::max(2.0,
                 std::max(imu_prediction_max_age_seconds_,
                          imu_prediction_max_rejected_frame_age_seconds_) + 1.0);
    config_.yaw_search_score_max_correspondence_distance =
        std::max(0.0, config_.yaw_search_score_max_correspondence_distance);
    config_.yaw_search_min_correspondences =
        std::max(0, config_.yaw_search_min_correspondences);
    if (config_.yaw_search_degrees.empty()) {
        config_.yaw_search_degrees.push_back(0.0);
    }
    config_.motion_prior_translation_sigma =
        std::max(1e-6, config_.motion_prior_translation_sigma);
    config_.motion_prior_z_sigma =
        std::max(1e-6, config_.motion_prior_z_sigma);
    config_.motion_prior_roll_pitch_sigma_deg =
        std::max(1e-6, config_.motion_prior_roll_pitch_sigma_deg);
    config_.motion_prior_yaw_sigma_deg =
        std::max(1e-6, config_.motion_prior_yaw_sigma_deg);
    config_.motion_prior_weight =
        std::max(0.0, config_.motion_prior_weight);
    config_.map_update_min_correspondences =
        std::max(0, config_.map_update_min_correspondences);
    if (!std::isfinite(config_.robust_max_correspondence_distance) ||
        config_.robust_max_correspondence_distance < 0.0) {
        RCLCPP_WARN(get_logger(), "Invalid robust_max_correspondence_distance; using 1.0");
        config_.robust_max_correspondence_distance = 1.0;
    }
    if (!std::isfinite(config_.robust_residual_threshold) ||
        config_.robust_residual_threshold <= 0.0) {
        RCLCPP_WARN(get_logger(), "Invalid robust_residual_threshold; using 0.35");
        config_.robust_residual_threshold = 0.35;
    }
    if (config_.robust_loss_type != "none" &&
        config_.robust_loss_type != "huber" &&
        config_.robust_loss_type != "cauchy" &&
        config_.robust_loss_type != "tukey") {
        RCLCPP_WARN(get_logger(),
                    "Invalid robust_loss_type '%s'; using cauchy",
                    config_.robust_loss_type.c_str());
        config_.robust_loss_type = "cauchy";
    }
    if (!std::isfinite(config_.trimmed_icp_keep_ratio)) {
        RCLCPP_WARN(get_logger(), "Invalid trimmed_icp_keep_ratio; using 0.80");
        config_.trimmed_icp_keep_ratio = 0.80;
    }
    config_.trimmed_icp_keep_ratio = std::clamp(config_.trimmed_icp_keep_ratio, 0.01, 1.0);
    config_.robust_min_correspondences =
        std::max(0, config_.robust_min_correspondences);
    if (!std::isfinite(config_.tentative_voxel_size) ||
        config_.tentative_voxel_size <= 0.0) {
        RCLCPP_WARN(get_logger(), "Invalid tentative_voxel_size; using 0.35");
        config_.tentative_voxel_size = 0.35;
    }
    config_.tentative_required_observations =
        std::max(1, config_.tentative_required_observations);
    config_.tentative_max_age_frames =
        std::max(1, config_.tentative_max_age_frames);
    if (!std::isfinite(config_.tentative_stable_support_radius) ||
        config_.tentative_stable_support_radius < 0.0) {
        RCLCPP_WARN(get_logger(), "Invalid tentative_stable_support_radius; using 0.45");
        config_.tentative_stable_support_radius = 0.45;
    }
    if (!std::isfinite(config_.dynamic_enable_max_delta_yaw_deg) ||
        config_.dynamic_enable_max_delta_yaw_deg < 0.0) {
        RCLCPP_WARN(get_logger(), "Invalid dynamic_enable_max_delta_yaw_deg; using 8.0");
        config_.dynamic_enable_max_delta_yaw_deg = 8.0;
    }
    if (!std::isfinite(config_.dynamic_relax_max_delta_yaw_deg) ||
        config_.dynamic_relax_max_delta_yaw_deg < config_.dynamic_enable_max_delta_yaw_deg) {
        RCLCPP_WARN(get_logger(),
                    "Invalid dynamic_relax_max_delta_yaw_deg; using enable threshold");
        config_.dynamic_relax_max_delta_yaw_deg =
            config_.dynamic_enable_max_delta_yaw_deg;
    }
    config_.map_update_min_stable_map_points =
        std::max(0, config_.map_update_min_stable_map_points);
    if (!std::isfinite(twist_smoothing_alpha_)) {
        RCLCPP_WARN(get_logger(), "Invalid twist_smoothing_alpha; using 1.0");
        twist_smoothing_alpha_ = 1.0;
    }
    twist_smoothing_alpha_ = std::clamp(twist_smoothing_alpha_, 0.0, 1.0);
    if (!std::isfinite(twist_min_dt_) || twist_min_dt_ < 0.0) {
        RCLCPP_WARN(get_logger(), "Invalid twist_min_dt; using 0.001");
        twist_min_dt_ = 0.001;
    }
    if (!std::isfinite(twist_max_dt_) || twist_max_dt_ <= twist_min_dt_) {
        RCLCPP_WARN(get_logger(), "Invalid twist_max_dt; using 1.0");
        twist_max_dt_ = std::max(1.0, twist_min_dt_ + 1.0);
    }
    if (!std::isfinite(twist_linear_covariance_) || twist_linear_covariance_ < 0.0) {
        RCLCPP_WARN(get_logger(), "Invalid twist_linear_covariance; using 0.25");
        twist_linear_covariance_ = 0.25;
    }
    if (!std::isfinite(twist_angular_covariance_) || twist_angular_covariance_ < 0.0) {
        RCLCPP_WARN(get_logger(), "Invalid twist_angular_covariance; using 0.25");
        twist_angular_covariance_ = 0.25;
    }

    if (!imu_prediction_rotation_only_) {
        RCLCPP_WARN(get_logger(),
                    "imu_prediction_rotation_only=false requested, but only rotation-only IMU "
                    "prediction is implemented; keeping translation prediction from GenZ-ICP");
        imu_prediction_rotation_only_ = true;
    }

    imu_prediction_gyro_bias_ = Eigen::Vector3d::Zero();
    imu_prediction_gyro_bias_ready_ = false;
    imu_prediction_gyro_bias_manual_override_ = false;
    if (!imu_prediction_gyro_bias_override.empty()) {
        if (imu_prediction_gyro_bias_override.size() == 3 &&
            AllFinite(imu_prediction_gyro_bias_override)) {
            imu_prediction_gyro_bias_ = Eigen::Vector3d(imu_prediction_gyro_bias_override[0],
                                                        imu_prediction_gyro_bias_override[1],
                                                        imu_prediction_gyro_bias_override[2]);
            imu_prediction_gyro_bias_ready_ = true;
            imu_prediction_gyro_bias_manual_override_ = true;
            RCLCPP_INFO(get_logger(), "Using manual imu_prediction_gyro_bias=%s rad/s",
                        FormatVector(imu_prediction_gyro_bias_).c_str());
        } else {
            RCLCPP_WARN(get_logger(),
                        "Ignoring imu_prediction_gyro_bias override: expected [x, y, z] finite values");
        }
    }
    if (!imu_prediction_gyro_bias_manual_override_) {
        imu_prediction_gyro_bias_ready_ = !enable_imu_prediction_gyro_bias_calibration_;
    }
    // clang-format on

    // Construct the main GenZ-ICP odometry node
    odometry_ = genz_icp::pipeline::GenZICP(config_);
    odometry_.SetTerminalStatusEnabled(terminal_status_enabled_);
    RCLCPP_INFO(this->get_logger(), "LiDAR deskew is %s", config_.deskew ? "enabled" : "disabled");
    RCLCPP_INFO(get_logger(),
                "Robust ICP outlier handling is %s: max_corr=%.3f residual_threshold=%.3f "
                "loss=%s trimmed=%s keep_ratio=%.2f min_correspondences=%d",
                config_.enable_robust_icp_outlier_handling ? "enabled" : "disabled",
                config_.robust_max_correspondence_distance,
                config_.robust_residual_threshold,
                config_.robust_loss_type.c_str(),
                config_.trimmed_icp_enabled ? "true" : "false",
                config_.trimmed_icp_keep_ratio,
                config_.robust_min_correspondences);
    RCLCPP_INFO(get_logger(),
                "Tentative map gating is %s: voxel_size=%.3f required_observations=%d "
                "max_age_frames=%d support_radius=%.3f use_tentative_for_icp=%s",
                config_.enable_tentative_map_gating ? "enabled" : "disabled",
                config_.tentative_voxel_size,
                config_.tentative_required_observations,
                config_.tentative_max_age_frames,
                config_.tentative_stable_support_radius,
                config_.use_tentative_points_for_icp ? "true" : "false");

    // Initialize subscribers
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud_topic", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        std::bind(&OdometryServer::RegisterFrame, this, std::placeholders::_1));
    if (enable_imu_motion_prediction_) {
        imu_prediction_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, rclcpp::SensorDataQoS(),
            std::bind(&OdometryServer::ImuPredictionCallback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(),
                    "IMU motion prediction enabled: imu=%s scale=%.12f bias_calibration=%s "
                    "point_time_field=%s",
                    imu_topic_.c_str(), imu_angular_velocity_scale_,
                    enable_imu_prediction_gyro_bias_calibration_ &&
                            !imu_prediction_gyro_bias_manual_override_
                        ? "enabled"
                        : "disabled",
                    imu_prediction_point_time_field_.c_str());
        RCLCPP_INFO(get_logger(),
                    "IMU prediction uses angular_velocity only; orientation and "
                    "linear_acceleration are ignored");
    }

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

void OdometryServer::ImuPredictionCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
    const double stamp = rclcpp::Time(msg->header.stamp).seconds();
    const Eigen::Vector3d raw_angular_velocity(msg->angular_velocity.x,
                                               msg->angular_velocity.y,
                                               msg->angular_velocity.z);
    const Eigen::Vector3d scaled_angular_velocity =
        imu_angular_velocity_scale_ * raw_angular_velocity;
    if (!std::isfinite(stamp) ||
        !raw_angular_velocity.allFinite() ||
        !scaled_angular_velocity.allFinite()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Dropping IMU prediction sample with non-finite time or angular velocity");
        return;
    }

    std::lock_guard<std::mutex> lock(imu_prediction_mutex_);
    if (!imu_prediction_gyro_bias_ready_) {
        UpdateImuPredictionGyroBiasCalibration(stamp, raw_angular_velocity,
                                               scaled_angular_velocity);
        if (!imu_prediction_gyro_bias_ready_) {
            return;
        }
    }

    const Eigen::Vector3d angular_velocity =
        scaled_angular_velocity - imu_prediction_gyro_bias_;
    if (!angular_velocity.allFinite()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Dropping IMU prediction sample with non-finite bias-corrected angular velocity");
        return;
    }

    latest_imu_prediction_stamp_ = std::max(latest_imu_prediction_stamp_, stamp);
    if (stamp < latest_imu_prediction_stamp_ - imu_prediction_buffer_seconds_) {
        return;
    }

    InsertImuPredictionSample(ImuPredictionSample{stamp, angular_velocity});

    const double cutoff = latest_imu_prediction_stamp_ - imu_prediction_buffer_seconds_;
    while (!imu_prediction_buffer_.empty() && imu_prediction_buffer_.front().stamp < cutoff) {
        imu_prediction_buffer_.pop_front();
    }
}

void OdometryServer::UpdateImuPredictionGyroBiasCalibration(
    const double stamp,
    const Eigen::Vector3d &raw_angular_velocity,
    const Eigen::Vector3d &scaled_angular_velocity) {
    if (std::isnan(imu_prediction_gyro_bias_calibration_start_)) {
        imu_prediction_gyro_bias_calibration_start_ = stamp;
        RCLCPP_INFO(get_logger(),
                    "Starting IMU prediction gyro bias calibration: duration=%.3fs min_samples=%d",
                    imu_prediction_gyro_bias_calibration_seconds_,
                    imu_prediction_gyro_bias_min_samples_);
    }

    imu_prediction_gyro_bias_raw_sum_ += raw_angular_velocity;
    imu_prediction_gyro_bias_scaled_sum_ += scaled_angular_velocity;
    ++imu_prediction_gyro_bias_sample_count_;

    const double elapsed =
        std::max(0.0, stamp - imu_prediction_gyro_bias_calibration_start_);
    if (elapsed < imu_prediction_gyro_bias_calibration_seconds_ ||
        imu_prediction_gyro_bias_sample_count_ <
            static_cast<size_t>(imu_prediction_gyro_bias_min_samples_)) {
        return;
    }

    const double inv_count =
        1.0 / static_cast<double>(imu_prediction_gyro_bias_sample_count_);
    const Eigen::Vector3d raw_mean =
        imu_prediction_gyro_bias_raw_sum_ * inv_count;
    const Eigen::Vector3d scaled_mean =
        imu_prediction_gyro_bias_scaled_sum_ * inv_count;

    imu_prediction_gyro_bias_ = scaled_mean;
    imu_prediction_gyro_bias_ready_ = true;
    RCLCPP_INFO(get_logger(),
                "IMU prediction gyro bias calibration complete: samples=%zu elapsed=%.3fs",
                imu_prediction_gyro_bias_sample_count_, elapsed);
    RCLCPP_INFO(get_logger(), "IMU prediction raw omega mean=%s",
                FormatVector(raw_mean).c_str());
    RCLCPP_INFO(get_logger(), "IMU prediction scaled omega mean=%s rad/s",
                FormatVector(scaled_mean).c_str());
    RCLCPP_INFO(get_logger(), "IMU prediction final gyro_bias=%s rad/s",
                FormatVector(imu_prediction_gyro_bias_).c_str());
}

void OdometryServer::InsertImuPredictionSample(const ImuPredictionSample &sample) {
    const auto it = std::lower_bound(
        imu_prediction_buffer_.begin(), imu_prediction_buffer_.end(), sample.stamp,
        [](const ImuPredictionSample &candidate, const double value) {
            return candidate.stamp < value;
        });

    if (it != imu_prediction_buffer_.end() &&
        std::abs(it->stamp - sample.stamp) <= kTimeEpsilon) {
        *it = sample;
    } else if (it != imu_prediction_buffer_.begin() &&
               std::abs(std::prev(it)->stamp - sample.stamp) <= kTimeEpsilon) {
        *std::prev(it) = sample;
    } else {
        imu_prediction_buffer_.insert(it, sample);
    }
}

std::optional<double> OdometryServer::ComputeScanReferenceTime(
    const sensor_msgs::msg::PointCloud2 &msg,
    std::string *reason) const {
    std::vector<std::string> candidates;
    const auto requested = ToLower(Trim(imu_prediction_point_time_field_));
    if (requested.empty() || requested == "auto") {
        candidates = {"time", "timestamp", "t", "offset_time", "point_time_offset",
                      "time_offset"};
    } else {
        candidates = SplitCandidates(imu_prediction_point_time_field_);
        if (candidates.empty()) candidates.push_back(imu_prediction_point_time_field_);
    }

    const PointField *time_field = nullptr;
    for (const auto &candidate : candidates) {
        if (const auto *field = FindField(msg, candidate)) {
            time_field = field;
            break;
        }
    }
    if (time_field == nullptr) {
        if (reason) {
            *reason = "point time field not found; available fields: " +
                      AvailableFieldsString(msg);
        }
        return std::nullopt;
    }

    std::vector<double> offsets_seconds;
    try {
        offsets_seconds =
            ExtractFieldSeconds(msg, *time_field, imu_prediction_point_time_unit_);
    } catch (const std::exception &ex) {
        if (reason) {
            *reason = std::string("failed to read point time field '") +
                      time_field->name + "': " + ex.what();
        }
        return std::nullopt;
    }

    const size_t n_points = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
    if (offsets_seconds.size() != n_points || offsets_seconds.empty() ||
        !AllFinite(offsets_seconds)) {
        if (reason) *reason = "point time field has invalid size or non-finite values";
        return std::nullopt;
    }

    const auto [min_it, max_it] =
        std::minmax_element(offsets_seconds.begin(), offsets_seconds.end());
    const double min_offset = *min_it;
    const double max_offset = *max_it;
    const double header_time = rclcpp::Time(msg.header.stamp).seconds();

    double scan_start = 0.0;
    double duration = 0.0;
    if (min_offset > kAbsoluteUnixTimeThreshold) {
        scan_start = min_offset;
        duration = max_offset - min_offset;
    } else if (min_offset < -kTimeEpsilon) {
        const double observed_start = header_time + min_offset;
        const double observed_end = header_time + max_offset;
        duration = observed_end - observed_start;
        scan_start =
            header_time - LocationOffset(imu_prediction_cloud_stamp_location_, duration);
    } else {
        duration = max_offset;
        scan_start =
            header_time - LocationOffset(imu_prediction_cloud_stamp_location_, duration);
    }

    if (!(duration > kTimeEpsilon)) {
        if (reason) *reason = "scan duration from point time field is zero or invalid";
        return std::nullopt;
    }

    const double reference_time =
        scan_start + LocationOffset(imu_prediction_deskew_reference_, duration);
    if (!std::isfinite(reference_time)) {
        if (reason) *reason = "computed scan reference time is non-finite";
        return std::nullopt;
    }
    return reference_time;
}

std::optional<ImuPredictionResult> OdometryServer::BuildImuMotionPrediction(
    const double current_reference_time,
    std::string *reason) const {
    if (!previous_accepted_scan_reference_time_) {
        if (reason) *reason = "no previous accepted scan reference time";
        return std::nullopt;
    }

    const double previous_reference_time = *previous_accepted_scan_reference_time_;
    if (!std::isfinite(previous_reference_time) || !std::isfinite(current_reference_time)) {
        if (reason) *reason = "non-finite scan reference time";
        return std::nullopt;
    }

    if (current_reference_time < previous_reference_time - kTimeEpsilon) {
        if (reason) *reason = "current scan reference time is older than previous accepted scan";
        return std::nullopt;
    }

    const double dt_total = current_reference_time - previous_reference_time;
    if (dt_total <= kTimeEpsilon) {
        return ImuPredictionResult{Sophus::SO3d(), 0.0, 0};
    }

    const bool using_rejected_frame_age =
        odometry_.ConsecutiveRegistrationRejections() > 0;
    const double max_prediction_age =
        using_rejected_frame_age
            ? imu_prediction_max_rejected_frame_age_seconds_
            : imu_prediction_max_age_seconds_;
    if (max_prediction_age > 0.0 && dt_total > max_prediction_age) {
        if (reason) {
            *reason = "prediction interval " + std::to_string(dt_total) +
                      "s exceeds " +
                      (using_rejected_frame_age
                           ? "imu_prediction_max_rejected_frame_age_seconds="
                           : "imu_prediction_max_age_seconds=") +
                      std::to_string(max_prediction_age);
        }
        return std::nullopt;
    }

    std::vector<ImuPredictionSample> samples;
    {
        std::lock_guard<std::mutex> lock(imu_prediction_mutex_);
        if (!imu_prediction_gyro_bias_ready_) {
            if (reason) *reason = "gyro bias calibration in progress";
            return std::nullopt;
        }
        samples.assign(imu_prediction_buffer_.begin(), imu_prediction_buffer_.end());
    }

    if (samples.size() < 2) {
        if (reason) *reason = "not enough IMU prediction samples buffered";
        return std::nullopt;
    }

    const auto first_after_start = std::upper_bound(
        samples.begin(), samples.end(), previous_reference_time,
        [](const double value, const ImuPredictionSample &sample) {
            return value < sample.stamp;
        });
    if (first_after_start == samples.begin()) {
        if (reason) {
            *reason = "IMU prediction buffer does not cover previous accepted reference; first IMU=" +
                      std::to_string(samples.front().stamp) +
                      " requested=" + std::to_string(previous_reference_time);
        }
        return std::nullopt;
    }
    const size_t first_index =
        static_cast<size_t>(std::distance(samples.begin(), first_after_start) - 1);

    const auto first_at_or_after_end = std::lower_bound(
        samples.begin(), samples.end(), current_reference_time,
        [](const ImuPredictionSample &sample, const double value) {
            return sample.stamp < value;
        });
    if (first_at_or_after_end == samples.end()) {
        if (reason) {
            *reason = "IMU prediction buffer does not cover current reference; last IMU=" +
                      std::to_string(samples.back().stamp) +
                      " requested=" + std::to_string(current_reference_time);
        }
        return std::nullopt;
    }
    const size_t last_index =
        static_cast<size_t>(std::distance(samples.begin(), first_at_or_after_end));

    Sophus::SO3d rotation;
    for (size_t i = first_index; i < last_index; ++i) {
        const double t0 = samples[i].stamp;
        const double t1 = samples[i + 1].stamp;
        const double dt = t1 - t0;
        if (!(dt > 0.0)) {
            if (reason) *reason = "IMU prediction samples are not strictly time ordered";
            return std::nullopt;
        }

        const bool overlaps_requested_interval =
            t0 < current_reference_time && t1 > previous_reference_time;
        if (imu_prediction_max_gap_seconds_ > 0.0 && overlaps_requested_interval &&
            dt > imu_prediction_max_gap_seconds_) {
            if (reason) {
                *reason = "IMU prediction gap " + std::to_string(dt) +
                          "s exceeds imu_prediction_max_gap_seconds=" +
                          std::to_string(imu_prediction_max_gap_seconds_);
            }
            return std::nullopt;
        }

        const double segment_start = std::max(previous_reference_time, t0);
        const double segment_end = std::min(current_reference_time, t1);
        if (segment_end <= segment_start) {
            continue;
        }

        const double alpha_start = (segment_start - t0) / dt;
        const double alpha_end = (segment_end - t0) / dt;
        const Eigen::Vector3d omega_start =
            samples[i].angular_velocity +
            alpha_start * (samples[i + 1].angular_velocity - samples[i].angular_velocity);
        const Eigen::Vector3d omega_end =
            samples[i].angular_velocity +
            alpha_end * (samples[i + 1].angular_velocity - samples[i].angular_velocity);

        // Body-frame gyro integration uses right composition:
        // R(t + dt) = R(t) * Exp(omega_body * dt).
        const Eigen::Vector3d omega_mid = 0.5 * (omega_start + omega_end);
        rotation = rotation * Sophus::SO3d::exp(omega_mid * (segment_end - segment_start));
    }

    if (!rotation.matrix().allFinite()) {
        if (reason) *reason = "IMU prediction rotation is non-finite";
        return std::nullopt;
    }

    return ImuPredictionResult{rotation, dt_total, last_index - first_index + 1};
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

    std::optional<double> current_scan_reference_time;
    std::optional<Sophus::SO3d> imu_rotation_prediction;
    std::string initial_guess_source = "normal";
    if (enable_imu_motion_prediction_) {
        std::string timing_reason;
        current_scan_reference_time = ComputeScanReferenceTime(*msg, &timing_reason);
        if (!current_scan_reference_time) {
            if (imu_prediction_debug_) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "IMU prediction unavailable: reason=%s", timing_reason.c_str());
            }
        } else {
            std::string prediction_reason;
            const auto prediction =
                BuildImuMotionPrediction(*current_scan_reference_time, &prediction_reason);
            if (prediction) {
                imu_rotation_prediction = prediction->rotation;
                initial_guess_source = "imu_prediction";
                if (imu_prediction_debug_) {
                    RCLCPP_INFO(get_logger(),
                                "IMU prediction: available=true dt=%.4f samples=%zu "
                                "rotation_delta_deg=%.3f",
                                prediction->dt, prediction->sample_count,
                                prediction->rotation.log().norm() * kRadiansToDegrees);
                }
            } else if (imu_prediction_debug_) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "IMU prediction unavailable: reason=%s", prediction_reason.c_str());
            }
        }
    }
    if (enable_imu_motion_prediction_ &&
        imu_prediction_debug_ &&
        !config_.enable_yaw_search_initializer) {
        RCLCPP_INFO(get_logger(), "ICP initial guess source: %s",
                    initial_guess_source.c_str());
    }

    // Register frame, main entry point to GenZ-ICP pipeline
    const auto registration_output =
        odometry_.RegisterFrame(points, timestamps, imu_rotation_prediction);
    const auto &[planar_points, non_planar_points, covariance] = registration_output;

    if (enable_imu_motion_prediction_ && current_scan_reference_time) {
        if (odometry_.LastFrameAccepted()) {
            previous_accepted_scan_reference_time_ = *current_scan_reference_time;
        } else if (imu_prediction_debug_) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "IMU prediction reference not advanced because registration was rejected");
        }
    }

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

    FillTwist(odom_msg, pose, stamp);

    odom_publisher_->publish(std::move(odom_msg));
}

void OdometryServer::FillTwist(nav_msgs::msg::Odometry &odom_msg,
                               const Sophus::SE3d &pose,
                               const rclcpp::Time &stamp) {
    odom_msg.twist.covariance.fill(0.0);
    if (!publish_twist_) {
        return;
    }

    odom_msg.twist.covariance[0] = twist_linear_covariance_;
    odom_msg.twist.covariance[7] = twist_linear_covariance_;
    odom_msg.twist.covariance[14] = twist_linear_covariance_;
    odom_msg.twist.covariance[21] = twist_angular_covariance_;
    odom_msg.twist.covariance[28] = twist_angular_covariance_;
    odom_msg.twist.covariance[35] = twist_angular_covariance_;

    const auto publish_velocity = [&](const Eigen::Vector3d &linear,
                                      const Eigen::Vector3d &angular) {
        odom_msg.twist.twist.linear.x = linear.x();
        odom_msg.twist.twist.linear.y = linear.y();
        odom_msg.twist.twist.linear.z = linear.z();
        odom_msg.twist.twist.angular.x = angular.x();
        odom_msg.twist.twist.angular.y = angular.y();
        odom_msg.twist.twist.angular.z = angular.z();
    };

    const auto publish_smoothed_or_zero = [&]() {
        if (has_smoothed_twist_) {
            publish_velocity(smoothed_linear_velocity_, smoothed_angular_velocity_);
        } else {
            publish_velocity(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
        }
    };

    const auto store_previous_pose = [&](const Eigen::Vector3d &position,
                                         const Eigen::Quaterniond &orientation) {
        previous_twist_stamp_ = stamp;
        previous_twist_position_ = position;
        previous_twist_orientation_ = orientation;
        has_previous_twist_pose_ = true;
    };

    const Eigen::Vector3d current_position = pose.translation();
    Eigen::Quaterniond current_orientation(pose.so3().unit_quaternion());
    current_orientation.normalize();

    if (!current_position.allFinite() || !current_orientation.coeffs().allFinite()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Cannot compute odometry twist from non-finite pose");
        publish_smoothed_or_zero();
        return;
    }

    if (!has_previous_twist_pose_) {
        publish_velocity(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
        store_previous_pose(current_position, current_orientation);
        return;
    }

    double dt = 0.0;
    try {
        dt = (stamp - previous_twist_stamp_).seconds();
    } catch (const std::exception &ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Cannot compute odometry twist dt: %s", ex.what());
        publish_smoothed_or_zero();
        store_previous_pose(current_position, current_orientation);
        return;
    }

    if (!std::isfinite(dt)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Cannot compute odometry twist from non-finite dt");
        publish_smoothed_or_zero();
        store_previous_pose(current_position, current_orientation);
        return;
    }

    if (dt <= twist_min_dt_) {
        publish_smoothed_or_zero();
        return;
    }

    if (dt > twist_max_dt_) {
        smoothed_linear_velocity_.setZero();
        smoothed_angular_velocity_.setZero();
        has_smoothed_twist_ = false;
        publish_velocity(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
        store_previous_pose(current_position, current_orientation);
        return;
    }

    const Eigen::Vector3d linear_world =
        (current_position - previous_twist_position_) / dt;
    const Eigen::Matrix3d rotation_world_body =
        current_orientation.normalized().toRotationMatrix();
    const Eigen::Vector3d computed_linear_velocity =
        twist_in_child_frame_ ? rotation_world_body.transpose() * linear_world : linear_world;

    Eigen::Quaterniond previous_orientation = previous_twist_orientation_.normalized();
    Eigen::Quaterniond delta_orientation =
        previous_orientation.conjugate() * current_orientation.normalized();
    delta_orientation.normalize();
    if (delta_orientation.w() < 0.0) {
        delta_orientation.coeffs() *= -1.0;
    }

    const Eigen::AngleAxisd angle_axis(delta_orientation);
    const Eigen::Vector3d angular_body =
        angle_axis.axis() * angle_axis.angle() / dt;
    const Eigen::Vector3d computed_angular_velocity =
        twist_in_child_frame_ ? angular_body : rotation_world_body * angular_body;

    if (!computed_linear_velocity.allFinite() || !computed_angular_velocity.allFinite()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Cannot compute odometry twist from non-finite velocity");
        publish_smoothed_or_zero();
        store_previous_pose(current_position, current_orientation);
        return;
    }

    if (!has_smoothed_twist_) {
        smoothed_linear_velocity_ = computed_linear_velocity;
        smoothed_angular_velocity_ = computed_angular_velocity;
        has_smoothed_twist_ = true;
    } else {
        const double alpha = twist_smoothing_alpha_;
        smoothed_linear_velocity_ =
            alpha * computed_linear_velocity +
            (1.0 - alpha) * smoothed_linear_velocity_;
        smoothed_angular_velocity_ =
            alpha * computed_angular_velocity +
            (1.0 - alpha) * smoothed_angular_velocity_;
    }

    publish_velocity(smoothed_linear_velocity_, smoothed_angular_velocity_);
    store_previous_pose(current_position, current_orientation);

    if (twist_debug_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "twist dt=%.3fs v=%s m/s omega=%s rad/s yaw_rate=%.3f deg/s",
                             dt,
                             FormatVector(smoothed_linear_velocity_).c_str(),
                             FormatVector(smoothed_angular_velocity_).c_str(),
                             smoothed_angular_velocity_.z() * kRadiansToDegrees);
    }
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
