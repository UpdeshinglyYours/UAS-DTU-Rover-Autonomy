// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss.
// Modified by Daehan Lee, Hyungtae Lim, and Soohee Han, 2024

#include <Eigen/Core>
#include <sophus/so3.hpp>

#include "genz_icp/core/ImuIntegration.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace genz_icp_ros {
namespace {

using Imu = sensor_msgs::msg::Imu;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;

constexpr double kAbsoluteUnixTimeThreshold = 1.0e8;
constexpr double kTimeEpsilon = 1.0e-9;
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

enum class TimeUnit { Seconds, Milliseconds, Microseconds, Nanoseconds };
enum class ScanLocation { Start, Middle, End };

struct ImuSample {
    double stamp{0.0};
    Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gravity_direction{Eigen::Vector3d::UnitZ()};
    bool gravity_direction_valid{false};
};

struct ScanTiming {
    std::vector<double> point_times;
    double scan_start{0.0};
    double reference_time{0.0};
    double scan_end{0.0};
    double duration{0.0};
    double min_point_time{0.0};
    double max_point_time{0.0};
    bool absolute_point_times{false};
    bool offsets_relative_to_header{false};
};

struct OrientationTrajectory {
    std::vector<double> stamps;
    std::vector<Eigen::Vector3d> angular_velocities;
    std::vector<Sophus::SO3d> rotations;
};

struct PendingCloud {
    PointCloud2::ConstSharedPtr msg;
    ScanTiming timing;
    std::string time_field_name;
    size_t point_count{0};
    double queued_at{0.0};
    size_t sequence{0};
};

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

std::optional<TimeUnit> ParseTimeUnit(const std::string &unit) {
    const auto normalized = ToLower(Trim(unit));
    if (normalized == "seconds" || normalized == "second" || normalized == "sec" ||
        normalized == "s") {
        return TimeUnit::Seconds;
    }
    if (normalized == "milliseconds" || normalized == "millisecond" || normalized == "msec" ||
        normalized == "ms") {
        return TimeUnit::Milliseconds;
    }
    if (normalized == "microseconds" || normalized == "microsecond" || normalized == "usec" ||
        normalized == "us") {
        return TimeUnit::Microseconds;
    }
    if (normalized == "nanoseconds" || normalized == "nanosecond" || normalized == "nsec" ||
        normalized == "ns") {
        return TimeUnit::Nanoseconds;
    }
    return std::nullopt;
}

std::optional<ScanLocation> ParseScanLocation(const std::string &location) {
    const auto normalized = ToLower(Trim(location));
    if (normalized == "start" || normalized == "begin" || normalized == "beginning") {
        return ScanLocation::Start;
    }
    if (normalized == "middle" || normalized == "mid" || normalized == "midpoint" ||
        normalized == "center" || normalized == "centre") {
        return ScanLocation::Middle;
    }
    if (normalized == "end" || normalized == "finish" || normalized == "last") {
        return ScanLocation::End;
    }
    return std::nullopt;
}

double UnitScale(const TimeUnit unit) {
    switch (unit) {
        case TimeUnit::Seconds:
            return 1.0;
        case TimeUnit::Milliseconds:
            return 1.0e-3;
        case TimeUnit::Microseconds:
            return 1.0e-6;
        case TimeUnit::Nanoseconds:
            return 1.0e-9;
    }
    return 1.0;
}

double LocationOffset(const ScanLocation location, const double duration) {
    switch (location) {
        case ScanLocation::Start:
            return 0.0;
        case ScanLocation::Middle:
            return 0.5 * duration;
        case ScanLocation::End:
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

bool HasFloat32Field(const PointCloud2 &cloud, const std::string &name) {
    const auto *field = FindField(cloud, name);
    return field != nullptr && field->datatype == PointField::FLOAT32;
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
                                        const TimeUnit unit) {
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

bool IsTemporaryImuCoverageError(const std::string &error) {
    return error.find("Not enough IMU samples buffered") != std::string::npos ||
           error.find("IMU buffer does not cover scan start/reference time") != std::string::npos ||
           error.find("IMU buffer does not cover scan end/reference time") != std::string::npos;
}

}  // namespace

class ImuRotationDeskewNode final : public rclcpp::Node {
public:
    ImuRotationDeskewNode() : rclcpp::Node("imu_rotation_deskew_node") {
        cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/bf_lidar/point_cloud_out");
        imu_topic_ = declare_parameter<std::string>("imu_topic", "/bf_lidar/imu_out");
        output_topic_ =
            declare_parameter<std::string>("output_topic", "/bf_lidar/point_cloud_deskewed");
        point_time_field_param_ = declare_parameter<std::string>("point_time_field", "auto");
        const auto point_time_unit_param =
            declare_parameter<std::string>("point_time_unit", "seconds");
        const auto cloud_stamp_location_param =
            declare_parameter<std::string>("cloud_stamp_location", "start");
        const auto deskew_reference_param =
            declare_parameter<std::string>("deskew_reference", "middle");
        imu_buffer_seconds_ = declare_parameter<double>("imu_buffer_seconds", 2.0);
        max_imu_gap_seconds_ = declare_parameter<double>("max_imu_gap_seconds", 0.03);
        debug_print_ = declare_parameter<bool>("debug_print", false);
        invert_correction_ = declare_parameter<bool>("invert_correction", false);
        enable_lidar_lever_arm_correction_ =
            declare_parameter<bool>("enable_lidar_lever_arm_correction", false);
        const auto lidar_lever_arm_param =
            declare_parameter<std::vector<double>>("lidar_lever_arm",
                                                   std::vector<double>{0.0, 0.0, 0.0});
        imu_angular_velocity_scale_ =
            declare_parameter<double>("imu_angular_velocity_scale", 1.0);
        enable_gyro_bias_calibration_ =
            declare_parameter<bool>("enable_gyro_bias_calibration", false);
        gyro_bias_calibration_seconds_ =
            declare_parameter<double>("gyro_bias_calibration_seconds", 2.0);
        gyro_bias_min_samples_ = declare_parameter<int>("gyro_bias_min_samples", 50);
        const auto gyro_bias_override =
            declare_parameter<std::vector<double>>("gyro_bias", std::vector<double>{});
        print_gyro_stats_ = declare_parameter<bool>("print_gyro_stats", true);
        publish_raw_during_gyro_bias_calibration_ =
            declare_parameter<bool>("publish_raw_during_gyro_bias_calibration", true);
        enable_pending_cloud_queue_ =
            declare_parameter<bool>("enable_pending_cloud_queue", true);
        max_pending_cloud_wait_seconds_ =
            declare_parameter<double>("max_pending_cloud_wait_seconds", 0.15);
        publish_raw_on_missing_imu_after_wait_ =
            declare_parameter<bool>("publish_raw_on_missing_imu_after_wait", false);
        drop_cloud_on_missing_imu_after_wait_ =
            declare_parameter<bool>("drop_cloud_on_missing_imu_after_wait", true);
        max_pending_clouds_ = declare_parameter<int>("max_pending_clouds", 20);
        enable_translational_deskew_ =
            declare_parameter<bool>("enable_translational_deskew", false);
        bias_topic_ = declare_parameter<std::string>("bias_topic", "/genz/imu/bias");
        velocity_topic_ =
            declare_parameter<std::string>("velocity_topic", "/genz/odometry");
        translation_max_velocity_age_seconds_ =
            declare_parameter<double>("translation_max_velocity_age_seconds", 0.5);
        translation_max_acceleration_ =
            declare_parameter<double>("translation_max_acceleration", 5.0);
        translation_max_velocity_ =
            declare_parameter<double>("translation_max_velocity", 5.0);
        gravity_magnitude_ =
            declare_parameter<double>("gravity_magnitude", genz_icp::imu::kStandardGravity);
        gravity_correction_gain_ =
            declare_parameter<double>("gravity_correction_gain", 1.0);
        use_imu_orientation_for_gravity_ =
            declare_parameter<bool>("use_imu_orientation_for_gravity", true);

        if (const auto unit = ParseTimeUnit(point_time_unit_param)) {
            point_time_unit_ = *unit;
        } else {
            RCLCPP_WARN(get_logger(), "Invalid point_time_unit '%s'; using seconds",
                        point_time_unit_param.c_str());
        }

        if (const auto location = ParseScanLocation(cloud_stamp_location_param)) {
            cloud_stamp_location_ = *location;
        } else {
            RCLCPP_WARN(get_logger(), "Invalid cloud_stamp_location '%s'; using start",
                        cloud_stamp_location_param.c_str());
        }

        if (const auto reference = ParseScanLocation(deskew_reference_param)) {
            deskew_reference_ = *reference;
        } else {
            RCLCPP_WARN(get_logger(), "Invalid deskew_reference '%s'; using middle",
                        deskew_reference_param.c_str());
        }

        imu_buffer_seconds_ = std::max(0.1, imu_buffer_seconds_);
        max_imu_gap_seconds_ = std::max(0.0, max_imu_gap_seconds_);
        if (!std::isfinite(imu_angular_velocity_scale_)) {
            RCLCPP_WARN(get_logger(), "Invalid imu_angular_velocity_scale; using 1.0");
            imu_angular_velocity_scale_ = 1.0;
        }
        gyro_bias_calibration_seconds_ = std::max(0.0, gyro_bias_calibration_seconds_);
        gyro_bias_min_samples_ = std::max(1, gyro_bias_min_samples_);
        max_pending_cloud_wait_seconds_ = std::max(0.0, max_pending_cloud_wait_seconds_);
        max_pending_clouds_ = std::max(1, max_pending_clouds_);
        translation_max_velocity_age_seconds_ =
            std::max(0.0, translation_max_velocity_age_seconds_);
        translation_max_acceleration_ = std::max(0.0, translation_max_acceleration_);
        translation_max_velocity_ = std::max(0.0, translation_max_velocity_);
        if (!std::isfinite(gravity_magnitude_) || gravity_magnitude_ < 1.0) {
            gravity_magnitude_ = genz_icp::imu::kStandardGravity;
        }
        gravity_correction_gain_ = std::clamp(gravity_correction_gain_, 0.0, 1.0);

        if (lidar_lever_arm_param.size() == 3 && AllFinite(lidar_lever_arm_param)) {
            lidar_lever_arm_ = Eigen::Vector3d(lidar_lever_arm_param[0],
                                               lidar_lever_arm_param[1],
                                               lidar_lever_arm_param[2]);
        } else {
            RCLCPP_WARN(get_logger(),
                        "Invalid lidar_lever_arm parameter: expected exactly 3 finite values; "
                        "using [0.0, 0.0, 0.0]");
            lidar_lever_arm_ = Eigen::Vector3d::Zero();
        }

        if (!gyro_bias_override.empty()) {
            if (gyro_bias_override.size() == 3 && AllFinite(gyro_bias_override)) {
                gyro_bias_ = Eigen::Vector3d(gyro_bias_override[0],
                                             gyro_bias_override[1],
                                             gyro_bias_override[2]);
                gyro_bias_ready_ = true;
                gyro_bias_manual_override_ = true;
                if (print_gyro_stats_) {
                    RCLCPP_INFO(get_logger(), "Using manual gyro_bias=%s rad/s",
                                FormatVector(gyro_bias_).c_str());
                }
            } else {
                RCLCPP_WARN(get_logger(),
                            "Ignoring gyro_bias override: expected [x, y, z] finite values");
            }
        }

        if (!gyro_bias_manual_override_) {
            gyro_bias_ready_ = !enable_gyro_bias_calibration_;
        }

        auto output_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        publisher_ = create_publisher<PointCloud2>(output_topic_, output_qos);
        imu_subscriber_ = create_subscription<Imu>(
            imu_topic_, rclcpp::SensorDataQoS(),
            std::bind(&ImuRotationDeskewNode::ImuCallback, this, std::placeholders::_1));
        cloud_subscriber_ = create_subscription<PointCloud2>(
            cloud_topic_, rclcpp::SensorDataQoS(),
            std::bind(&ImuRotationDeskewNode::CloudCallback, this, std::placeholders::_1));
        if (enable_translational_deskew_) {
            bias_subscriber_ = create_subscription<Imu>(
                bias_topic_, rclcpp::QoS(1).reliable().transient_local(),
                std::bind(&ImuRotationDeskewNode::BiasCallback, this,
                          std::placeholders::_1));
            velocity_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
                velocity_topic_, rclcpp::QoS(10),
                std::bind(&ImuRotationDeskewNode::VelocityCallback, this,
                          std::placeholders::_1));
        }
        tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf2_buffer_->setUsingDedicatedThread(true);
        tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

        RCLCPP_INFO(get_logger(), "IMU rotation deskew: cloud=%s imu=%s output=%s",
                    cloud_topic_.c_str(), imu_topic_.c_str(), output_topic_.c_str());
        if (enable_translational_deskew_) {
            RCLCPP_INFO(get_logger(),
                        "Using angular_velocity plus bias-corrected linear_acceleration; "
                        "IMU orientation supplies roll/pitch gravity direction only");
        } else {
            RCLCPP_INFO(get_logger(),
                        "Rotation-only compatibility path: IMU orientation and "
                        "linear_acceleration do not alter points");
        }
        RCLCPP_INFO(get_logger(),
                    "IMU angular velocity scale=%.12f, gyro bias calibration=%s",
                    imu_angular_velocity_scale_,
                    enable_gyro_bias_calibration_ && !gyro_bias_manual_override_
                        ? "enabled"
                        : "disabled");
        RCLCPP_INFO(get_logger(),
                    "Lidar lever-arm correction=%s, lidar_lever_arm=%s m",
                    enable_lidar_lever_arm_correction_ ? "enabled" : "disabled",
                    FormatVector(lidar_lever_arm_).c_str());
        RCLCPP_INFO(get_logger(),
                    "Translational deskew=%s bias_topic=%s velocity_topic=%s",
                    enable_translational_deskew_ ? "enabled" : "disabled",
                    bias_topic_.c_str(), velocity_topic_.c_str());
    }

private:
    void ImuCallback(const Imu::ConstSharedPtr msg) {
        const double stamp = rclcpp::Time(msg->header.stamp).seconds();
        const Eigen::Vector3d raw_angular_velocity(msg->angular_velocity.x,
                                                   msg->angular_velocity.y,
                                                   msg->angular_velocity.z);
        const Eigen::Vector3d scaled_angular_velocity =
            imu_angular_velocity_scale_ * raw_angular_velocity;
        const Eigen::Vector3d linear_acceleration(msg->linear_acceleration.x,
                                                  msg->linear_acceleration.y,
                                                  msg->linear_acceleration.z);
        if (!std::isfinite(stamp) ||
            !raw_angular_velocity.allFinite() ||
            !scaled_angular_velocity.allFinite() ||
            !linear_acceleration.allFinite()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Dropping IMU sample with non-finite time or angular velocity");
            return;
        }

        Eigen::Vector3d gravity_direction = Eigen::Vector3d::UnitZ();
        bool gravity_direction_valid = false;
        if (use_imu_orientation_for_gravity_ && msg->orientation_covariance[0] >= 0.0) {
            Eigen::Quaterniond orientation(msg->orientation.w, msg->orientation.x,
                                           msg->orientation.y, msg->orientation.z);
            if (orientation.coeffs().allFinite() && orientation.norm() > 1.0e-6) {
                orientation.normalize();
                gravity_direction = orientation.conjugate() * Eigen::Vector3d::UnitZ();
                gravity_direction_valid = gravity_direction.allFinite() &&
                                          gravity_direction.norm() > 1.0e-6;
                if (gravity_direction_valid) gravity_direction.normalize();
            }
        }

        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            if (imu_frame_id_.empty()) {
                imu_frame_id_ = msg->header.frame_id;
            } else if (imu_frame_id_ != msg->header.frame_id) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "Dropping IMU sample because frame changed from '%s' to '%s'",
                                     imu_frame_id_.c_str(), msg->header.frame_id.c_str());
                return;
            }
            if (std::isfinite(last_received_imu_stamp_) &&
                stamp <= last_received_imu_stamp_ + kTimeEpsilon) {
                ++total_non_monotonic_imu_samples_;
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "Non-monotonic/duplicate deskew IMU timestamp %.9f after %.9f; total=%zu",
                    stamp, last_received_imu_stamp_,
                    total_non_monotonic_imu_samples_.load());
            }
            last_received_imu_stamp_ = std::max(last_received_imu_stamp_, stamp);
            if (!gyro_bias_ready_) {
                UpdateGyroBiasCalibration(stamp, raw_angular_velocity, scaled_angular_velocity);
                if (!gyro_bias_ready_) {
                    return;
                }
            }

            const Eigen::Vector3d angular_velocity = scaled_angular_velocity - gyro_bias_;
            if (!angular_velocity.allFinite()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "Dropping IMU sample with non-finite bias-corrected angular velocity");
                return;
            }

            latest_imu_stamp_ = std::max(latest_imu_stamp_, stamp);
            if (stamp < latest_imu_stamp_ - imu_buffer_seconds_) {
                return;
            }

            const ImuSample sample{stamp, angular_velocity, linear_acceleration,
                                   gravity_direction, gravity_direction_valid};
            const auto it = std::lower_bound(
                imu_buffer_.begin(), imu_buffer_.end(), stamp,
                [](const ImuSample &candidate, const double value) {
                    return candidate.stamp < value;
                });

            if (it != imu_buffer_.end() && std::abs(it->stamp - stamp) <= kTimeEpsilon) {
                *it = sample;
            } else if (it != imu_buffer_.begin() &&
                       std::abs(std::prev(it)->stamp - stamp) <= kTimeEpsilon) {
                *std::prev(it) = sample;
            } else {
                imu_buffer_.insert(it, sample);
            }

            const double cutoff = latest_imu_stamp_ - imu_buffer_seconds_;
            while (!imu_buffer_.empty() && imu_buffer_.front().stamp < cutoff) {
                imu_buffer_.pop_front();
            }
        }

        ProcessPendingClouds();
    }

    void CloudCallback(const PointCloud2::ConstSharedPtr msg) {
        if (GyroBiasCalibrationInProgress()) {
            if (publish_raw_during_gyro_bias_calibration_) {
                PublishRawWithWarning(
                    msg, "Gyro bias calibration in progress; publishing raw cloud");
            } else {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "Gyro bias calibration in progress; not publishing deskewed cloud");
            }
            return;
        }

        if (!HasFloat32Field(*msg, "x") || !HasFloat32Field(*msg, "y") ||
            !HasFloat32Field(*msg, "z")) {
            PublishRawWithWarning(
                msg, "PointCloud2 x/y/z fields must exist and be FLOAT32; available fields: " +
                         AvailableFieldsString(*msg));
            return;
        }

        const auto *time_field = SelectPointTimeField(*msg);
        if (time_field == nullptr) {
            PublishRawWithWarning(
                msg, "PointCloud2 point time field not found; available fields: " +
                         AvailableFieldsString(*msg));
            return;
        }

        std::vector<double> point_time_offsets;
        try {
            point_time_offsets = ExtractFieldSeconds(*msg, *time_field, point_time_unit_);
        } catch (const std::exception &ex) {
            PublishRawWithWarning(msg, std::string("Failed to read point time field '") +
                                           time_field->name + "': " + ex.what());
            return;
        }

        const size_t n_points = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
        if (point_time_offsets.size() != n_points || !AllFinite(point_time_offsets)) {
            PublishRawWithWarning(msg, "Point time field has invalid size or non-finite values");
            return;
        }

        const auto timing = BuildScanTiming(*msg, point_time_offsets);
        if (!timing) {
            PublishRawWithWarning(msg, "Unable to determine scan timing from point offsets");
            return;
        }

        if (timing->duration <= kTimeEpsilon) {
            PublishRawWithWarning(msg, "Point time span is zero; publishing raw cloud");
            return;
        }

        const double imu_interval_start =
            std::min(timing->reference_time, timing->min_point_time);
        const double imu_interval_end =
            std::max(timing->reference_time, timing->max_point_time);

        std::string trajectory_error;
        const auto trajectory =
            BuildOrientationTrajectory(imu_interval_start, imu_interval_end, &trajectory_error);
        if (!trajectory) {
            if (enable_pending_cloud_queue_ && IsTemporaryImuCoverageError(trajectory_error)) {
                QueuePendingCloud(msg, *timing, time_field->name, n_points, trajectory_error);
            } else {
                PublishRawWithWarning(msg, trajectory_error);
            }
            return;
        }

        DeskewUsingAvailableMotion(msg, *timing, *trajectory);

        if (debug_print_) {
            const Sophus::SO3d r_ref = InterpolateOrientation(*trajectory, timing->reference_time);
            const Sophus::SO3d r_end = InterpolateOrientation(*trajectory, timing->max_point_time);
            const double reference_to_end_deg =
                (r_ref.inverse() * r_end).log().norm() * kRadiansToDegrees;
            RCLCPP_INFO(get_logger(),
                        "Deskewed %zu points using field '%s': scan=[%.6f %.6f] ref=%.6f "
                        "imu_samples=%zu ref_to_last=%.3f deg%s",
                        n_points, time_field->name.c_str(), timing->scan_start,
                        timing->scan_end, timing->reference_time, trajectory->stamps.size(),
                        reference_to_end_deg,
                        invert_correction_ ? " inverted" : "");
        }
    }

    void UpdateGyroBiasCalibration(const double stamp,
                                   const Eigen::Vector3d &raw_angular_velocity,
                                   const Eigen::Vector3d &scaled_angular_velocity) {
        if (std::isnan(gyro_bias_calibration_start_)) {
            gyro_bias_calibration_start_ = stamp;
            if (print_gyro_stats_) {
                RCLCPP_INFO(get_logger(),
                            "Starting gyro bias calibration: duration=%.3fs min_samples=%d",
                            gyro_bias_calibration_seconds_, gyro_bias_min_samples_);
            }
        }

        gyro_bias_raw_sum_ += raw_angular_velocity;
        gyro_bias_scaled_sum_ += scaled_angular_velocity;
        ++gyro_bias_sample_count_;

        const double elapsed = std::max(0.0, stamp - gyro_bias_calibration_start_);
        if (elapsed < gyro_bias_calibration_seconds_ ||
            gyro_bias_sample_count_ < static_cast<size_t>(gyro_bias_min_samples_)) {
            return;
        }

        const double inv_count = 1.0 / static_cast<double>(gyro_bias_sample_count_);
        const Eigen::Vector3d raw_mean = gyro_bias_raw_sum_ * inv_count;
        const Eigen::Vector3d scaled_mean = gyro_bias_scaled_sum_ * inv_count;

        gyro_bias_ = scaled_mean;
        gyro_bias_ready_ = true;
        if (print_gyro_stats_) {
            RCLCPP_INFO(get_logger(),
                        "Gyro bias calibration complete: samples=%zu elapsed=%.3fs",
                        gyro_bias_sample_count_, elapsed);
            RCLCPP_INFO(get_logger(), "Raw omega mean=%s", FormatVector(raw_mean).c_str());
            RCLCPP_INFO(get_logger(), "Scaled omega mean=%s rad/s",
                        FormatVector(scaled_mean).c_str());
            RCLCPP_INFO(get_logger(), "Final gyro_bias=%s rad/s",
                        FormatVector(gyro_bias_).c_str());
        }
    }

    bool GyroBiasCalibrationInProgress() const {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        return enable_gyro_bias_calibration_ &&
               !gyro_bias_manual_override_ &&
               !gyro_bias_ready_;
    }

    double NowSeconds() const {
        return this->now().seconds();
    }

    static double WaitedMilliseconds(const PendingCloud &pending, const double now_seconds) {
        return 1000.0 * std::max(0.0, now_seconds - pending.queued_at);
    }

    void QueuePendingCloud(const PointCloud2::ConstSharedPtr &msg,
                           const ScanTiming &timing,
                           const std::string &time_field_name,
                           const size_t point_count,
                           const std::string &reason) {
        PendingCloud pending;
        pending.msg = msg;
        pending.timing = timing;
        pending.time_field_name = time_field_name;
        pending.point_count = point_count;
        pending.queued_at = NowSeconds();
        pending.sequence = next_pending_sequence_++;

        std::lock_guard<std::mutex> lock(pending_mutex_);
        const auto insert_at = std::lower_bound(
            pending_clouds_.begin(), pending_clouds_.end(), pending.timing.min_point_time,
            [](const PendingCloud &candidate, const double stamp) {
                return candidate.timing.min_point_time < stamp;
            });
        pending_clouds_.insert(insert_at, std::move(pending));
        ++total_queued_clouds_;

        if (debug_print_) {
            RCLCPP_INFO(get_logger(),
                        "Queued cloud waiting for IMU coverage: pending=%zu total_queued=%zu "
                        "scan=[%.6f %.6f] ref=%.6f field=%s reason=%s",
                        pending_clouds_.size(), total_queued_clouds_.load(),
                        timing.scan_start, timing.scan_end, timing.reference_time,
                        time_field_name.c_str(), reason.c_str());
        }

        while (pending_clouds_.size() > static_cast<size_t>(max_pending_clouds_)) {
            const auto dropped = pending_clouds_.front();
            pending_clouds_.pop_front();
            ++total_dropped_clouds_;
            RCLCPP_WARN(get_logger(),
                        "Dropped pending cloud after waiting %.1f ms: max_pending_clouds=%d "
                        "total_dropped=%zu",
                        WaitedMilliseconds(dropped, NowSeconds()), max_pending_clouds_,
                        total_dropped_clouds_.load());
        }
    }

    bool RemovePendingCloud(const size_t sequence) {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        const auto it = std::find_if(
            pending_clouds_.begin(), pending_clouds_.end(),
            [&](const PendingCloud &candidate) {
                return candidate.sequence == sequence;
            });
        if (it == pending_clouds_.end()) {
            return false;
        }
        pending_clouds_.erase(it);
        return true;
    }

    std::optional<PendingCloud> OldestPendingCloud() const {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        if (pending_clouds_.empty()) {
            return std::nullopt;
        }
        return pending_clouds_.front();
    }

    void ProcessPendingClouds() {
        if (!enable_pending_cloud_queue_) {
            return;
        }

        while (true) {
            const auto pending_opt = OldestPendingCloud();
            if (!pending_opt) {
                return;
            }

            const PendingCloud pending = *pending_opt;
            const double interval_start =
                std::min(pending.timing.reference_time, pending.timing.min_point_time);
            const double interval_end =
                std::max(pending.timing.reference_time, pending.timing.max_point_time);

            std::string trajectory_error;
            const auto trajectory =
                BuildOrientationTrajectory(interval_start, interval_end, &trajectory_error);
            if (trajectory) {
                DeskewUsingAvailableMotion(pending.msg, pending.timing, *trajectory);
                RemovePendingCloud(pending.sequence);
                if (debug_print_) {
                    RCLCPP_INFO(get_logger(),
                                "Processed pending cloud after %.1f ms: total_deskewed=%zu",
                                WaitedMilliseconds(pending, NowSeconds()),
                                total_deskewed_clouds_.load());
                }
                continue;
            }

            const double now_seconds = NowSeconds();
            const double waited_seconds = std::max(0.0, now_seconds - pending.queued_at);
            if (waited_seconds < max_pending_cloud_wait_seconds_) {
                return;
            }

            if (publish_raw_on_missing_imu_after_wait_) {
                RCLCPP_WARN(get_logger(),
                            "Published raw pending cloud after waiting %.1f ms: %s",
                            WaitedMilliseconds(pending, now_seconds),
                            trajectory_error.c_str());
                PublishRawCloud(pending.msg);
                RemovePendingCloud(pending.sequence);
                continue;
            }

            if (drop_cloud_on_missing_imu_after_wait_) {
                ++total_dropped_clouds_;
                RCLCPP_WARN(get_logger(),
                            "Dropped pending cloud after waiting %.1f ms: %s total_dropped=%zu",
                            WaitedMilliseconds(pending, now_seconds),
                            trajectory_error.c_str(),
                            total_dropped_clouds_.load());
                RemovePendingCloud(pending.sequence);
                continue;
            }

            return;
        }
    }

    const PointField *SelectPointTimeField(const PointCloud2 &cloud) const {
        std::vector<std::string> candidates;
        const auto requested = ToLower(Trim(point_time_field_param_));
        if (requested.empty() || requested == "auto") {
            candidates = {"time", "timestamp", "t", "offset_time", "point_time_offset",
                          "time_offset"};
        } else {
            candidates = SplitCandidates(point_time_field_param_);
            if (candidates.empty()) candidates.push_back(point_time_field_param_);
        }

        for (const auto &candidate : candidates) {
            if (const auto *field = FindField(cloud, candidate)) {
                return field;
            }
        }
        return nullptr;
    }

    std::optional<ScanTiming> BuildScanTiming(const PointCloud2 &cloud,
                                              const std::vector<double> &offsets_seconds) const {
        if (offsets_seconds.empty()) return std::nullopt;

        const auto [min_it, max_it] =
            std::minmax_element(offsets_seconds.begin(), offsets_seconds.end());
        const double min_offset = *min_it;
        const double max_offset = *max_it;
        const double header_time = rclcpp::Time(cloud.header.stamp).seconds();

        ScanTiming timing;
        timing.point_times.resize(offsets_seconds.size());
        timing.absolute_point_times = min_offset > kAbsoluteUnixTimeThreshold;

        if (timing.absolute_point_times) {
            timing.point_times = offsets_seconds;
            timing.min_point_time = min_offset;
            timing.max_point_time = max_offset;
            timing.scan_start = min_offset;
            timing.scan_end = max_offset;
            timing.duration = timing.scan_end - timing.scan_start;
            timing.reference_time =
                timing.scan_start + LocationOffset(deskew_reference_, timing.duration);
            return timing;
        }

        timing.offsets_relative_to_header = min_offset < -kTimeEpsilon;

        if (timing.offsets_relative_to_header) {
            for (size_t i = 0; i < offsets_seconds.size(); ++i) {
                timing.point_times[i] = header_time + offsets_seconds[i];
            }
            const double observed_start = header_time + min_offset;
            const double observed_end = header_time + max_offset;
            timing.duration = observed_end - observed_start;
            timing.scan_start =
                header_time - LocationOffset(cloud_stamp_location_, timing.duration);
        } else {
            // Most LiDAR drivers store non-negative per-point offsets from scan start.
            // The header stamp location tells us how to recover that scan start time.
            timing.duration = max_offset;
            timing.scan_start =
                header_time - LocationOffset(cloud_stamp_location_, timing.duration);
            for (size_t i = 0; i < offsets_seconds.size(); ++i) {
                timing.point_times[i] = timing.scan_start + offsets_seconds[i];
            }
        }

        timing.scan_end = timing.scan_start + timing.duration;
        timing.reference_time =
            timing.scan_start + LocationOffset(deskew_reference_, timing.duration);

        const auto [min_time_it, max_time_it] =
            std::minmax_element(timing.point_times.begin(), timing.point_times.end());
        timing.min_point_time = *min_time_it;
        timing.max_point_time = *max_time_it;

        if (!(timing.duration >= 0.0)) return std::nullopt;
        return timing;
    }

    void BiasCallback(const Imu::ConstSharedPtr msg) {
        const Eigen::Vector3d accel_bias(msg->linear_acceleration.x,
                                         msg->linear_acceleration.y,
                                         msg->linear_acceleration.z);
        if (!accel_bias.allFinite()) {
            RCLCPP_WARN(get_logger(), "Ignoring non-finite translational deskew accel bias");
            return;
        }
        std::lock_guard<std::mutex> lock(translation_mutex_);
        translation_accel_bias_ = accel_bias;
        translation_bias_frame_id_ = msg->header.frame_id;
        translation_bias_ready_ = true;
        RCLCPP_INFO(get_logger(), "Translational deskew received accel_bias=%s m/s^2 frame=%s",
                    FormatVector(translation_accel_bias_).c_str(),
                    translation_bias_frame_id_.c_str());
    }

    void VelocityCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const Eigen::Vector3d velocity(msg->twist.twist.linear.x,
                                       msg->twist.twist.linear.y,
                                       msg->twist.twist.linear.z);
        const double stamp = rclcpp::Time(msg->header.stamp).seconds();
        if (!velocity.allFinite() || !std::isfinite(stamp)) return;
        std::lock_guard<std::mutex> lock(translation_mutex_);
        translation_velocity_ = velocity;
        translation_velocity_stamp_ = stamp;
        translation_velocity_frame_id_ = msg->child_frame_id;
        translation_velocity_ready_ = true;
    }

    std::optional<Sophus::SO3d> LookupRotation(const std::string &target_frame,
                                                const std::string &source_frame,
                                                std::string *error) const {
        if (source_frame.empty() || target_frame.empty() || source_frame == target_frame) {
            return Sophus::SO3d();
        }
        std::string tf_error;
        if (!tf2_buffer_->_frameExists(source_frame) ||
            !tf2_buffer_->_frameExists(target_frame) ||
            !tf2_buffer_->canTransform(target_frame, source_frame,
                                       tf2::TimePointZero, &tf_error)) {
            if (error) {
                *error = "missing rotation " + target_frame + " <- " + source_frame +
                         ": " + tf_error;
            }
            return std::nullopt;
        }
        try {
            const auto transform = tf2_buffer_->lookupTransform(
                target_frame, source_frame, tf2::TimePointZero);
            Eigen::Quaterniond quaternion(transform.transform.rotation.w,
                                          transform.transform.rotation.x,
                                          transform.transform.rotation.y,
                                          transform.transform.rotation.z);
            if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1.0e-6) {
                if (error) *error = "non-finite or zero transform quaternion";
                return std::nullopt;
            }
            quaternion.normalize();
            return Sophus::SO3d(quaternion);
        } catch (const tf2::TransformException &exception) {
            if (error) *error = exception.what();
            return std::nullopt;
        }
    }

    std::optional<genz_icp::imu::Trajectory> BuildMotionTrajectory(
        const std::string &cloud_frame_id,
        double interval_start,
        double interval_end,
        double reference_time,
        std::string *error) const {
        std::vector<ImuSample> samples;
        std::string imu_frame_id;
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            samples.assign(imu_buffer_.begin(), imu_buffer_.end());
            imu_frame_id = imu_frame_id_;
        }
        Eigen::Vector3d accel_bias;
        Eigen::Vector3d velocity;
        double velocity_stamp = 0.0;
        std::string bias_frame;
        std::string velocity_frame;
        bool velocity_ready = false;
        {
            std::lock_guard<std::mutex> lock(translation_mutex_);
            if (!translation_bias_ready_) {
                if (error) *error = "waiting for calibrated accelerometer bias";
                return std::nullopt;
            }
            accel_bias = translation_accel_bias_;
            bias_frame = translation_bias_frame_id_;
            velocity = translation_velocity_;
            velocity_stamp = translation_velocity_stamp_;
            velocity_frame = translation_velocity_frame_id_;
            velocity_ready = translation_velocity_ready_;
        }

        std::string tf_error;
        const auto cloud_from_imu = LookupRotation(cloud_frame_id, imu_frame_id, &tf_error);
        if (!cloud_from_imu) {
            if (error) *error = tf_error;
            return std::nullopt;
        }
        const auto cloud_from_bias = LookupRotation(cloud_frame_id, bias_frame, &tf_error);
        if (!cloud_from_bias) {
            if (error) *error = tf_error;
            return std::nullopt;
        }

        std::vector<genz_icp::imu::Sample> converted;
        converted.reserve(samples.size());
        for (const auto &sample : samples) {
            genz_icp::imu::Sample entry;
            entry.stamp = sample.stamp;
            entry.angular_velocity = *cloud_from_imu * sample.angular_velocity;
            entry.linear_acceleration = *cloud_from_imu * sample.linear_acceleration;
            entry.gravity_direction = *cloud_from_imu * sample.gravity_direction;
            entry.gravity_direction_valid = sample.gravity_direction_valid;
            converted.push_back(entry);
        }

        genz_icp::imu::State initial_state;
        const auto first_after_start = std::upper_bound(
            converted.begin(), converted.end(), interval_start,
            [](double stamp, const genz_icp::imu::Sample &sample) {
                return stamp < sample.stamp;
            });
        if (first_after_start != converted.begin()) {
            const auto &sample = *std::prev(first_after_start);
            if (sample.gravity_direction_valid) {
                initial_state.orientation = genz_icp::imu::AlignGravity(
                    Sophus::SO3d(), sample.gravity_direction, gravity_correction_gain_);
            }
        }

        if (velocity_ready &&
            std::abs(reference_time - velocity_stamp) <=
                translation_max_velocity_age_seconds_) {
            const auto cloud_from_velocity =
                LookupRotation(cloud_frame_id, velocity_frame, &tf_error);
            if (cloud_from_velocity) {
                const Eigen::Vector3d velocity_cloud = *cloud_from_velocity * velocity;
                initial_state.velocity = initial_state.orientation * velocity_cloud;
            }
        }

        genz_icp::imu::IntegrationConfig config;
        config.max_gap_seconds = max_imu_gap_seconds_;
        config.max_acceleration = translation_max_acceleration_;
        config.max_velocity = translation_max_velocity_;
        config.gravity_magnitude = gravity_magnitude_;
        config.gravity_correction_gain = gravity_correction_gain_;
        config.use_gravity_constraint = true;
        config.integrate_translation = true;
        return genz_icp::imu::Integrate(
            converted, interval_start, interval_end, initial_state,
            Eigen::Vector3d::Zero(), *cloud_from_bias * accel_bias, config, error);
    }

    std::optional<OrientationTrajectory> BuildOrientationTrajectory(const double interval_start,
                                                                    const double interval_end,
                                                                    std::string *error) const {
        std::vector<ImuSample> samples;
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            samples.assign(imu_buffer_.begin(), imu_buffer_.end());
        }

        if (samples.size() < 2) {
            if (error) *error = "Not enough IMU samples buffered";
            return std::nullopt;
        }

        const auto first_after_start = std::upper_bound(
            samples.begin(), samples.end(), interval_start,
            [](const double value, const ImuSample &sample) {
                return value < sample.stamp;
            });
        if (first_after_start == samples.begin()) {
            if (error) {
                *error = "IMU buffer does not cover scan start/reference time; first IMU=" +
                         std::to_string(samples.front().stamp) +
                         " requested=" + std::to_string(interval_start);
            }
            return std::nullopt;
        }
        const size_t first_index =
            static_cast<size_t>(std::distance(samples.begin(), first_after_start) - 1);

        const auto first_at_or_after_end = std::lower_bound(
            samples.begin(), samples.end(), interval_end,
            [](const ImuSample &sample, const double value) {
                return sample.stamp < value;
            });
        if (first_at_or_after_end == samples.end()) {
            if (error) {
                *error = "IMU buffer does not cover scan end/reference time; last IMU=" +
                         std::to_string(samples.back().stamp) +
                         " requested=" + std::to_string(interval_end);
            }
            return std::nullopt;
        }
        const size_t last_index =
            static_cast<size_t>(std::distance(samples.begin(), first_at_or_after_end));

        OrientationTrajectory trajectory;
        trajectory.stamps.reserve(last_index - first_index + 1);
        trajectory.angular_velocities.reserve(last_index - first_index + 1);
        trajectory.rotations.reserve(last_index - first_index + 1);

        for (size_t i = first_index; i <= last_index; ++i) {
            trajectory.stamps.push_back(samples[i].stamp);
            trajectory.angular_velocities.push_back(samples[i].angular_velocity);
        }

        trajectory.rotations.emplace_back();
        for (size_t i = 0; i + 1 < trajectory.stamps.size(); ++i) {
            const double dt = trajectory.stamps[i + 1] - trajectory.stamps[i];
            if (!(dt > 0.0)) {
                if (error) *error = "IMU samples are not strictly time ordered";
                return std::nullopt;
            }

            const bool overlaps_requested_interval =
                trajectory.stamps[i] < interval_end && trajectory.stamps[i + 1] > interval_start;
            if (max_imu_gap_seconds_ > 0.0 && overlaps_requested_interval &&
                dt > max_imu_gap_seconds_) {
                if (error) {
                    *error = "IMU gap " + std::to_string(dt) +
                             "s exceeds max_imu_gap_seconds=" +
                             std::to_string(max_imu_gap_seconds_);
                }
                return std::nullopt;
            }

            const Eigen::Vector3d omega_mid =
                0.5 * (trajectory.angular_velocities[i] +
                       trajectory.angular_velocities[i + 1]);
            trajectory.rotations.push_back(trajectory.rotations.back() *
                                           Sophus::SO3d::exp(omega_mid * dt));
        }

        return trajectory;
    }

    Sophus::SO3d InterpolateOrientation(const OrientationTrajectory &trajectory,
                                        const double stamp) const {
        if (stamp <= trajectory.stamps.front()) return trajectory.rotations.front();
        if (stamp >= trajectory.stamps.back()) return trajectory.rotations.back();

        const auto upper =
            std::upper_bound(trajectory.stamps.begin(), trajectory.stamps.end(), stamp);
        const size_t index =
            static_cast<size_t>(std::distance(trajectory.stamps.begin(), upper) - 1);

        const double t0 = trajectory.stamps[index];
        const double t1 = trajectory.stamps[index + 1];
        const double dt = t1 - t0;
        if (!(dt > 0.0)) return trajectory.rotations[index];

        const double partial_dt = stamp - t0;
        const double alpha = partial_dt / dt;
        const Eigen::Vector3d omega_at_stamp =
            trajectory.angular_velocities[index] +
            alpha * (trajectory.angular_velocities[index + 1] -
                     trajectory.angular_velocities[index]);

        // Body-frame gyro integration uses right composition:
        // R(t + dt) = R(t) * Exp(omega_body * dt).
        const Eigen::Vector3d omega_mid =
            0.5 * (trajectory.angular_velocities[index] + omega_at_stamp);
        return trajectory.rotations[index] * Sophus::SO3d::exp(omega_mid * partial_dt);
    }

    void DeskewUsingAvailableMotion(const PointCloud2::ConstSharedPtr &msg,
                                     const ScanTiming &timing,
                                     const OrientationTrajectory &rotation_trajectory) {
        if (!enable_translational_deskew_ || invert_correction_) {
            if (enable_translational_deskew_ && invert_correction_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "Translational deskew does not support invert_correction; using rotation only");
            }
            DeskewAndPublish(msg, timing, rotation_trajectory);
            return;
        }
        const double interval_start =
            std::min(timing.reference_time, timing.min_point_time);
        const double interval_end =
            std::max(timing.reference_time, timing.max_point_time);
        std::string error;
        const auto motion_trajectory = BuildMotionTrajectory(
            msg->header.frame_id, interval_start, interval_end,
            timing.reference_time, &error);
        if (!motion_trajectory) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Translational deskew unavailable (%s); preserving rotation-only deskew",
                error.c_str());
            DeskewAndPublish(msg, timing, rotation_trajectory);
            return;
        }
        DeskewAndPublishTranslation(msg, timing, *motion_trajectory);
    }

    void DeskewAndPublishTranslation(
        const PointCloud2::ConstSharedPtr &msg,
        const ScanTiming &timing,
        const genz_icp::imu::Trajectory &trajectory) {
        auto corrected = std::make_unique<PointCloud2>(*msg);
        const auto reference_state =
            genz_icp::imu::InterpolateState(trajectory, timing.reference_time);
        sensor_msgs::PointCloud2ConstIterator<float> in_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> in_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> in_z(*msg, "z");
        sensor_msgs::PointCloud2Iterator<float> out_x(*corrected, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(*corrected, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(*corrected, "z");

        for (size_t i = 0; i < timing.point_times.size();
             ++i, ++in_x, ++in_y, ++in_z, ++out_x, ++out_y, ++out_z) {
            Eigen::Vector3d raw_point(*in_x, *in_y, *in_z);
            if (!raw_point.allFinite()) continue;
            if (enable_lidar_lever_arm_correction_) raw_point += lidar_lever_arm_;
            const auto point_state =
                genz_icp::imu::InterpolateState(trajectory, timing.point_times[i]);
            Eigen::Vector3d corrected_point = genz_icp::imu::DeskewPoint(
                raw_point, point_state, reference_state, true);
            if (enable_lidar_lever_arm_correction_) corrected_point -= lidar_lever_arm_;
            *out_x = static_cast<float>(corrected_point.x());
            *out_y = static_cast<float>(corrected_point.y());
            *out_z = static_cast<float>(corrected_point.z());
        }
        publisher_->publish(std::move(corrected));
        ++total_deskewed_clouds_;
        if (debug_print_) {
            const Eigen::Vector3d scan_translation =
                trajectory.states.back().position - trajectory.states.front().position;
            RCLCPP_INFO(get_logger(),
                        "Translationally deskewed %zu points; integrated scan translation=%s m",
                        timing.point_times.size(), FormatVector(scan_translation).c_str());
        }
    }

    void DeskewAndPublish(const PointCloud2::ConstSharedPtr &msg,
                          const ScanTiming &timing,
                          const OrientationTrajectory &trajectory) {
        auto corrected = std::make_unique<PointCloud2>(*msg);

        const Sophus::SO3d reference_orientation =
            InterpolateOrientation(trajectory, timing.reference_time);

        sensor_msgs::PointCloud2ConstIterator<float> in_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> in_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> in_z(*msg, "z");
        sensor_msgs::PointCloud2Iterator<float> out_x(*corrected, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(*corrected, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(*corrected, "z");

        for (size_t i = 0; i < timing.point_times.size();
             ++i, ++in_x, ++in_y, ++in_z, ++out_x, ++out_y, ++out_z) {
            const Eigen::Vector3d raw_point(*in_x, *in_y, *in_z);
            if (!raw_point.allFinite()) {
                continue;
            }

            const Sophus::SO3d point_orientation =
                InterpolateOrientation(trajectory, timing.point_times[i]);
            Sophus::SO3d point_to_reference =
                reference_orientation.inverse() * point_orientation;
            if (invert_correction_) {
                point_to_reference = point_to_reference.inverse();
            }

            Eigen::Vector3d corrected_point;
            if (enable_lidar_lever_arm_correction_) {
                corrected_point =
                    point_to_reference * (raw_point + lidar_lever_arm_) - lidar_lever_arm_;
            } else {
                corrected_point = point_to_reference * raw_point;
            }
            *out_x = static_cast<float>(corrected_point.x());
            *out_y = static_cast<float>(corrected_point.y());
            *out_z = static_cast<float>(corrected_point.z());
        }

        publisher_->publish(std::move(corrected));
        ++total_deskewed_clouds_;
    }

    void PublishRawCloud(const PointCloud2::ConstSharedPtr &msg) {
        auto raw = std::make_unique<PointCloud2>(*msg);
        publisher_->publish(std::move(raw));
        ++total_raw_fallback_clouds_;
    }

    void PublishRawWithWarning(const PointCloud2::ConstSharedPtr &msg, const std::string &reason) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", reason.c_str());
        PublishRawCloud(msg);
    }

    std::string cloud_topic_;
    std::string imu_topic_;
    std::string output_topic_;
    std::string point_time_field_param_;
    TimeUnit point_time_unit_{TimeUnit::Seconds};
    ScanLocation cloud_stamp_location_{ScanLocation::Start};
    ScanLocation deskew_reference_{ScanLocation::Middle};
    double imu_buffer_seconds_{2.0};
    double max_imu_gap_seconds_{0.03};
    bool debug_print_{false};
    bool invert_correction_{false};
    bool enable_lidar_lever_arm_correction_{false};
    Eigen::Vector3d lidar_lever_arm_{Eigen::Vector3d::Zero()};
    double imu_angular_velocity_scale_{1.0};
    bool enable_gyro_bias_calibration_{false};
    double gyro_bias_calibration_seconds_{2.0};
    int gyro_bias_min_samples_{50};
    bool print_gyro_stats_{true};
    bool publish_raw_during_gyro_bias_calibration_{true};
    bool enable_pending_cloud_queue_{true};
    double max_pending_cloud_wait_seconds_{0.15};
    bool publish_raw_on_missing_imu_after_wait_{false};
    bool drop_cloud_on_missing_imu_after_wait_{true};
    int max_pending_clouds_{20};
    bool enable_translational_deskew_{false};
    std::string bias_topic_{"/genz/imu/bias"};
    std::string velocity_topic_{"/genz/odometry"};
    double translation_max_velocity_age_seconds_{0.5};
    double translation_max_acceleration_{5.0};
    double translation_max_velocity_{5.0};
    double gravity_magnitude_{genz_icp::imu::kStandardGravity};
    double gravity_correction_gain_{1.0};
    bool use_imu_orientation_for_gravity_{true};

    mutable std::mutex imu_mutex_;
    std::deque<ImuSample> imu_buffer_;
    double latest_imu_stamp_{-std::numeric_limits<double>::infinity()};
    double last_received_imu_stamp_{-std::numeric_limits<double>::infinity()};
    Eigen::Vector3d gyro_bias_{Eigen::Vector3d::Zero()};
    bool gyro_bias_ready_{true};
    bool gyro_bias_manual_override_{false};
    double gyro_bias_calibration_start_{std::numeric_limits<double>::quiet_NaN()};
    size_t gyro_bias_sample_count_{0};
    Eigen::Vector3d gyro_bias_raw_sum_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gyro_bias_scaled_sum_{Eigen::Vector3d::Zero()};
    std::string imu_frame_id_;

    mutable std::mutex translation_mutex_;
    Eigen::Vector3d translation_accel_bias_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d translation_velocity_{Eigen::Vector3d::Zero()};
    double translation_velocity_stamp_{0.0};
    std::string translation_bias_frame_id_;
    std::string translation_velocity_frame_id_;
    bool translation_bias_ready_{false};
    bool translation_velocity_ready_{false};

    mutable std::mutex pending_mutex_;
    std::deque<PendingCloud> pending_clouds_;
    std::atomic<size_t> next_pending_sequence_{0};
    std::atomic<size_t> total_deskewed_clouds_{0};
    std::atomic<size_t> total_queued_clouds_{0};
    std::atomic<size_t> total_dropped_clouds_{0};
    std::atomic<size_t> total_raw_fallback_clouds_{0};
    std::atomic<size_t> total_non_monotonic_imu_samples_{0};

    rclcpp::Subscription<Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<Imu>::SharedPtr bias_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr velocity_subscriber_;
    rclcpp::Subscription<PointCloud2>::SharedPtr cloud_subscriber_;
    rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
};

}  // namespace genz_icp_ros

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<genz_icp_ros::ImuRotationDeskewNode>());
    rclcpp::shutdown();
    return 0;
}
