/**
 * @file
 * @copyright Copyright (C) 2020, Blickfeld GmbH
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * Implementation for class BlickfeldDriver
 */

#include <arpa/inet.h>
#include <cmath>
#include <numeric>
#include <string>
#include <utility>

#include <rclcpp/exceptions.hpp>

#include "blickfeld_driver/blickfeld_driver.h"
#include "blickfeld_driver_core/blickfeld_driver_types.h"
#include "blickfeld_driver_core/blickfeld_driver_utils.h"

namespace blickfeld {
namespace ros_interop {

BlickfeldDriver::BlickfeldDriver(const rclcpp::NodeOptions& options) : Node("bf_lidar", options) {
  /// setup parameters
  this->declare_parameter(launch::host_, std::string(""));

  /// point cloud data publishing options
  const PointCloudOptions point_cloud_options;
  this->declare_parameter(launch::lidar_frame_id_, point_cloud_options.lidar_frame_id);
  this->declare_parameter(launch::publish_ambient_light_, point_cloud_options.ambient_light);
  this->declare_parameter(launch::publish_explicit_range_, point_cloud_options.range);
  this->declare_parameter(launch::publish_intensities_, point_cloud_options.intensities);
  this->declare_parameter(launch::publish_no_return_points_, point_cloud_options.no_return_points);
  this->declare_parameter(launch::publish_point_id_, point_cloud_options.point_id);
  this->declare_parameter(launch::publish_point_time_offset_, point_cloud_options.point_time_offset);
  this->declare_parameter(launch::no_return_point_range_, point_cloud_options.no_return_point_range);

  try {
    this->declare_parameter(launch::returns_publishing_options_,
                            driver_utilities::returnOptionsToString(point_cloud_options.return_options));
  } catch (const std::invalid_argument& exe) {
    RCLCPP_FATAL(this->get_logger(), exe.what(),
                 ". Could not set default parameter value for 'return_publishing_options'");
  }

  /// image publishing options
  const ImageOptions image_options;
  try {
    this->declare_parameter(launch::projection_type_,
                            driver_utilities::projectionTypeToString(image_options.projection_type));
  } catch (const std::invalid_argument& exe) {
    RCLCPP_FATAL(this->get_logger(), exe.what(), ". Could not set default parameter value for 'projection_type'");
  }
  this->declare_parameter(launch::publish_ambient_image_, image_options.ambient_image);
  this->declare_parameter(launch::publish_intensity_image_, image_options.intensity_image);
  this->declare_parameter(launch::publish_range_image_, image_options.range_image);
  this->declare_parameter(launch::publish_point_id_image_, image_options.point_id_image);

  /// imu options
  const ImuOptions imu_options;
  try {
    this->declare_parameter(launch::imu_acceleration_unit_,
                            driver_utilities::imuAccelerationUnitToString(imu_options.imu_acceleration_unit));
  } catch (const std::invalid_argument& exe) {
    RCLCPP_FATAL(this->get_logger(), exe.what(), ". Could not set default parameter value for 'imu_acceleration_unit'");
  }
  this->declare_parameter(launch::imu_stream_, imu_options.imu_stream);
  this->declare_parameter(launch::publish_imu_static_tf_at_start_, imu_options.imu_static_tf);

  /// on device algorithms and parameters
  const DeviceAlgorithmOptions device_algorithm_options;
  this->declare_parameter(launch::use_background_subtraction_, device_algorithm_options.use_background_subtraction);
  this->declare_parameter(launch::use_neighbor_filter_, device_algorithm_options.use_neighbor_filter);
  this->declare_parameter(launch::background_subtraction_exponential_decay_rate_,
                          device_algorithm_options.background_subtraction_exponential_decay_rate);
  this->declare_parameter(launch::background_subtraction_num_initialization_frames_,
                          device_algorithm_options.background_subtraction_num_initialization_frames);

  /// common publishing options
  this->declare_parameter(launch::use_lidar_timestamp_, true);
  /// throttling interval in milliseconds
  this->declare_parameter(launch::logging_throttled_interval_, 10000);
}

BlickfeldDriver::~BlickfeldDriver() { BlickfeldDriverCore::stop(); }

bool BlickfeldDriver::init() {
  RCLCPP_INFO(this->get_logger(), "Init Blickfeld ROS driver ...");

  std::string host = get_parameter(launch::host_).as_string();
  if (host.compare("") == 0) {
    RCLCPP_FATAL(this->get_logger(), "Device host has not been set");
    return false;
  }
  use_lidar_timestamp_ = get_parameter(launch::use_lidar_timestamp_).as_bool();
  logging_throttled_interval_ = get_parameter(launch::logging_throttled_interval_).as_int();

  const PointCloudOptions point_cloud_options = createPointCloudOptions();
  const DeviceAlgorithmOptions device_algorithm_options = createDeviceAlgorithmOptions();
  const ImageOptions image_options = createImageOptions();
  const ImuOptions imu_options = createImuOptions();

  point_cloud_publisher_ = this->create_publisher<PointCloud2>("~/point_cloud_out", 4);
  diagnostics_publisher_ = this->create_publisher<DiagnosticStatus>("~/diagnostic_out", 4);

  if (image_options.ambient_image == true) {
    ambient_image_publisher_ = this->create_publisher<SensorMsgImage>("~/ambient_image_out", 1);
  }
  if (image_options.intensity_image == true) {
    intensity_image_publisher_ = this->create_publisher<SensorMsgImage>("~/intensity_image_out", 1);
  }
  if (image_options.range_image == true) {
    range_image_publisher_ = this->create_publisher<SensorMsgImage>("~/range_image_out", 1);
  }

  if (image_options.point_id_image == true) {
    point_id_image_publisher_ = this->create_publisher<SensorMsgImage>("~/point_id_image_out", 1);
  }

  if (imu_options.imu_stream == true) {
    imu_publisher_ = this->create_publisher<SensorMsgImu>("~/imu_out", 4);
  }
  /// setup parameter handling
  using namespace std::placeholders;
  scan_pattern_service_ = this->create_service<blickfeld_driver::srv::SetScanPattern>(
      "~/set_scan_pattern_service", std::bind(&BlickfeldDriver::setScanPatternCallback, this, _1, _2, _3));
  imu_static_tf_service_ = this->create_service<blickfeld_driver::srv::ImuStaticTransformation>(
      "~/publish_imu_static_tf_service", std::bind(&BlickfeldDriver::publishImuStaticTFCallback, this, _1, _2, _3));
  static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  BlickfeldDriverCore::init(host, point_cloud_options, device_algorithm_options, image_options, imu_options);
  BlickfeldDriverCore::start();
  return true;
}

void BlickfeldDriver::publishPointCloud(PointCloud2Ptr& point_cloud, time_t device_time) {
  setMessageTime(device_time, *point_cloud);
  point_cloud_publisher_->publish(std::move((point_cloud)));
}

void BlickfeldDriver::publishStatus(DiagnosticStatusPtr& status) { diagnostics_publisher_->publish(std::move(status)); }

void BlickfeldDriver::publishRangeImage(SensorMsgImagePtr& range_image, time_t device_time) {
  setMessageTime(device_time, *range_image);
  range_image_publisher_->publish(std::move(range_image));
}

void BlickfeldDriver::publishIntensityImage(SensorMsgImagePtr& intensity_image, time_t device_time) {
  setMessageTime(device_time, *intensity_image);
  intensity_image_publisher_->publish(std::move(intensity_image));
}

void BlickfeldDriver::publishAmbientImage(SensorMsgImagePtr& ambient_image, time_t device_time) {
  setMessageTime(device_time, *ambient_image);
  ambient_image_publisher_->publish(std::move(ambient_image));
}

void BlickfeldDriver::publishPointIdImage(SensorMsgImagePtr& point_id_image, time_t device_time) {
  setMessageTime(device_time, *point_id_image);
  point_id_image_publisher_->publish(std::move(point_id_image));
}

void BlickfeldDriver::publishImu(SensorMsgImuPtr& imu, time_t device_time) {
  setMessageTime(device_time, *imu);
  imu_publisher_->publish(std::move(imu));
}

void BlickfeldDriver::publishImuStaticTF(const TransformMsgStamped& transform_msg_stamped) {
  static_broadcaster_->sendTransform(transform_msg_stamped);
}

void BlickfeldDriver::printLogMessages(const LogMessages& log_messages) {
  for (const auto& log_message : log_messages) {
    switch (log_message.first) {
      case (LogLevel::Critical): {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), logging_throttled_interval_,
                              log_message.second.str().c_str());
        break;
      }
      case (LogLevel::Warning): {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), logging_throttled_interval_,
                             log_message.second.str().c_str());
        break;
      }
      case (LogLevel::Debug): {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), logging_throttled_interval_,
                              log_message.second.str().c_str());
        break;
      }
    }
  }
}

void BlickfeldDriver::setScanPatternCallback(
    __attribute__((unused)) const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<blickfeld_driver::srv::SetScanPattern::Request> service_request,
    std::shared_ptr<blickfeld_driver::srv::SetScanPattern::Response> service_response) {
  SetScanPatternResponse response = setScanPattern(reinterpret_cast<SetScanPatternRequest&>(*service_request));
  *service_response = reinterpret_cast<blickfeld_driver::srv::SetScanPattern::Response&>(response);
  return;
}

void BlickfeldDriver::publishImuStaticTFCallback(
    __attribute__((unused)) const std::shared_ptr<rmw_request_id_t> request_header,
    __attribute__((unused)) const std::shared_ptr<blickfeld_driver::srv::ImuStaticTransformation::Request>
        service_request,
    std::shared_ptr<blickfeld_driver::srv::ImuStaticTransformation::Response> service_response) {
  service_response->imu_transformation = *getStaticTF();
  publishImuStaticTF(service_response->imu_transformation);
  return;
}

PointCloudOptions BlickfeldDriver::createPointCloudOptions() {
  PointCloudOptions point_cloud_options;

  point_cloud_options.lidar_frame_id = get_parameter(launch::lidar_frame_id_).as_string();
  point_cloud_options.ambient_light = get_parameter(launch::publish_ambient_light_).as_bool();
  point_cloud_options.range = get_parameter(launch::publish_explicit_range_).as_bool();
  point_cloud_options.intensities = get_parameter(launch::publish_intensities_).as_bool();
  point_cloud_options.no_return_points = get_parameter(launch::publish_no_return_points_).as_bool();
  point_cloud_options.point_id = get_parameter(launch::publish_point_id_).as_bool();
  point_cloud_options.point_time_offset = get_parameter(launch::publish_point_time_offset_).as_bool();
  if (point_cloud_options.no_return_points == true)
    point_cloud_options.no_return_point_range = get_parameter(launch::no_return_point_range_).as_double();
  try {
    point_cloud_options.return_options =
        driver_utilities::returnOptionsFromString(get_parameter(launch::returns_publishing_options_).as_string());
  } catch (const std::invalid_argument& exe) {
    RCLCPP_WARN(this->get_logger(), "Could not parse parameter 'return_publishing_options_', using default: %s",
                driver_utilities::returnOptionsToString(point_cloud_options.return_options).c_str());
  }
  return point_cloud_options;
}

DeviceAlgorithmOptions BlickfeldDriver::createDeviceAlgorithmOptions() {
  DeviceAlgorithmOptions device_algorithm_options;

  device_algorithm_options.use_background_subtraction = get_parameter(launch::use_background_subtraction_).as_bool();
  device_algorithm_options.use_neighbor_filter = get_parameter(launch::use_neighbor_filter_).as_bool();
  device_algorithm_options.background_subtraction_exponential_decay_rate =
      static_cast<float>(get_parameter(launch::background_subtraction_exponential_decay_rate_).as_double());
  device_algorithm_options.background_subtraction_num_initialization_frames =
      get_parameter(launch::background_subtraction_num_initialization_frames_).as_int();

  return device_algorithm_options;
}

ImageOptions BlickfeldDriver::createImageOptions() {
  ImageOptions image_options;

  try {
    image_options.projection_type =
        driver_utilities::projectionTypeFromString(get_parameter(launch::projection_type_).as_string());
  } catch (const std::invalid_argument& exe) {
    RCLCPP_WARN(this->get_logger(), exe.what(), ". Could not parse parameter 'projection_type', using default: ",
                driver_utilities::projectionTypeToString(image_options.projection_type));
  }
  image_options.ambient_image = get_parameter(launch::publish_ambient_image_).as_bool();
  image_options.intensity_image = get_parameter(launch::publish_intensity_image_).as_bool();
  image_options.range_image = get_parameter(launch::publish_range_image_).as_bool();
  image_options.point_id_image = get_parameter(launch::publish_point_id_image_).as_bool();

  return image_options;
}

ImuOptions BlickfeldDriver::createImuOptions() {
  ImuOptions imu_options;

  imu_options.lidar_frame_id = get_parameter(launch::lidar_frame_id_).as_string();

  try {
    imu_options.imu_acceleration_unit =
        driver_utilities::imuAccelerationUnitFromString(get_parameter(launch::imu_acceleration_unit_).as_string());
  } catch (const std::invalid_argument& exe) {
    RCLCPP_WARN(this->get_logger(), "Could not parse parameter 'imu_acceleration_unit', using default: %s",
                driver_utilities::imuAccelerationUnitToString(imu_options.imu_acceleration_unit).c_str());
  }
  imu_options.imu_stream = get_parameter(launch::imu_stream_).as_bool();
  imu_options.imu_static_tf = get_parameter(launch::publish_imu_static_tf_at_start_).as_bool();

  return imu_options;
}

template <typename T>
void BlickfeldDriver::setMessageTime(time_t device_time, T& ros_msg) {
  if (use_lidar_timestamp_ == true) {
    ros_msg.header.stamp = rclcpp::Time(device_time);
  } else {
    ros_msg.header.stamp = this->now();
  }
}

}  // namespace ros_interop
}  // namespace blickfeld
