/**
 * @file
 * @copyright Copyright (C) 2020, Blickfeld GmbH
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * Header for class BlickfeldDriver
 */

#pragma once

#include <atomic>
#include <deque>
#include <thread>

#include <tf2_ros/static_transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <std_srvs/srv/empty.hpp>

/// service header, build-generated from IDL/srv
#include <blickfeld_driver/srv/imu_static_transformation.hpp>
#include <blickfeld_driver/srv/set_scan_pattern.hpp>

#include "blickfeld_driver_core/blickfeld_driver_core.h"

namespace blickfeld {
namespace ros_interop {

/// centralized place to keep the parameter string literals
namespace launch {
/// setup parameters
const std::string host_ = "host";
const std::string lidar_frame_id_ = "lidar_frame_id";

/// point cloud data publishing options
const std::string publish_ambient_light_ = "publish_ambient_light";
const std::string publish_explicit_range_ = "publish_explicit_range";
const std::string publish_intensities_ = "publish_intensities";
const std::string publish_no_return_points_ = "publish_no_return_points";
const std::string publish_point_id_ = "publish_point_id";
const std::string publish_point_time_offset_ = "publish_point_time_offset";
const std::string no_return_point_range_ = "no_return_point_range";
const std::string returns_publishing_options_ = "returns_publishing_options";

/// image publishing options
const std::string projection_type_ = "projection_type";
const std::string publish_ambient_image_ = "publish_ambient_image";
const std::string publish_intensity_image_ = "publish_intensity_image";
const std::string publish_range_image_ = "publish_range_image";
const std::string publish_point_id_image_ = "publish_point_id_image";

/// imu data publishing options
const std::string imu_acceleration_unit_ = "imu_acceleration_unit";
const std::string imu_stream_ = "publish_imu";
const std::string publish_imu_static_tf_at_start_ = "publish_imu_static_tf_at_start";

/// on device algorithms and their parameters
const std::string use_background_subtraction_ = "use_background_subtraction";
const std::string use_neighbor_filter_ = "use_neighbor_filter";
const std::string background_subtraction_exponential_decay_rate_ = "background_subtraction_exponential_decay_rate";
const std::string background_subtraction_num_initialization_frames_ =
    "background_subtraction_num_initialization_frames";

/// common publishing options
const std::string use_lidar_timestamp_ = "use_lidar_timestamp";

const std::string logging_throttled_interval_ = "logging_throttled_interval";
}  // namespace launch

/**
 * @class blickfeld::ros_interop::BlickfeldDriver
 * @brief Main class for the node to handle the ROS interfacing
 */
class BlickfeldDriver : public rclcpp::Node, public BlickfeldDriverCore {
 public:
  /**
   * Constructor, declares parameters constructively
   * @param[in] options contains parameter environment
   */
  explicit BlickfeldDriver(const rclcpp::NodeOptions& options);

  /**
   * @brief Desctructor
   *
   */
  ~BlickfeldDriver() override;

  /**
   * @brief Reads ROS params, sets up the publishers, and starts the core thread
   * @return true on success
   */
  bool init();

  /**
   * @brief publishPointCloud publishes a point cloud 2 message
   *
   * @param[in] point_cloud PointCloud2Ptr
   * @param[in] device_time device time
   */
  void publishPointCloud(PointCloud2Ptr& point_cloud, time_t device_time) override;

  /**
   * @brief publishStatus publishes a diagnostic status message
   *
   * @param[in] status DiagnosticStatusPtr
   */
  void publishStatus(DiagnosticStatusPtr& status) override;

  /**
   * @brief publishRangeImage publishes a sensor image message based on points range
   *
   * @param[in] range_image
   * @param[in] device_time device time
   */
  void publishRangeImage(SensorMsgImagePtr& range_image, time_t device_time) override;

  /**
   * @brief publishIntensityImage publishes a sensor image message based on points intensity
   *
   * @param[in] intensity_image
   * @param[in] device_time device time
   */
  void publishIntensityImage(SensorMsgImagePtr& intensity_image, time_t device_time) override;

  /**
   * @brief publishAmbientImage publishes a sensor image message based on points ambient
   *
   * @param[in] ambient_image
   * @param[in] device_time device time
   */
  void publishAmbientImage(SensorMsgImagePtr& ambient_image, time_t device_time) override;

  /**
   * @brief publishAmbientImage publishes a sensor image message which contains the point id's for every pixel
   *
   * @param[in] point_id_image
   * @param[in] device_time device time
   */
  void publishPointIdImage(SensorMsgImagePtr& point_id_image, time_t device_time) override;

  /**
   * @brief publishImu publishes a sensor imu message
   *
   * @param[in] imu
   * @param[in] device_time
   */
  void publishImu(SensorMsgImuPtr& imu, time_t device_time) override;

  /**
   * @brief publishImuStaticTF publishes imu acceleration as static tf transform
   *
   * @param[in] transform_msg_stamped
   */
  void publishImuStaticTF(const TransformMsgStamped& transform_msg_stamped) override;

  /**
   * @brief printLogMessages prints log messages that were collected during different stages of processing
   *
   * @param[in] log_messages std::unordered_map<LogLevel, std::ostringstream>;
   */
  void printLogMessages(const LogMessages& log_messages) override;

 private:
  /**
   * @brief createPointCloudOptions reads and verifies the ROS parameters and create point_cloud_options
   *
   * @return PointCloudOptions, point cloud options such as publishing options
   */
  PointCloudOptions createPointCloudOptions();

  /**
   * @brief createDeviceAlgorithmOptions reads and verifies the ROS parameters and create device_algorithm_options
   *
   * @return DeviceAlgorithmOptions, the options for on device algorithm
   */
  DeviceAlgorithmOptions createDeviceAlgorithmOptions();

  /**
   * @brief createImageOptions reads and verifies the ROS parameters and create image options
   *
   * @return ImageOptions created image options
   */
  ImageOptions createImageOptions();

  /**
   * @brief createImuOptions reads and verifies the ROS parameters and creates imu options
   *
   * @return ImuOptions created imu options
   */
  ImuOptions createImuOptions();

  /**
   * @brief setScanPatternCallback is a service callback for setting the scan pattern on the device
   * @param[in] request_header
   * @param[in] request
   * @param[out] response
   */
  void setScanPatternCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                              const std::shared_ptr<blickfeld_driver::srv::SetScanPattern::Request> request,
                              std::shared_ptr<blickfeld_driver::srv::SetScanPattern::Response> response);

  /**
   * @brief publishImuStaticTFCallback is a service callback for publishing static tf generated by IMU
   * @param[in] request_header
   * @param[in] request
   * @param[out] response
   */
  void publishImuStaticTFCallback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<blickfeld_driver::srv::ImuStaticTransformation::Request> service_request,
      std::shared_ptr<blickfeld_driver::srv::ImuStaticTransformation::Response> service_response);
  /**
   * @brief setMessageTime sets the time for the input ros_message
   *
   * @tparam T ros message
   * @param[in] device_time
   * @param[in,out] ros_message message
   */
  template <typename T>
  void setMessageTime(const time_t device_time, T& ros_message);

  std::shared_ptr<rclcpp::Publisher<SensorMsgImage>> point_id_image_publisher_;
  std::shared_ptr<rclcpp::Publisher<SensorMsgImage>> ambient_image_publisher_;
  std::shared_ptr<rclcpp::Publisher<DiagnosticStatus>> diagnostics_publisher_;
  std::shared_ptr<rclcpp::Publisher<SensorMsgImu>> imu_publisher_;
  rclcpp::Service<blickfeld_driver::srv::ImuStaticTransformation>::SharedPtr imu_static_tf_service_;
  std::shared_ptr<rclcpp::Publisher<SensorMsgImage>> intensity_image_publisher_;
  std::shared_ptr<rclcpp::Publisher<PointCloud2>> point_cloud_publisher_;
  std::shared_ptr<rclcpp::Publisher<SensorMsgImage>> range_image_publisher_;
  rclcpp::Service<blickfeld_driver::srv::SetScanPattern>::SharedPtr scan_pattern_service_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  bool use_lidar_timestamp_;
  /// logging throttling interval in milliseconds
  size_t logging_throttled_interval_;
};

}  // namespace ros_interop
}  // namespace blickfeld
