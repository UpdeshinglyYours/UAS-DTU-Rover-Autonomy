/**
 * @file
 * @copyright Copyright (C) 2020, Blickfeld GmbH
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * Header for class blickfeld driver core
 */

#pragma once

#include <atomic>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <blickfeld/scanner.h>

#include "blickfeld_driver/blickfeld_driver_ros_types.h"
#include "blickfeld_driver_core/blickfeld_driver_imu_parser.h"
#include "blickfeld_driver_core/blickfeld_driver_point_cloud_parser.h"
#include "blickfeld_driver_core/blickfeld_driver_types.h"

namespace blickfeld {
namespace ros_interop {

/**
 * @brief The is the Base class for a Blickfeld Driver Core. It converts the received data into ROS compatible messages.
 *
 */
class BlickfeldDriverCore {
 public:
  /**
   * @brief Construct a new Blickfeld Driver Core
   *
   */
  BlickfeldDriverCore() = default;

  /**
   * @brief Destroy the Blickfeld Driver Core
   *
   */
  virtual ~BlickfeldDriverCore();

  /**
   * @brief init initializes the variables
   *
   * @param[in] host host name or ip address of the sensor
   * @param[in] point_cloud_options  defines the fields to configure point cloud
   * @param[in] device_algorithm_options  defines the fields to configure on device algorithms
   * @param[in] image_options defines the fields to publish images from point cloud
   * @param[in] imu_options defines the fields to parse and publish imu data
   */
  void init(const std::string& host, const PointCloudOptions& point_cloud_options = PointCloudOptions(),
            const DeviceAlgorithmOptions& device_algorithm_options = DeviceAlgorithmOptions(),
            const ImageOptions& image_options = ImageOptions(), const ImuOptions& imu_options = ImuOptions());

  /**
   * @brief Start a new thread for grabbing frames from the sensor. Stop function should be called after start to
   * stop the thread gracefully (calling destructor is not enough).
   *
   */
  void start();

  /**
   * @brief stop the thread for grabbing data from the sensor
   *
   */
  void stop();

  /**
   * @brief publishPointCloud publishes a point cloud 2 message
   *
   * @param[in] point_cloud PointCloud2Ptr
   * @param[in] device_time device time
   */
  virtual void publishPointCloud(PointCloud2Ptr& point_cloud, time_t device_time) = 0;

  /**
   * @brief publishStatus publishes a diagnostic status message
   *
   * @param[in] status DiagnosticStatusPtr
   */
  virtual void publishStatus(DiagnosticStatusPtr& status) = 0;

  /**
   * @brief publishRangeImage publishes a sensor image message based on points range
   *
   * @param[in] range_image
   * @param[in] device_time device time
   */
  virtual void publishRangeImage(SensorMsgImagePtr& range_image, time_t device_time) = 0;

  /**
   * @brief publishIntensityImage publishes a sensor image message based on points intensity
   *
   * @param[in] intensity_image
   * @param[in] device_time device time
   */
  virtual void publishIntensityImage(SensorMsgImagePtr& intensity_image, time_t device_time) = 0;

  /**
   * @brief publishAmbientImage publishes a sensor image message based on points ambient
   *
   * @param[in] ambient_image
   * @param[in] device_time device time
   */
  virtual void publishAmbientImage(SensorMsgImagePtr& ambient_image, time_t device_time) = 0;

  /**
   * @brief publishAmbientImage publishes a sensor image message with the point id's for every pixel
   *
   * @param[in] point_id_image
   * @param[in] device_time device time
   */
  virtual void publishPointIdImage(SensorMsgImagePtr& point_id_image, time_t device_time) = 0;

  /**
   * @brief publishImu publishes a sensor imu message
   *
   * @param[in] imu
   * @param[in] device_time
   */
  virtual void publishImu(SensorMsgImuPtr& imu, time_t device_time) = 0;

  /**
   * @brief publishImuStaticTF converts imu acceleration into transform message stamped and published it as static tf
   * transform
   *
   * @param[in] transform_msg_stamped
   */
  virtual void publishImuStaticTF(const TransformMsgStamped& transform_msg_stamped) = 0;

  /**
   * @brief printLogMessages prints log messages that were collected during different stages of processing
   *
   * @param[in] log_messages std::unordered_map<LogLevel, std::ostringstream>;
   */
  virtual void printLogMessages(const LogMessages& log_messages) = 0;

  /**
   * @brief isRunning
   *
   * @return true if the driver is still running
   */
  bool isRunning() const;

 protected:
  /**
   * @brief setScanPattern sets the scan pattern of lidar
   *
   * @param[in] service_request
   * @return service_response
   */
  SetScanPatternResponse setScanPattern(const SetScanPatternRequest& service_request);

  /**
   * @brief getStaticTF creates a connection to the scanner to get the imu acceleration then convert it to transform
   * message stamped and return it
   *
   */
  TransformMsgStampedPtr getStaticTF();

 private:
  /**
   * @brief setupOnDeviceAlgorithms configures point cloud stream for on-device algorithms
   *
   * @param[in] device_algorithm_options on device algorithm options and parameters
   */
  void setupOnDeviceAlgorithms(const DeviceAlgorithmOptions& device_algorithm_options);

  /**
   * @brief pointCloudSpinner collects point cloud frames from a Blickfeld sensor and process them
   *
   */
  void pointCloudSpinner();

  /**
   * @brief createPointCloudStream connects to a Blickfeld sensor to start point cloud streaming
   *
   */
  void createPointCloudStream();

  /**
   * @brief imuSpinner collects imu samples from a Blickfeld sensor and process them
   *
   */
  void imuSpinner();

  /**
   * @brief createImuStream connects to a Blickfeld sensor to start imu streaming
   *
   */
  void createImuStream();

  /**
   * @brief addDebugMessage adds dropped frame and frame information into the debug messages
   *
   * @param[in] frame Currently received frame
   * @param[in] debugging_times
   */
  void addDebugMessage(const blickfeld::protocol::data::Frame& frame, const DebuggingTimes& debugging_times);

  /**
   * @brief isPointCloudStreaming checks device connection and point cloud streaming availability
   *
   * @return true if driver is connected to device and point cloud streaming is available
   */
  bool isPointCloudStreaming();

  /**
   * @brief isImuStreaming checks device connection and imu streaming availability
   *
   * @return true if driver is connected to device and imu streaming is available
   */
  bool isImuStreaming();

  std::thread imu_grabber_thread_;
  std::shared_ptr<blickfeld::scanner> imu_scanner_ = nullptr;
  std::shared_ptr<blickfeld::imu_stream> imu_stream_ = nullptr;
  std::thread point_cloud_grabber_thread_;
  std::shared_ptr<blickfeld::scanner> point_cloud_scanner_ = nullptr;
  std::mutex point_cloud_scanner_mutex_;
  std::shared_ptr<scanner::point_cloud_stream<blickfeld::protocol::data::Frame>> point_cloud_stream_ = nullptr;
  blickfeld::protocol::stream::Subscribe_PointCloud subscription_;

  std::string host_;
  std::atomic<bool> is_running_{false};

  BlickfeldDriverImuParser imu_parser_;
  BlickfeldDriverPointCloudParser point_cloud_parser_;

  std::chrono::high_resolution_clock::time_point last_frame_receive_time_;
  std::chrono::high_resolution_clock::time_point last_frame_start_time_ =
      std::chrono::high_resolution_clock::time_point::max();
  std::deque<float> last_processing_durations_;
  LogMessages log_messages_;

  ImuOptions imu_options_;
};

}  // namespace ros_interop
}  // namespace blickfeld
