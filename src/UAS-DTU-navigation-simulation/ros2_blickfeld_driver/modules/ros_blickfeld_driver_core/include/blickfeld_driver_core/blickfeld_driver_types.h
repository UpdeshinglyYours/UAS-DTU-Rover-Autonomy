/**
 * @file
 * @copyright Copyright (C) 2020, Blickfeld GmbH
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * Header for class blickfeld driver types
 */

#pragma once

#include <blickfeld/scanner.h>

namespace blickfeld {
namespace ros_interop {

/**
 * @brief imu acceleration unit in published imu data
 *
 */
enum class ImuAccelerationUnit { G, MetersPerSecondSquared };

/**
 * @brief log level for messages
 *
 */
enum class LogLevel { Critical, Warning, Debug };

/**
 * @brief Indicates how an image row should be projected
 *
 */
enum class ProjectionType { AnglePreserving, ScanlinePreserving };

/**
 * @brief In the multi return case this only forwards the desired return type to ROS
 *
 */
enum class ReturnOptions { Strongest, Closest, Farthest, All };

struct DebuggingTimes {
  std::chrono::high_resolution_clock::time_point frame_receive_start_time;
  std::chrono::high_resolution_clock::time_point frame_receive_end_time;
  std::chrono::high_resolution_clock::time_point imu_static_start_time;
  std::chrono::high_resolution_clock::time_point imu_static_end_time;
  std::chrono::high_resolution_clock::time_point imu_stream_start_time;
  std::chrono::high_resolution_clock::time_point imu_stream_end_time;
  std::chrono::high_resolution_clock::time_point parse_frame_end_time;
  std::chrono::high_resolution_clock::time_point parse_status_end_time;
};

struct DeviceAlgorithmOptions {
  bool use_background_subtraction = false;
  int background_subtraction_num_initialization_frames = 10;
  float background_subtraction_exponential_decay_rate = 0.005f;
  bool use_neighbor_filter = false;
};

struct ImageOptions {
  bool range_image = false;
  bool ambient_image = false;
  bool intensity_image = false;
  bool point_id_image = false;

  ProjectionType projection_type = ProjectionType::AnglePreserving;
};

struct ImuOptions {
  bool imu_stream = false;
  bool imu_static_tf = false;
  ImuAccelerationUnit imu_acceleration_unit = ImuAccelerationUnit::G;
  std::string lidar_frame_id = "lidar";
};

struct PointCloudOptions {
  bool range = false;
  bool angles = false;
  bool intensities = false;
  bool ambient_light = false;
  bool point_id = false;
  bool point_time_offset = false;
  std::string lidar_frame_id = "lidar";
  bool no_return_points = false;
  float no_return_point_range = 1.f;
  ReturnOptions return_options = ReturnOptions::Strongest;
};

struct FrameStreamOutput {
  PointCloud2Ptr point_cloud_msg;
  SensorMsgImagePtr range_image;
  SensorMsgImagePtr intensity_image;
  SensorMsgImagePtr ambient_image;
  SensorMsgImagePtr point_id_image;
  time_t frame_time;
};

struct SetScanPatternRequest {
  /// in deg
  float fov_horizontal;
  /// in deg
  float fov_vertical;
  /// in deg
  float angle_spacing;
  uint16_t scanlines_up;
  uint16_t scanlines_down;
  std::string frame_mode;
  std::string pulse_type;
};

struct SetScanPatternResponse {
  bool success;
  std::string message;
};

using FrameT = blickfeld::protocol::data::Frame;
using FrameModeT = blickfeld::protocol::config::ScanPattern::Pulse::FrameMode;
using PointT = blickfeld::protocol::data::Point;
using PointReturnT = blickfeld::protocol::data::Point_Return;
using PointReturnsT = google::protobuf::RepeatedPtrField<PointReturnT>;
using LogMessages = std::unordered_map<LogLevel, std::ostringstream>;
using ScanlineT = blickfeld::protocol::data::Scanline;
using ScanPatternT = blickfeld::protocol::config::ScanPattern;

}  // namespace ros_interop
}  // namespace blickfeld
