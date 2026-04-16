/**
 * @file
 * @copyright Copyright (C) 2020, Blickfeld GmbH
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * Header for class BlickfeldDriverImuParser
 */

#pragma once

#include <string>

#include <Eigen/Dense>

#include <blickfeld/scanner.h>

#include "blickfeld_driver/blickfeld_driver_ros_types.h"
#include "blickfeld_driver_core/blickfeld_driver_types.h"

namespace blickfeld {
namespace ros_interop {

/**
 * @brief This class converts imu frame to ros imu message and static tf message
 *
 */
class BlickfeldDriverImuParser {
 public:
  /**
   * @brief Constructor
   *
   * @param[in] acceleration_unit imu acceleration unit in g or m/s^2
   * @param[in] frame_id imu frame_id
   */
  BlickfeldDriverImuParser(const ImuAccelerationUnit& acceleration_unit = ImuAccelerationUnit::G,
                           const std::string& frame_id = "frame_id");

  /**
   * @brief parseImuSamples converts a blickfeld::protocol::data::IMU (a list of imu states containing imu acceleration
   * and angular velocity) to list of ros sensor msg imu
   *
   * @param[in] imu blickfeld::protocol::data::IMU
   * @return std::vector<std::pair<SensorMsgImuPtr,time_t>> generated list of ros sensor imu messages and imu time pairs
   */
  std::vector<std::pair<SensorMsgImuPtr, time_t>> parseImuSamples(const blickfeld::protocol::data::IMU& imu);

  /**
   * @brief parseImuStatic creates a transform message stamped based on imu acceleration
   *
   * @param[in] imu_acceleration
   * @return TransformMsgStampedPtr
   */
  TransformMsgStampedPtr parseImuStatic(const google::protobuf::RepeatedField<float>& imu_acceleration);

 private:
  ImuAccelerationUnit acceleration_unit_;
  std::string frame_id_;

  static constexpr auto imu_minimum_error_allowed_ = 1e-6;
  static constexpr auto g_to_meters_per_second_squared_ = 9.80665;
};

}  // namespace ros_interop
}  // namespace blickfeld
