/**
 * @file
 * @copyright Copyright (C) 2020, Blickfeld GmbH
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * Implementation for class BlickfeldDriverImuParser
 */

#include "blickfeld_driver_core/blickfeld_driver_imu_parser.h"

namespace blickfeld {
namespace ros_interop {

BlickfeldDriverImuParser::BlickfeldDriverImuParser(const ImuAccelerationUnit& acceleration_unit,
                                                   const std::string& frame_id)
    : acceleration_unit_(acceleration_unit), frame_id_(frame_id) {}

std::vector<std::pair<SensorMsgImuPtr, time_t>> BlickfeldDriverImuParser::parseImuSamples(
    const blickfeld::protocol::data::IMU& imu) {
  std::vector<std::pair<SensorMsgImuPtr, time_t>> imu_msgs;
  for (auto sample : imu.samples()) {
    imu_msgs.emplace_back(std::pair<SensorMsgImuPtr, time_t>());
    imu_msgs.back().first = SensorMsgImuPtr(new SensorMsgImu());
    imu_msgs.back().second = imu.start_time_ns() + sample.start_offset_ns();

    imu_msgs.back().first->header.frame_id = frame_id_;

    imu_msgs.back().first->angular_velocity.x = sample.angular_velocity(0);
    imu_msgs.back().first->angular_velocity.y = sample.angular_velocity(1);
    imu_msgs.back().first->angular_velocity.z = sample.angular_velocity(2);

    const auto unit_conversion = acceleration_unit_ == ImuAccelerationUnit::G ? 1 : g_to_meters_per_second_squared_;
    imu_msgs.back().first->linear_acceleration.x = unit_conversion * sample.acceleration(0);
    imu_msgs.back().first->linear_acceleration.y = unit_conversion * sample.acceleration(1);
    imu_msgs.back().first->linear_acceleration.z = unit_conversion * sample.acceleration(2);
  }
  return imu_msgs;
}

TransformMsgStampedPtr BlickfeldDriverImuParser::parseImuStatic(
    const google::protobuf::RepeatedField<float>& imu_acceleration) {
  const Eigen::Vector3f gravity_in_aligned_lidar(0, 0, -1);

  /// HINT: The gravity vector is not actually in the lidar frame but the imu frame. Since we only care about the
  /// relative transformation from the imu to the gravity vector, the lidar-imu calibration can be ignored.
  const Eigen::Vector3f gravity_in_lidar(imu_acceleration.data());

  /// HINT: Find rotation that rotates the imu-data such that it aligns with [0, 0, -1]
  const Eigen::Quaternionf gravity_from_lidar =
      Eigen::Quaternionf::FromTwoVectors(gravity_in_lidar, gravity_in_aligned_lidar);

  /// HINT: gravity_from_lidar transforms the coordinates to gravity aligned frame, but tf2 expects a frame transform
  /// which is the inverse transformation
  const Eigen::Quaternionf lidar_to_gravity_frame_transform = gravity_from_lidar.inverse();
  const tf2::Quaternion lidar_to_gravity_frame_rot_tf2(
      lidar_to_gravity_frame_transform.x(), lidar_to_gravity_frame_transform.y(), lidar_to_gravity_frame_transform.z(),
      lidar_to_gravity_frame_transform.w());
  tf2::Vector3 lidar_to_gravity_frame_translation_tf2(0, 0, 0);

  TransformMsgStampedPtr static_transform_stamped(new TransformMsgStamped());
  static_transform_stamped->header.frame_id = frame_id_;
  static_transform_stamped->child_frame_id = frame_id_ + "_imu";
  static_transform_stamped->transform =
      tf2::toMsg(tf2::Transform(lidar_to_gravity_frame_rot_tf2, lidar_to_gravity_frame_translation_tf2));
  return static_transform_stamped;
}

}  // namespace ros_interop
}  // namespace blickfeld
