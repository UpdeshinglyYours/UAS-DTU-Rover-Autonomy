/**
 * @file
 * @copyright Copyright (C) 2020, Blickfeld GmbH
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * Header for blickfeld driver ros types
 */

#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace blickfeld {
namespace ros_interop {

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2Ptr = sensor_msgs::msg::PointCloud2::UniquePtr;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using DiagnosticStatusPtr = diagnostic_msgs::msg::DiagnosticStatus::UniquePtr;
using DiagnosticKeyValue = diagnostic_msgs::msg::KeyValue;
using PointField = sensor_msgs::msg::PointField;
using SensorMsgImage = sensor_msgs::msg::Image;
using SensorMsgImagePtr = sensor_msgs::msg::Image::UniquePtr;
using MsgHeader = std_msgs::msg::Header;
using MsgHeaderPtr = std_msgs::msg::Header::SharedPtr;
using SensorMsgImu = sensor_msgs::msg::Imu;
using SensorMsgImuPtr = sensor_msgs::msg::Imu::UniquePtr;
using TransformMsgStamped = geometry_msgs::msg::TransformStamped;
using TransformMsgStampedPtr = geometry_msgs::msg::TransformStamped::SharedPtr;

}  // namespace ros_interop
}  // namespace blickfeld
