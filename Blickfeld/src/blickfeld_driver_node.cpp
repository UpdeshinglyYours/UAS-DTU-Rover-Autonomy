/**
 * @file
 * @copyright Copyright (C) 2020, Blickfeld GmbH
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * ROS2 node of BlickfeldDriver
 */
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "blickfeld_driver/blickfeld_driver.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto bf_node = std::make_shared<blickfeld::ros_interop::BlickfeldDriver>(rclcpp::NodeOptions());
  if (bf_node->init() == true) {
    rclcpp::spin(bf_node);
  }
  rclcpp::shutdown();

  return 0;
}
