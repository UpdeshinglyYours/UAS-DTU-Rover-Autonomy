/**
 * @file
 * @copyright Copyright (C) 2020, Blickfeld GmbH
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * Implementation for class BlickfeldDriverComponent
 */

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "blickfeld_driver/blickfeld_driver.h"

namespace blickfeld {
namespace ros_interop {

/**
 * @class blickfeld::ros_interop::BlickfeldDriverComponent
 * @brief Class wrapping the Blickfeld driver into a component
 *
 */
class BlickfeldDriverComponent {
 public:
  /**
   * @brief Constructor
   *
   * @param[in] options the node options
   */
  explicit BlickfeldDriverComponent(rclcpp::NodeOptions options) : driver_(std::make_unique<BlickfeldDriver>(options)) {
    driver_->init();
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const {
    return driver_->get_node_base_interface();
  }

 private:
  std::unique_ptr<BlickfeldDriver> driver_;
};

}  // namespace ros_interop
}  // namespace blickfeld

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(blickfeld::ros_interop::BlickfeldDriverComponent)
