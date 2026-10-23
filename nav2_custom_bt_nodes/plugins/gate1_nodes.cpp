#include "nav2_custom_bt_nodes/gate1_nodes.hpp"
#include <algorithm>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_custom_bt_nodes
{

namespace
{
bool robotXY(const BT::NodeConfiguration & conf, double & x, double & y)
{
  auto tf_buffer = conf.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  auto node = conf.blackboard->get<rclcpp::Node::SharedPtr>("node");
  try {
    auto t = tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
    x = t.transform.translation.x;
    y = t.transform.translation.y;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(node->get_logger(), "TF Error in gate1 waypoint node: %s", ex.what());
    return false;
  }
}
}  // namespace

// ==============================================================================
// DropPassedWaypoints Implementation
// ==============================================================================
BT::NodeStatus DropPassedWaypoints::tick()
{
  Waypoints wps;
  getInput("input_goals", wps);
  double radius = 1.0, pass_radius = 3.0;
  bool keep_last = false;
  getInput("radius", radius);
  getInput("pass_radius", pass_radius);
  getInput("keep_last", keep_last);

  double x, y;
  if (wps.empty() || !robotXY(config(), x, y)) {
    setOutput("output_goals", wps);
    return BT::NodeStatus::SUCCESS;
  }
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  size_t n_drop = 0;
  while (n_drop < wps.size() && !(keep_last && wps.size() - n_drop == 1)) {
    const auto & a = wps[n_drop].pose.position;
    const double d = std::hypot(x - a.x, y - a.y);
    bool passed = d < radius;
    if (!passed && d < pass_radius && n_drop + 1 < wps.size()) {
      const auto & b = wps[n_drop + 1].pose.position;
      passed = (x - a.x) * (b.x - a.x) + (y - a.y) * (b.y - a.y) > 0.0;  // beyond a, towards b
    }
    if (!passed) {
      break;
    }
    RCLCPP_INFO(node->get_logger(), "gate1: waypoint (%.2f, %.2f) %s (%.2f m), %zu left",
                a.x, a.y, d < radius ? "reached" : "passed", d, wps.size() - n_drop - 1);
    ++n_drop;
  }
  if (n_drop > 0) {
    wps.erase(wps.begin(), wps.begin() + static_cast<long>(n_drop));
  }
  setOutput("output_goals", wps);
  return BT::NodeStatus::SUCCESS;
}

// ==============================================================================
// WaypointWindow Implementation
// ==============================================================================
BT::NodeStatus WaypointWindow::tick()
{
  Waypoints wps;
  getInput("input_goals", wps);
  int size = 2;
  getInput("size", size);
  if (wps.empty() || size < 1) {
    return BT::NodeStatus::FAILURE;
  }
  Waypoints window(wps.begin(), wps.begin() + std::min<size_t>(static_cast<size_t>(size), wps.size()));
  setOutput("window", window);
  return BT::NodeStatus::SUCCESS;
}

// ==============================================================================
// SkipWaypoint Implementation
// ==============================================================================
BT::NodeStatus SkipWaypoint::tick()
{
  Waypoints wps;
  getInput("input_goals", wps);
  if (!wps.empty()) {
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    RCLCPP_WARN(node->get_logger(), "gate1: SKIPPING waypoint (%.2f, %.2f) (unreachable), %zu left",
                wps.front().pose.position.x, wps.front().pose.position.y, wps.size() - 1);
    wps.erase(wps.begin());
  }
  setOutput("output_goals", wps);
  return BT::NodeStatus::SUCCESS;
}

// ==============================================================================
// TrimToNearestWaypoint Implementation
// ==============================================================================
BT::NodeStatus TrimToNearestWaypoint::tick()
{
  bool trigger = false;
  getInput("trigger", trigger);  // unset at the start of a mission -> false
  if (!trigger) {
    return BT::NodeStatus::SUCCESS;
  }
  setOutput("trigger", false);

  Waypoints wps;
  getInput("input_goals", wps);
  double x, y;
  if (wps.empty() || !robotXY(config(), x, y)) {
    return BT::NodeStatus::SUCCESS;
  }
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  double move_threshold = 3.0;
  getInput("move_threshold", move_threshold);
  geometry_msgs::msg::PoseStamped at;
  const bool have_at = static_cast<bool>(getInput("paused_at", at));
  const double px = have_at ? at.pose.position.x : x, py = have_at ? at.pose.position.y : y;
  const double moved = std::hypot(x - px, y - py);
  if (moved < move_threshold) {
    RCLCPP_INFO(node->get_logger(), "gate1: resumed after a pause, moved %.2f m: continuing as before "
                "(%zu waypoints left)", moved, wps.size());
    return BT::NodeStatus::SUCCESS;
  }

  // closest point on the remaining pattern; segment -1 = the leg it was on (pause position -> w[0])
  auto seg_dist = [&](double ax, double ay, double bx, double by) {
      const double vx = bx - ax, vy = by - ay, l2 = vx * vx + vy * vy;
      const double t = l2 > 1e-9 ? std::clamp(((x - ax) * vx + (y - ay) * vy) / l2, 0.0, 1.0) : 0.0;
      return std::hypot(x - (ax + t * vx), y - (ay + t * vy));
    };
  long best = -1;
  double best_d = seg_dist(px, py, wps[0].pose.position.x, wps[0].pose.position.y);
  for (size_t i = 0; i + 1 < wps.size(); ++i) {
    const double d = seg_dist(wps[i].pose.position.x, wps[i].pose.position.y,
                              wps[i + 1].pose.position.x, wps[i + 1].pose.position.y);
    if (d < best_d - 1e-6) {
      best_d = d;
      best = static_cast<long>(i);
    }
  }
  const size_t n_drop = static_cast<size_t>(best + 1);
  RCLCPP_WARN(node->get_logger(), "gate1: resumed after a pause, moved %.2f m: closest to the pattern is %s "
              "(%.2f m away): dropping %zu waypoint(s), %zu left", moved,
              best < 0 ? "the leg it was on" : "a later segment", best_d, n_drop, wps.size() - n_drop);
  if (n_drop > 0) {
    wps.erase(wps.begin(), wps.begin() + static_cast<long>(n_drop));
    setOutput("output_goals", wps);
  }
  return BT::NodeStatus::SUCCESS;
}

// ==============================================================================
// ClearFlag Implementation
// ==============================================================================
BT::NodeStatus ClearFlag::tick()
{
  setOutput("flag", false);
  return BT::NodeStatus::SUCCESS;
}

} // namespace nav2_custom_bt_nodes
