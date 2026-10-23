#include "nav2_custom_bt_nodes/escape_nodes.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace nav2_custom_bt_nodes
{

namespace
{
double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double wrap(double a)
{
  return std::remainder(a, 2.0 * M_PI);
}

// Rover pose in `frame` from TF (latest).
bool poseIn(
  const std::shared_ptr<tf2_ros::Buffer> & tf, const std::string & frame, double & x, double & y, double & yaw)
{
  try {
    auto t = tf->lookupTransform(frame, "base_link", tf2::TimePointZero);
    x = t.transform.translation.x;
    y = t.transform.translation.y;
    yaw = yawOf(t.transform.rotation);
  } catch (const tf2::TransformException &) {
    return false;
  }
  return true;
}

// Rover pose in `frame` from TF (latest), with the transform's time stamp [s].
bool poseStampedIn(
  const std::shared_ptr<tf2_ros::Buffer> & tf, const std::string & frame,
  double & x, double & y, double & yaw, double & t)
{
  try {
    auto tr = tf->lookupTransform(frame, "base_link", tf2::TimePointZero);
    x = tr.transform.translation.x;
    y = tr.transform.translation.y;
    yaw = yawOf(tr.transform.rotation);
    t = rclcpp::Time(tr.header.stamp).seconds();
  } catch (const tf2::TransformException &) {
    return false;
  }
  return true;
}

// Nearest casualty: distance and the bearing from it to (x, y). Casualties are in the map frame,
// which is also the local costmap's frame here (global_frame: map).
double nearestCasualty(
  const std::vector<geometry_msgs::msg::PoseStamped> & cas, double x, double y, double & cx, double & cy)
{
  double best = std::numeric_limits<double>::infinity();
  for (const auto & p : cas) {
    const double d = std::hypot(x - p.pose.position.x, y - p.pose.position.y);
    if (d < best) {
      best = d;
      cx = p.pose.position.x;
      cy = p.pose.position.y;
    }
  }
  return best;
}
}  // namespace

// ==============================================================================
// LocalCostmapView
// ==============================================================================
LocalCostmapView::LocalCostmapView(const rclcpp::Node::SharedPtr & node)
{
  cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(cb_group_, node->get_node_base_interface());
  rclcpp::SubscriptionOptions opt;
  opt.callback_group = cb_group_;
  sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(
    "local_costmap/costmap_raw", rclcpp::QoS(1).transient_local().reliable(),
    [this](nav2_msgs::msg::Costmap::SharedPtr m) {costmap_ = m;}, opt);
}

unsigned char LocalCostmapView::costAt(double wx, double wy) const
{
  const auto & md = costmap_->metadata;
  const int mx = static_cast<int>(std::floor((wx - md.origin.position.x) / md.resolution));
  const int my = static_cast<int>(std::floor((wy - md.origin.position.y) / md.resolution));
  if (mx < 0 || my < 0 || mx >= static_cast<int>(md.size_x) || my >= static_cast<int>(md.size_y)) {
    return 0;
  }
  return costmap_->data[my * md.size_x + mx];
}

unsigned char LocalCostmapView::maxCostAround(double wx, double wy, double r) const
{
  const double res = costmap_->metadata.resolution;
  unsigned char best = 0;
  for (double dx = -r; dx <= r + 1e-9; dx += res) {
    for (double dy = -r; dy <= r + 1e-9; dy += res) {
      if (dx * dx + dy * dy <= r * r) {
        best = std::max(best, costAt(wx + dx, wy + dy));
      }
    }
  }
  return best;
}

double LocalCostmapView::clearLength(double x, double y, double yaw, double hl, double hw, double max_len) const
{
  const double res = costmap_->metadata.resolution;
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (double fx = hl; fx <= hl + max_len + 1e-9; fx += res) {
    for (double fy = -hw; fy <= hw + 1e-9; fy += res) {
      if (costAt(x + fx * c - fy * s, y + fx * s + fy * c) >= 254) {
        return std::max(0.0, fx - hl);
      }
    }
  }
  return max_len;
}

void LocalCostmapView::lethalAround(double x, double y, double r, double & nearest, double & ax, double & ay) const
{
  const double res = costmap_->metadata.resolution;
  nearest = std::numeric_limits<double>::infinity();
  ax = ay = 0.0;
  for (double dx = -r; dx <= r + 1e-9; dx += res) {
    for (double dy = -r; dy <= r + 1e-9; dy += res) {
      const double d = std::hypot(dx, dy);
      if (d > r || d < 1e-3 || costAt(x + dx, y + dy) < 254) {
        continue;
      }
      nearest = std::min(nearest, d);
      ax -= dx / (d * d);   // unit vector from the cell to the rover, weighted 1/d
      ay -= dy / (d * d);
    }
  }
}

// ==============================================================================
// KeepoutClear
// ==============================================================================
BT::NodeStatus KeepoutClear::tick()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  auto tf = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  std::vector<geometry_msgs::msg::PoseStamped> cas;
  if (!getInput("casualties", cas) || cas.empty()) {
    // Fail loud: without the casualty list the rover must not drive.
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 2000, "KeepoutClear: no casualties on the blackboard");
    return BT::NodeStatus::FAILURE;
  }
  double kr = 2.0, cr = 0.6, margin = 0.1, react = 0.15, decel = 2.0;
  getInput("keepout_radius", kr);
  getInput("circumradius", cr);
  getInput("margin", margin);
  getInput("reaction_time", react);
  getInput("decel", decel);

  double x, y, yaw, t;
  if (!poseStampedIn(tf, "map", x, y, yaw, t)) {
    return BT::NodeStatus::FAILURE;
  }
  if (last_d_.size() != cas.size()) {
    last_d_.assign(cas.size(), 0.0);
    v_in_.assign(cas.size(), 0.0);
    last_t_ = -1.0;
  }
  // The BT ticks faster than TF updates: only difference poses with a new stamp.
  const double dt = t - last_t_;
  const bool stale = last_t_ < 0.0 || dt < 0.0 || dt > 0.5;
  const bool fresh = !stale && dt >= 0.02;

  bool clear = true;
  for (size_t i = 0; i < cas.size(); ++i) {
    const double d = std::hypot(x - cas[i].pose.position.x, y - cas[i].pose.position.y);
    if (stale) {
      v_in_[i] = 0.0;
      last_d_[i] = d;
    } else if (fresh) {
      v_in_[i] = 0.5 * v_in_[i] + 0.5 * std::max((last_d_[i] - d) / dt, 0.0);
      last_d_[i] = d;
    }
    const double v = v_in_[i];
    const double stop = v * react + v * v / (2.0 * decel);
    const double clearance = d - cr - kr;
    if (clearance - stop <= margin) {
      clear = false;
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 1000,
        "KeepoutClear: STOP near casualty (%.2f, %.2f): centre %.2f m, corner clearance %.2f m, "
        "inward %.2f m/s needs %.2f m", cas[i].pose.position.x, cas[i].pose.position.y, d, clearance, v,
        stop + margin);
    }
  }
  if (stale || fresh) {
    last_t_ = t;
  }
  return clear ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ==============================================================================
// ComputeEscapeHeading
// ==============================================================================
ComputeEscapeHeading::ComputeEscapeHeading(const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  view_ = std::make_unique<LocalCostmapView>(config().blackboard->get<rclcpp::Node::SharedPtr>("node"));
}

BT::NodeStatus ComputeEscapeHeading::tick()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  auto tf = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  view_->update();
  if (!view_->ready()) {
    RCLCPP_WARN(node->get_logger(), "ComputeEscapeHeading: no local costmap yet");
    return BT::NodeStatus::FAILURE;
  }
  std::vector<geometry_msgs::msg::PoseStamped> cas;
  getInput("casualties", cas);
  double require_cas = 0.0, obstacle_within = 1.3, search_r = 2.0, min_clear = 0.8, max_check = 2.5;
  double kr = 2.0, cr = 0.6, turn_margin = 0.05, hl = 0.38, hw = 0.35;
  getInput("require_casualty_within", require_cas);
  getInput("obstacle_within", obstacle_within);
  getInput("search_radius", search_r);
  getInput("min_clear", min_clear);
  getInput("max_check", max_check);
  getInput("keepout_radius", kr);
  getInput("circumradius", cr);
  getInput("turn_margin", turn_margin);
  getInput("half_length", hl);
  getInput("half_width", hw);

  double x, y, yaw;
  if (!poseIn(tf, view_->frame(), x, y, yaw)) {
    RCLCPP_WARN(node->get_logger(), "ComputeEscapeHeading: no transform %s -> base_link", view_->frame().c_str());
    return BT::NodeStatus::FAILURE;
  }
  double cx = 0.0, cy = 0.0;
  const double d_cas = nearestCasualty(cas, x, y, cx, cy);
  if (require_cas > 0.0 && d_cas > require_cas) {
    return BT::NodeStatus::FAILURE;  // keepout-only escape and no casualty close by
  }
  double nearest, ax, ay;
  view_->lethalAround(x, y, search_r, nearest, ax, ay);
  const bool near_disc = d_cas < kr + 1.0;
  if (nearest > obstacle_within && !near_disc) {
    RCLCPP_INFO(node->get_logger(), "ComputeEscapeHeading: nothing within %.1f m to escape from", obstacle_within);
    return BT::NodeStatus::FAILURE;
  }
  const double out = std::atan2(y - cy, x - cx);          // away from the nearest casualty
  const double away = std::atan2(ay, ax);                 // away from nearby lethal cells

  // Too close to a casualty disc to turn in place: a corner could cross keepout_radius.
  if (d_cas < kr + cr + turn_margin) {
    const double len = view_->clearLength(x, y, yaw, hl, hw + 0.05, max_check);
    if (std::cos(wrap(yaw - out)) >= 0.5 && len >= 0.3) {
      RCLCPP_WARN(node->get_logger(), "ComputeEscapeHeading: casualty centre %.2f m, too close to turn: keep heading "
        "(%.0f deg off away, %.2f m clear)", d_cas, wrap(yaw - out) * 180.0 / M_PI, len);
      setOutput("heading", yaw);
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_ERROR(node->get_logger(), "ComputeEscapeHeading: casualty centre %.2f m, too close to turn safely and not "
      "facing away (%.0f deg off): no escape", d_cas, wrap(yaw - out) * 180.0 / M_PI);
    return BT::NodeStatus::FAILURE;
  }

  double best_score = -1e9, best_heading = yaw, best_len = 0.0;
  for (int k = 0; k < 36; ++k) {
    const double h = wrap(yaw + k * M_PI / 18.0);
    if (d_cas < 4.0 && std::cos(wrap(h - out)) < 0.0) {
      continue;  // never head toward a casualty close by
    }
    const double len = view_->clearLength(x, y, h, hl, hw + 0.05, max_check);
    if (len < 0.4) {
      continue;
    }
    double score = std::min(len, max_check) / max_check
      + 0.7 * std::cos(wrap(h - (near_disc ? out : away)))
      - 0.25 * std::fabs(wrap(h - yaw)) / M_PI;
    if (len < min_clear) {
      score -= 1.0;  // usable, but only if nothing better
    }
    if (score > best_score) {
      best_score = score;
      best_heading = h;
      best_len = len;
    }
  }
  if (best_score <= -1e8) {
    RCLCPP_ERROR(node->get_logger(), "ComputeEscapeHeading: no heading with a clear corridor (nearest obstacle %.2f m)",
      nearest);
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_WARN(node->get_logger(), "ComputeEscapeHeading: nearest obstacle %.2f m, casualty %.2f m: turn %.0f deg, "
    "%.2f m clear ahead", nearest, d_cas, wrap(best_heading - yaw) * 180.0 / M_PI, best_len);
  setOutput("heading", best_heading);
  return BT::NodeStatus::SUCCESS;
}

// ==============================================================================
// TurnToHeading
// ==============================================================================
TurnToHeading::TurnToHeading(const std::string & name, const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

BT::NodeStatus TurnToHeading::onStart()
{
  double heading = 0.0, tol_deg = 8.0, x, y, yaw;
  if (!getInput("heading", heading) || !poseIn(tf_, "map", x, y, yaw)) {
    return BT::NodeStatus::FAILURE;
  }
  getInput("tolerance_deg", tol_deg);
  const double err = wrap(heading - yaw);
  RCLCPP_INFO(node_->get_logger(), "TurnToHeading: heading error %.1f deg", err * 180.0 / M_PI);
  if (std::fabs(err) <= tol_deg * M_PI / 180.0) {
    return BT::NodeStatus::SUCCESS;
  }
  t0_ = node_->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TurnToHeading::onRunning()
{
  double heading = 0.0, tol_deg = 8.0, max_rate = 0.6, min_rate = 0.45, timeout = 8.0, x, y, yaw;
  getInput("heading", heading);
  getInput("tolerance_deg", tol_deg);
  getInput("max_rate", max_rate);
  getInput("min_rate", min_rate);
  getInput("timeout", timeout);
  geometry_msgs::msg::Twist cmd;
  if (!poseIn(tf_, "map", x, y, yaw)) {
    cmd_pub_->publish(cmd);
    return BT::NodeStatus::FAILURE;
  }
  const double err = wrap(heading - yaw);
  const double elapsed = (node_->now() - t0_).seconds();
  if (std::fabs(err) <= tol_deg * M_PI / 180.0) {
    cmd_pub_->publish(cmd);
    RCLCPP_INFO(node_->get_logger(), "TurnToHeading: done, %.1f deg off after %.1f s", err * 180.0 / M_PI, elapsed);
    return BT::NodeStatus::SUCCESS;
  }
  if (elapsed > timeout) {
    cmd_pub_->publish(cmd);
    RCLCPP_WARN(node_->get_logger(), "TurnToHeading: gave up, %.1f deg off after %.1f s", err * 180.0 / M_PI, elapsed);
    return BT::NodeStatus::FAILURE;
  }
  cmd.angular.z = std::copysign(std::clamp(1.2 * std::fabs(err), min_rate, max_rate), err);
  cmd_pub_->publish(cmd);
  return BT::NodeStatus::RUNNING;
}

void TurnToHeading::onHalted()
{
  cmd_pub_->publish(geometry_msgs::msg::Twist());
}

// ==============================================================================
// DriveForwardClear
// ==============================================================================
DriveForwardClear::DriveForwardClear(const std::string & name, const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  view_ = std::make_unique<LocalCostmapView>(node_);
  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

void DriveForwardClear::stop()
{
  cmd_pub_->publish(geometry_msgs::msg::Twist());
}

BT::NodeStatus DriveForwardClear::onStart()
{
  std::vector<geometry_msgs::msg::PoseStamped> cas;
  getInput("casualties", cas);
  double near = 3.0;
  getInput("casualty_near", near);
  double x, y, yaw;
  if (!poseIn(tf_, "map", x, y, yaw)) {
    return BT::NodeStatus::FAILURE;
  }
  const double d_cas = nearestCasualty(cas, x, y, cx_, cy_);
  near_casualty_ = d_cas < near;
  if (near_casualty_) {
    const double off = wrap(yaw - std::atan2(y - cy_, x - cx_));
    if (std::cos(off) < 0.5) {
      RCLCPP_WARN(node_->get_logger(), "DriveForwardClear: casualty %.2f m away and not facing away (%.0f deg off), "
        "not driving", d_cas, off * 180.0 / M_PI);
      return BT::NodeStatus::FAILURE;
    }
  }
  x0_ = x;
  y0_ = y;
  t0_ = node_->now();
  stop();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DriveForwardClear::onRunning()
{
  view_->update();
  double settle = 1.0, speed = 0.25, min_d = 0.3, max_d = 1.2, cas_clear = 3.5, hl = 0.38, hw = 0.35;
  double look = 0.5, timeout = 8.0;
  int clear_cost = 128;
  getInput("settle_time", settle);
  getInput("speed", speed);
  getInput("min_distance", min_d);
  getInput("max_distance", max_d);
  getInput("clear_cost", clear_cost);
  getInput("casualty_clear", cas_clear);
  getInput("half_length", hl);
  getInput("half_width", hw);
  getInput("lookahead", look);
  getInput("timeout", timeout);

  const double elapsed = (node_->now() - t0_).seconds();
  if (elapsed > timeout) {
    stop();
    RCLCPP_WARN(node_->get_logger(), "DriveForwardClear: timed out after %.1f s", elapsed);
    return BT::NodeStatus::FAILURE;
  }
  if (elapsed < settle || !view_->ready()) {
    if (!view_->ready() && elapsed > settle + 2.0) {
      stop();
      RCLCPP_WARN(node_->get_logger(), "DriveForwardClear: no local costmap, not driving blind");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;  // let the lidar mark what is now ahead
  }
  double x, y, yaw, mx, my, myaw;
  if (!poseIn(tf_, "map", x, y, yaw) || !poseIn(tf_, view_->frame(), mx, my, myaw)) {
    stop();
    return BT::NodeStatus::FAILURE;
  }
  const double moved = std::hypot(x - x0_, y - y0_);
  const double d_cas = std::hypot(x - cx_, y - cy_);
  const int cost = view_->maxCostAround(mx, my, 0.1);
  const bool clear_of_cas = !near_casualty_ || d_cas >= cas_clear;
  if (moved >= max_d || (moved >= min_d && cost < clear_cost && clear_of_cas)) {
    stop();
    RCLCPP_INFO(node_->get_logger(), "DriveForwardClear: out after %.2f m (centre cost %d, casualty %.2f m)",
      moved, cost, d_cas);
    return BT::NodeStatus::SUCCESS;
  }
  if (view_->clearLength(mx, my, myaw, hl, hw + 0.05, look) < look) {
    stop();
    RCLCPP_WARN(node_->get_logger(), "DriveForwardClear: obstacle ahead after %.2f m", moved);
    return moved >= min_d ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = speed;
  cmd_pub_->publish(cmd);
  return BT::NodeStatus::RUNNING;
}

void DriveForwardClear::onHalted()
{
  stop();
}

}  // namespace nav2_custom_bt_nodes
