#include "nav2_custom_bt_nodes/manual_pause_nodes.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>
#include "tf2_ros/buffer.h"

namespace nav2_custom_bt_nodes
{

// ==============================================================================
// PauseWhileModeManual Implementation
// ==============================================================================
PauseWhileModeManual::PauseWhileModeManual(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
  config.blackboard->get("node", node_);

  std::string topic = "/mavros/state";
  getInput("state_topic", topic);
  std::string pause_modes = "MANUAL,HOLD";
  getInput("pause_modes", pause_modes);
  std::stringstream ss(pause_modes);
  for (std::string m; std::getline(ss, m, ',');) {
    m.erase(std::remove(m.begin(), m.end(), ' '), m.end());
    if (!m.empty()) pause_modes_.push_back(m);
  }
  resume_mode_ = "GUIDED";
  getInput("resume_mode", resume_mode_);

  // bt_navigator never spins this node's default callback group, so give the subscription
  // its own group and spin it from tick() (same pattern as Nav2's IsBatteryLowCondition).
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  // mavros publishes the state latched (reliable + transient_local): we get the current mode
  // as soon as we connect, not only at the next heartbeat.
  sub_ = node_->create_subscription<mavros_msgs::msg::State>(
    topic, rclcpp::QoS(10).transient_local(),
    [this](const mavros_msgs::msg::State::SharedPtr msg) {
      if (msg->mode == last_mode_) {
        return;
      }
      RCLCPP_WARN(node_->get_logger(), "[MANUAL PAUSE] mode %s -> %s",
                  last_mode_.empty() ? "(none)" : last_mode_.c_str(), msg->mode.c_str());
      last_mode_ = msg->mode;
      const bool pause_mode =
        std::find(pause_modes_.begin(), pause_modes_.end(), msg->mode) != pause_modes_.end();
      if (pause_mode && !paused_.load()) {
        RCLCPP_WARN(node_->get_logger(),
                    "\n\n⏸️ [MANUAL PAUSE] %s: mission FROZEN, Nav2 goals cancelled. "
                    "Switch to %s to continue.\n", msg->mode.c_str(), resume_mode_.c_str());
        paused_since_ = node_->now();
        paused_.store(true);
        // gate1: remember where the pause started (no-op when the port is not remapped)
        try {
          auto tf = this->config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
          auto t = tf->lookupTransform("map", "base_link", tf2::TimePointZero);
          geometry_msgs::msg::PoseStamped p;
          p.header.frame_id = "map";
          p.pose.position.x = t.transform.translation.x;
          p.pose.position.y = t.transform.translation.y;
          setOutput("paused_at", p);
        } catch (const std::exception &) {
        }
      } else if (msg->mode == resume_mode_ && paused_.load()) {
        RCLCPP_WARN(node_->get_logger(),
                    "\n\n▶️ [MANUAL PAUSE] %s: resuming after %.1f s (current casualty is redone "
                    "from here).\n", resume_mode_.c_str(), (node_->now() - paused_since_).seconds());
        paused_.store(false);
        setOutput("resumed", true);  // no-op when the port is not remapped (gate3)
      }
    },
    options);
}

BT::NodeStatus PauseWhileModeManual::tick()
{
  callback_group_executor_.spin_some();

  if (!paused_.load()) {
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                       "⏸️ [MANUAL PAUSE] Holding in %s... %.0f s", last_mode_.c_str(),
                       (node_->now() - paused_since_).seconds());
  return BT::NodeStatus::RUNNING;
}

void PauseWhileModeManual::halt()
{
  // The pause follows the flight mode only: a halt from above (mission cancelled) does not
  // clear it.
  setStatus(BT::NodeStatus::IDLE);
}

// ==============================================================================
// ResumeOrPopNextGoal Implementation
// ==============================================================================
BT::NodeStatus ResumeOrPopNextGoal::tick()
{
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  if (!getInput("input_goals", goals)) return BT::NodeStatus::FAILURE;

  bool in_progress = false;
  getInput("in_progress", in_progress);  // unset on the first casualty -> false

  if (in_progress && goals == remaining_) {
    setOutput("output_goal", current_);
    return BT::NodeStatus::SUCCESS;
  }

  if (goals.empty()) return BT::NodeStatus::FAILURE;

  current_ = goals.front();
  goals.erase(goals.begin());
  remaining_ = goals;

  setOutput("output_goal", current_);
  setOutput("output_goals", goals);
  setOutput("in_progress", true);
  return BT::NodeStatus::SUCCESS;
}

// ==============================================================================
// MarkGoalDone Implementation
// ==============================================================================
BT::NodeStatus MarkGoalDone::tick()
{
  setOutput("in_progress", false);
  return BT::NodeStatus::SUCCESS;
}

// ==============================================================================
// SpinToFaceGoal Implementation
// ==============================================================================
void SpinToFaceGoal::on_tick()
{
  geometry_msgs::msg::PoseStamped goal;
  getInput("goal", goal);
  double tol_deg = 10.0, rate = 0.5, margin = 5.0, done_deg = 3.0;
  getInput("tolerance_deg", tol_deg);
  getInput("done_tolerance_deg", done_deg);
  getInput("expected_rate", rate);
  getInput("time_margin", margin);
  done_tol_ = done_deg * M_PI / 180.0;
  traveled_ = 0.0;
  target_ = 0.0;

  auto tf_buffer = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  try {
    auto t = tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
    const auto & q = t.transform.rotation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double bearing = std::atan2(goal.pose.position.y - t.transform.translation.y,
                                      goal.pose.position.x - t.transform.translation.x);
    const double err = std::atan2(std::sin(bearing - yaw), std::cos(bearing - yaw));
    if (std::abs(err) >= tol_deg * M_PI / 180.0) {
      target_ = err;
    }
    RCLCPP_INFO(node_->get_logger(), "SpinToFaceGoal: heading error %.1f deg -> spin %.1f deg",
                err * 180.0 / M_PI, target_ * 180.0 / M_PI);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(node_->get_logger(), "TF Error in SpinToFaceGoal: %s", ex.what());
  }

  // target 0 = Spin succeeds straight away
  goal_.target_yaw = target_;
  goal_.time_allowance = rclcpp::Duration::from_seconds(std::abs(target_) / rate + margin);
}

void SpinToFaceGoal::on_wait_for_result(
  std::shared_ptr<const nav2_msgs::action::Spin::Feedback> feedback)
{
  if (feedback) {
    traveled_ = feedback->angular_distance_traveled;
  }
}

BT::NodeStatus SpinToFaceGoal::tick()
{
  BT::NodeStatus status = BtActionNode<nav2_msgs::action::Spin>::tick();
  if (status == BT::NodeStatus::RUNNING && target_ != 0.0 &&
    std::abs(target_) - std::abs(traveled_) < done_tol_)
  {
    RCLCPP_INFO(node_->get_logger(), "SpinToFaceGoal: %.1f deg left, close enough",
                (std::abs(target_) - std::abs(traveled_)) * 180.0 / M_PI);
    halt();  // cancels the Spin; behavior_server stops the robot
    return BT::NodeStatus::SUCCESS;
  }
  return status;
}

} // namespace nav2_custom_bt_nodes
