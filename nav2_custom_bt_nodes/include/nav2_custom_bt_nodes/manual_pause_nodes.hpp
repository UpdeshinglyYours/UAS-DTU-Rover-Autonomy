#ifndef NAV2_CUSTOM_BT_NODES_MANUAL_PAUSE_NODES_HPP_
#define NAV2_CUSTOM_BT_NODES_MANUAL_PAUSE_NODES_HPP_

#include <atomic>
#include <string>
#include <vector>
#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "mavros_msgs/msg/state.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_custom_bt_nodes
{

// ==============================================================================
// 1. PauseWhileModeManual (Action Node)
// ------------------------------------------------------------------------------
// Purpose: Freeze the mission while the operator has the rover in MANUAL or HOLD.
// Logic:   Listens to /mavros/state. Any of pause_modes ("MANUAL,HOLD") latches "paused",
//          resume_mode (GUIDED) clears it, any other mode leaves it as it is.
//          Returns RUNNING while paused, FAILURE otherwise. Put it as the FIRST
//          child of a ReactiveFallback: while it is RUNNING the fallback halts the
//          mission branch, which cancels every Nav2 action in flight (FollowPath,
//          ComputePathToPose, Wait, Spin...) so the controller stops sending cmd_vel.
// ==============================================================================
class PauseWhileModeManual : public BT::ActionNodeBase
{
public:
  PauseWhileModeManual(const std::string & name, const BT::NodeConfiguration & config);
  ~PauseWhileModeManual() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("state_topic", "/mavros/state", "mavros_msgs/State topic"),
      BT::InputPort<std::string>("pause_modes", "MANUAL,HOLD", "Comma-separated modes that freeze the mission"),
      BT::InputPort<std::string>("resume_mode", "GUIDED", "Mode that unfreezes it"),
      // gate1: set true when a pause ends (TrimToNearestWaypoint reads and clears it); optional
      BT::OutputPort<bool>("resumed", "Set true when a pause ends"),
      // gate1: where the rover was when the pause started (map frame); optional
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("paused_at", "Rover pose when the pause started")
    };
  }

  BT::NodeStatus tick() override;
  void halt() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr sub_;

  std::vector<std::string> pause_modes_;
  std::string resume_mode_;
  std::atomic<bool> paused_{false};
  std::string last_mode_;
  rclcpp::Time paused_since_;
};

// ==============================================================================
// 2. ResumeOrPopNextGoal (Action Node)
// ------------------------------------------------------------------------------
// Purpose: PopNextGoal that survives a halt (pause), so a resume redoes the SAME casualty.
// Logic:   If {in_progress} is true and {input_goals} is still exactly the list this node
//          left behind at its last pop, the previous casualty was interrupted: output it
//          again without popping. Otherwise pop the front like PopNextGoal and set
//          {in_progress}=true. MarkGoalDone sets it back to false once the casualty is
//          finished. The list check stops a stale flag from a cancelled mission leaking into
//          the next one (bt_navigator keeps the blackboard between goals; a new goal
//          rewrites {goals}). FAILURE when nothing is in progress and the list is empty.
// ==============================================================================
class ResumeOrPopNextGoal : public BT::SyncActionNode
{
public:
  ResumeOrPopNextGoal(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("input_goals", "List of remaining goals"),
      BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("output_goals", "List of goals minus the popped one"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_goal", "Current goal (re-sent on resume)"),
      BT::BidirectionalPort<bool>("in_progress", "True while the current goal is unfinished")
    };
  }
  BT::NodeStatus tick() override;

private:
  std::vector<geometry_msgs::msg::PoseStamped> remaining_;
  geometry_msgs::msg::PoseStamped current_;
};

// ==============================================================================
// 3. MarkGoalDone (Action Node)
// ------------------------------------------------------------------------------
// Purpose: Close the casualty opened by ResumeOrPopNextGoal ({in_progress}=false).
// ==============================================================================
class MarkGoalDone : public BT::SyncActionNode
{
public:
  MarkGoalDone(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf) {}

  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<bool>("in_progress", "Set to false")};
  }
  BT::NodeStatus tick() override;
};

// ==============================================================================
// 4. SpinToFaceGoal (Action Node, client of the behavior_server "spin" action)
// ------------------------------------------------------------------------------
// Purpose: Turn in place until the rover's nose points at the casualty.
// Logic:   On every new start it reads the rover's pose from TF and asks Spin for the
//          signed shortest angle to the casualty bearing (Nav2 Humble's own <Spin> reads
//          spin_dist only once, when the tree is built, so it cannot do this). Below
//          tolerance_deg it does not turn at all. It stops the spin (SUCCESS) once less than
//          done_tolerance_deg is left: Spin slows to min_rotational_vel for the last few
//          degrees, which is below what the rover can turn at, and would otherwise sit there
//          until time_allowance runs out. time_allowance = angle / expected_rate + time_margin.
// ==============================================================================
class SpinToFaceGoal : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::Spin>
{
public:
  SpinToFaceGoal(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtActionNode<nav2_msgs::action::Spin>(xml_tag_name, action_name, conf) {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Pose to face (casualty center)"),
      BT::InputPort<double>("tolerance_deg", 10.0, "Do not turn if already within this [deg]"),
      BT::InputPort<double>("done_tolerance_deg", 3.0, "Stop the spin with this much left [deg]"),
      BT::InputPort<double>("expected_rate", 0.5, "Expected turn rate, for time_allowance [rad/s]"),
      BT::InputPort<double>("time_margin", 5.0, "Extra time_allowance [s]")
    });
  }

  void on_tick() override;
  void on_wait_for_result(std::shared_ptr<const nav2_msgs::action::Spin::Feedback> feedback) override;
  BT::NodeStatus tick() override;

private:
  double target_{0.0};
  double traveled_{0.0};
  double done_tol_{0.0};
};

} // namespace nav2_custom_bt_nodes

#endif // NAV2_CUSTOM_BT_NODES_MANUAL_PAUSE_NODES_HPP_
