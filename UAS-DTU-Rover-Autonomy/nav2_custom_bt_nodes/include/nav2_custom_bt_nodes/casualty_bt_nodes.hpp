#ifndef NAV2_CUSTOM_BT_NODES_CASUALTY_BT_NODES_HPP_
#define NAV2_CUSTOM_BT_NODES_CASUALTY_BT_NODES_HPP_

#include <string>
#include <vector>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp" // Added for the subscriber
#include <atomic>                   // Added for thread safety

namespace nav2_custom_bt_nodes
{

// ==============================================================================
// 1. PopNextGoal (Action Node)
// ------------------------------------------------------------------------------
// Purpose: Iterates through the array of casualties. 
// Logic:   Reads the `{goals}` vector, extracts the 0th index, and saves the 
//          rest of the vector back to the blackboard.
// ==============================================================================
class PopNextGoal : public BT::SyncActionNode
{
public:
  PopNextGoal(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("input_goals", "List of remaining goals"),
      BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("output_goals", "List of goals minus the popped one"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_goal", "The popped goal")
    };
  }
  BT::NodeStatus tick() override;
};

// ==============================================================================
// 2. CalculateApproachPose (Action Node)
// ------------------------------------------------------------------------------
// Purpose: Calculates the safest, shortest-distance entry point to the circle.
// Logic:   Finds the robot's current position, draws a straight line to the 
//          casualty, and places the target coordinate exactly 3.5m away.
// ==============================================================================
class CalculateApproachPose : public BT::SyncActionNode
{
public:
  CalculateApproachPose(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("center", "Casualty center pose"),
      BT::InputPort<double>("radius", 3.5, "Orbit radius [m]"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_pose", "The approach pose")
    };
  }
  BT::NodeStatus tick() override;
};

// ==============================================================================
// 3. GenerateOrbitPath (Action Node)
// ------------------------------------------------------------------------------
// Purpose: Synthesizes a massive, curvy path for the controller to follow.
// Logic:   Generates a 350-degree path around the casualty. It uses 350 instead
//          of 360 degrees so the start/end points don't overlap, which would
//          accidentally trigger Nav2's goal-checker prematurely.
// ==============================================================================
class GenerateOrbitPath : public BT::SyncActionNode
{
public:
  GenerateOrbitPath(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("center", "Casualty center pose"),
      BT::InputPort<double>("radius", 3.5, "Orbit radius [m]"),
      BT::OutputPort<nav_msgs::msg::Path>("output_path", "Generated 350 path")
    };
  }
  BT::NodeStatus tick() override;
};

// ==============================================================================
// 4. IsGoalCloserThan (Condition Node)
// ------------------------------------------------------------------------------
// Purpose: The 100Hz hardware kill-switch. 
// Logic:   Because this inherits from ConditionNode (not ActionNode), it evaluates
//          instantly. It returns SUCCESS if the robot is INSIDE the danger zone, 
//          which triggers our XML <Inverter> to halt the controller.
// ==============================================================================
class IsGoalCloserThan : public BT::ConditionNode
{
public:
  IsGoalCloserThan(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(name, conf) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("distance", "Distance threshold [m]"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Target goal")
    };
  }
  BT::NodeStatus tick() override;
};

// ==============================================================================
// 5. GeneratePolygonOrbit (Action Node)
// ------------------------------------------------------------------------------
// Purpose: //
// Logic:   //
// ==============================================================================
class GenerateOrbitWaypoints : public BT::SyncActionNode
{
public:
  GenerateOrbitWaypoints(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("center", "Casualty center pose"),
      BT::InputPort<double>("radius", 3.5, "Orbit radius [m]"),
      BT::InputPort<int>("num_waypoints", 8, "Number of polygon points"),
      // Notice this outputs an array (std::vector), just like our main mission goals!
      BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("output_goals", "Generated waypoint array")
    };
  }
  BT::NodeStatus tick() override;
};

// ==============================================================================
// 6. DynamicWaitAction (Action Node)
// ------------------------------------------------------------------------------
// Purpose: Listens to a ROS topic and pauses the tree execution internally.
// Logic:   Returns RUNNING for X seconds, then returns FAILURE to resume mission.
// ==============================================================================
class DynamicWaitAction : public BT::ActionNodeBase
{
public:
  DynamicWaitAction(const std::string& name, const BT::NodeConfiguration& config);
  ~DynamicWaitAction() override = default;

  // No ports needed! We bypassed the Blackboard entirely.
  static BT::PortsList providedPorts() { return {}; }
  
  BT::NodeStatus tick() override;
  void halt() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
  
  std::atomic<bool> trigger_active_{false};
  double wait_time_{0.0};
  std::chrono::time_point<std::chrono::steady_clock> start_time_;
};

} // namespace nav2_custom_bt_nodes

#endif // NAV2_CUSTOM_BT_NODES_CASUALTY_BT_NODES_HPP_
