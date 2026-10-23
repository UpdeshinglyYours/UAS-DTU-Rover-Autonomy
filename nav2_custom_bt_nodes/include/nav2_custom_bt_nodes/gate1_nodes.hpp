#ifndef NAV2_CUSTOM_BT_NODES_GATE1_NODES_HPP_
#define NAV2_CUSTOM_BT_NODES_GATE1_NODES_HPP_

#include <string>
#include <vector>
#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

// gate1_tree: lawnmower search. The goal array is a list of WAYPOINTS, consumed from the front.

namespace nav2_custom_bt_nodes
{

using Waypoints = std::vector<geometry_msgs::msg::PoseStamped>;

// ==============================================================================
// 1. DropPassedWaypoints (Action Node)
// ------------------------------------------------------------------------------
// Purpose: Remove waypoints the rover has reached or driven past, from the front of the list.
// Logic:   The front waypoint is dropped if the rover is within `radius` of it, or if it is within
//          `pass_radius` and already past it (beyond it along the line to the next waypoint).
//          Repeats for the new front. With keep_last=true the last waypoint is never dropped (so a
//          FollowPath to it can finish). Always SUCCESS.
// ==============================================================================
class DropPassedWaypoints : public BT::SyncActionNode
{
public:
  DropPassedWaypoints(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Waypoints>("input_goals", "Remaining waypoints"),
      BT::OutputPort<Waypoints>("output_goals", "Remaining waypoints minus the passed ones"),
      BT::InputPort<double>("radius", 1.0, "Reached when the rover centre is this close [m]"),
      BT::InputPort<double>("pass_radius", 3.0, "Passed: this close and beyond it towards the next [m]"),
      BT::InputPort<bool>("keep_last", false, "Never drop the last waypoint")
    };
  }
  BT::NodeStatus tick() override;
};

// ==============================================================================
// 2. WaypointWindow (Action Node)
// ------------------------------------------------------------------------------
// Purpose: The next `size` waypoints, for ComputePathThroughPoses. FAILURE when none are left.
// ==============================================================================
class WaypointWindow : public BT::SyncActionNode
{
public:
  WaypointWindow(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Waypoints>("input_goals", "Remaining waypoints"),
      BT::InputPort<int>("size", 2, "How many waypoints to plan through"),
      BT::OutputPort<Waypoints>("window", "The first `size` waypoints")
    };
  }
  BT::NodeStatus tick() override;
};

// ==============================================================================
// 3. SkipWaypoint (Action Node)
// ------------------------------------------------------------------------------
// Purpose: Give up on the front waypoint (it is never revisited). Always SUCCESS.
// ==============================================================================
class SkipWaypoint : public BT::SyncActionNode
{
public:
  SkipWaypoint(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Waypoints>("input_goals", "Remaining waypoints"),
      BT::OutputPort<Waypoints>("output_goals", "Remaining waypoints minus the front one")
    };
  }
  BT::NodeStatus tick() override;
};

// ==============================================================================
// 4. TrimToNearestWaypoint (Action Node)
// ------------------------------------------------------------------------------
// Purpose: After a MANUAL/HOLD pause, carry on from where the operator left the rover.
// Logic:   Only when {trigger} is true (set by PauseWhileModeManual on resume), then {trigger}=false.
//          If the rover is within move_threshold of where the pause started, nothing is dropped.
//          Otherwise the closest point on the REMAINING PATTERN is found: the leg the rover was on
//          (pause position -> first waypoint) and the segments between the remaining waypoints. If it
//          lies on segment w[i] -> w[i+1], waypoints w[0..i] are dropped and the rover continues to
//          w[i+1] (no going back). Following the pattern, not the nearest point: on a lawnmower the
//          next lane is often closer than the next point on the current lane. Always SUCCESS.
// ==============================================================================
class TrimToNearestWaypoint : public BT::SyncActionNode
{
public:
  TrimToNearestWaypoint(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Waypoints>("input_goals", "Remaining waypoints"),
      BT::OutputPort<Waypoints>("output_goals", "Remaining waypoints from the nearest one on"),
      BT::BidirectionalPort<bool>("trigger", "True right after a pause ended"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("paused_at", "Rover pose when the pause started"),
      BT::InputPort<double>("move_threshold", 3.0, "Moved less than this during the pause: drop nothing [m]")
    };
  }
  BT::NodeStatus tick() override;
};

// ==============================================================================
// 5. ClearFlag (Action Node)
// ------------------------------------------------------------------------------
// Purpose: Set a bool blackboard entry to false (mission start). Always SUCCESS.
// ==============================================================================
class ClearFlag : public BT::SyncActionNode
{
public:
  ClearFlag(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf) {}

  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<bool>("flag", "Set to false")};
  }
  BT::NodeStatus tick() override;
};

} // namespace nav2_custom_bt_nodes

#endif // NAV2_CUSTOM_BT_NODES_GATE1_NODES_HPP_
