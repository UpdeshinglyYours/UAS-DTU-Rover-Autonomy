#ifndef NAV2_CUSTOM_BT_NODES_ESCAPE_NODES_HPP_
#define NAV2_CUSTOM_BT_NODES_ESCAPE_NODES_HPP_

#include <memory>
#include <string>
#include <vector>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_custom_bt_nodes
{

// ==============================================================================
// LocalCostmapView  -- darpa_sim_spark addition
// ------------------------------------------------------------------------------
// Latest raw local costmap (0-255) for BT nodes. bt_navigator never spins the BT's node, so
// the subscription lives on its own callback group, spun by update().
// ==============================================================================
class LocalCostmapView
{
public:
  explicit LocalCostmapView(const rclcpp::Node::SharedPtr & node);
  void update() {executor_.spin_some();}
  bool ready() const {return static_cast<bool>(costmap_);}
  const std::string & frame() const {return costmap_->header.frame_id;}
  // Cost at a point in the costmap frame (0 outside the map).
  unsigned char costAt(double wx, double wy) const;
  // Highest cost within r of a point (costmap frame).
  unsigned char maxCostAround(double wx, double wy, double r) const;
  // Clear length of a straight corridor of half-width hw that starts hl ahead of (x, y) along yaw,
  // up to max_len; ends at the first lethal (254) cell. Costmap frame.
  double clearLength(double x, double y, double yaw, double hl, double hw, double max_len) const;
  // Lethal cells within r of (x, y): nearest distance and the sum of unit vectors from each cell to
  // (x, y), weighted 1/distance (points away from the obstacles). Costmap frame.
  void lethalAround(double x, double y, double r, double & nearest, double & ax, double & ay) const;

private:
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr sub_;
  nav2_msgs::msg::Costmap::SharedPtr costmap_;
};

// ==============================================================================
// KeepoutClear (Condition Node)  -- darpa_sim_spark addition
// ------------------------------------------------------------------------------
// Purpose: Stop before any part of the rover can reach a casualty's keepout circle.
// Logic:   For every casualty, the rover's safety circle (circumradius, 0.6 m = local_costmap
//          robot_radius; the real corners reach 0.517 m) must stay at least
//          margin outside keepout_radius even after braking from its current inward speed
//          (reaction_time * v + v^2 / (2 * decel)). Checking a circle that contains the corners instead of the
//          body outline means the rover can always turn in place after the stop. The inward
//          speed is the change in distance between ticks, so no subscription is needed.
//          FAILURE = stop now.
// ==============================================================================
class KeepoutClear : public BT::ConditionNode
{
public:
  KeepoutClear(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(name, conf) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("casualties", "Every casualty to keep out of"),
      BT::InputPort<double>("keepout_radius", 2.0, "Disqualification radius around a casualty [m]"),
      BT::InputPort<double>("circumradius", 0.6, "Rover safety radius [m] (= local_costmap robot_radius; corners are 0.517)"),
      BT::InputPort<double>("margin", 0.1, "Clearance still left after braking [m]"),
      BT::InputPort<double>("reaction_time", 0.15, "Delay from the trip to braking [s]"),
      BT::InputPort<double>("decel", 2.0, "Braking deceleration assumed [m/s^2]")
    };
  }
  BT::NodeStatus tick() override;

private:
  double last_t_{-1.0};
  std::vector<double> last_d_;
  std::vector<double> v_in_;
};

// ==============================================================================
// ComputeEscapeHeading (Action Node)  -- darpa_sim_spark addition
// ------------------------------------------------------------------------------
// Purpose: First step of the general escape: which way to turn to get away from what the
//          rover is stuck against (rocks, walls, a casualty keepout disc).
// Logic:   Candidate headings every 10 deg, scored by the clear corridor ahead (local costmap),
//          by agreement with the direction away from lethal cells within search_radius, and by
//          a small penalty on the turn. Never a heading toward a casualty within 4 m.
//          FAILURE when there is nothing to escape from (no lethal cell within obstacle_within
//          and no casualty disc close by), when require_casualty_within is set and no casualty is
//          that close, or when no heading has a usable corridor. Within reach of a keepout disc
//          (centre < keepout_radius + circumradius + turn_margin) it does not turn: it keeps
//          the current heading if that already points away from the casualty, else fails.
// ==============================================================================
class ComputeEscapeHeading : public BT::SyncActionNode
{
public:
  ComputeEscapeHeading(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("casualties", "Every casualty"),
      BT::InputPort<double>("require_casualty_within", 0.0, "If > 0: fail unless a casualty centre is this close [m]"),
      BT::InputPort<double>("obstacle_within", 1.3, "Fail if no lethal cell is this close to the rover centre [m]"),
      BT::InputPort<double>("search_radius", 2.0, "Lethal cells this close push the heading away [m]"),
      BT::InputPort<double>("min_clear", 0.8, "Clear corridor wanted ahead of the front edge [m]"),
      BT::InputPort<double>("max_check", 2.5, "Corridor checked this far [m]"),
      BT::InputPort<double>("keepout_radius", 2.0, "Disqualification radius around a casualty [m]"),
      BT::InputPort<double>("circumradius", 0.6, "Rover safety radius [m] (= local_costmap robot_radius; corners are 0.517)"),
      BT::InputPort<double>("turn_margin", 0.05, "Extra clearance a turn in place must keep [m]"),
      BT::InputPort<double>("half_length", 0.38, "Rover centre to front edge [m]"),
      BT::InputPort<double>("half_width", 0.35, "Rover centre to side [m]"),
      BT::OutputPort<double>("heading", "Heading to turn to before driving out [rad, map frame]")
    };
  }
  BT::NodeStatus tick() override;

private:
  std::unique_ptr<LocalCostmapView> view_;
};

// ==============================================================================
// TurnToHeading (Action Node)  -- darpa_sim_spark addition
// ------------------------------------------------------------------------------
// Purpose: Turn in place to a heading from the blackboard, where Nav2's Spin refuses.
// Logic:   Spin checks the costmap's 0.6 m disc footprint, which touches the keepout disc (and
//          the inflated cells around rocks) exactly where an escape has to turn, so it aborts with
//          "Collision Ahead". This node turns closed-loop on the TF heading by publishing cmd_vel
//          itself, at min_rate..max_rate (above the rover's 0.3 rad/s turning floor). Whether the
//          turn is safe is decided before it, by ComputeEscapeHeading (corner reach vs keepout).
// ==============================================================================
class TurnToHeading : public BT::StatefulActionNode
{
public:
  TurnToHeading(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("heading", "Heading to turn to [rad, map frame]"),
      BT::InputPort<double>("tolerance_deg", 8.0, "Done when within this [deg]"),
      BT::InputPort<double>("max_rate", 0.6, "Largest turn rate [rad/s]"),
      BT::InputPort<double>("min_rate", 0.45, "Smallest turn rate, above the turning floor [rad/s]"),
      BT::InputPort<double>("timeout", 8.0, "Give up after this long [s]")
    };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Time t0_;
};

// ==============================================================================
// DriveForwardClear (Action Node)  -- darpa_sim_spark addition
// ------------------------------------------------------------------------------
// Purpose: Last step of the general escape: drive straight ahead out of the trouble spot after
//          the turn, without MPPI (which cannot move while the rover is in an inflated zone).
// Logic:   Waits settle_time so the forward lidar can mark what is now ahead, then publishes
//          cmd_vel itself (like Nav2's behaviors) at speed. Stops on any lethal cell in a corridor
//          ahead of the front edge. SUCCESS after min_distance once the centre is out of the
//          inflated zone (cost below clear_cost), and at least casualty_clear from a casualty it
//          started near; also SUCCESS at max_distance. Near a casualty it refuses to start unless
//          facing within 60 deg of directly away.
// ==============================================================================
class DriveForwardClear : public BT::StatefulActionNode
{
public:
  DriveForwardClear(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("casualties", "Every casualty"),
      BT::InputPort<double>("settle_time", 1.0, "Wait before driving, for the lidar to see ahead [s]"),
      BT::InputPort<double>("speed", 0.25, "Forward speed [m/s]"),
      BT::InputPort<double>("min_distance", 0.3, "Drive at least this far [m]"),
      BT::InputPort<double>("max_distance", 1.2, "Drive at most this far [m]"),
      BT::InputPort<int>("clear_cost", 128, "Done once the cost around the centre is below this"),
      BT::InputPort<double>("casualty_near", 3.0, "A casualty centre this close counts as near [m]"),
      BT::InputPort<double>("casualty_clear", 3.5, "When started near a casualty, drive until this far from it [m]"),
      BT::InputPort<double>("half_length", 0.38, "Rover centre to front edge [m]"),
      BT::InputPort<double>("half_width", 0.35, "Rover centre to side [m]"),
      BT::InputPort<double>("lookahead", 0.5, "Obstacle check distance ahead of the front edge [m]"),
      BT::InputPort<double>("timeout", 8.0, "Give up after this long [s]")
    };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void stop();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<LocalCostmapView> view_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  double x0_{0.0}, y0_{0.0}, cx_{0.0}, cy_{0.0};
  bool near_casualty_{false};
  rclcpp::Time t0_;
};

}  // namespace nav2_custom_bt_nodes

#endif  // NAV2_CUSTOM_BT_NODES_ESCAPE_NODES_HPP_
