#include "nav2_custom_bt_nodes/casualty_bt_nodes.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <chrono>

namespace nav2_custom_bt_nodes
{

// ==============================================================================
// PopNextGoal Implementation
// ==============================================================================
BT::NodeStatus PopNextGoal::tick()
{
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  if (!getInput("input_goals", goals)) return BT::NodeStatus::FAILURE;
  
  if (goals.empty()) return BT::NodeStatus::FAILURE; 

  geometry_msgs::msg::PoseStamped current_goal = goals.front();
  goals.erase(goals.begin());

  setOutput("output_goal", current_goal);
  setOutput("output_goals", goals);
  
  return BT::NodeStatus::SUCCESS;
}

// ==============================================================================
// CalculateApproachPose Implementation
// ==============================================================================
BT::NodeStatus CalculateApproachPose::tick()
{
  geometry_msgs::msg::PoseStamped center;
  if (!getInput("center", center)) return BT::NodeStatus::FAILURE;

  double radius = 3.5;
  getInput("radius", radius);

  auto tf_buffer = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  
  geometry_msgs::msg::PoseStamped current_pose;
  try {
    auto transform = tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
    current_pose.pose.position.x = transform.transform.translation.x;
    current_pose.pose.position.y = transform.transform.translation.y;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(node->get_logger(), "TF Error in CalculateApproachPose: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  double angle_to_robot = std::atan2(
    current_pose.pose.position.y - center.pose.position.y,
    current_pose.pose.position.x - center.pose.position.x
  );

  geometry_msgs::msg::PoseStamped approach_pose;
  approach_pose.header = center.header; 
  
  approach_pose.pose.position.x = center.pose.position.x + radius * std::cos(angle_to_robot);
  approach_pose.pose.position.y = center.pose.position.y + radius * std::sin(angle_to_robot);
  approach_pose.pose.position.z = center.pose.position.z;

  tf2::Quaternion q;
  q.setRPY(0, 0, angle_to_robot + M_PI);
  approach_pose.pose.orientation.x = q.x();
  approach_pose.pose.orientation.y = q.y();
  approach_pose.pose.orientation.z = q.z();
  approach_pose.pose.orientation.w = q.w();

  setOutput("output_pose", approach_pose);
  return BT::NodeStatus::SUCCESS;
}

// ==============================================================================
// GenerateOrbitPath Implementation
// ==============================================================================
BT::NodeStatus GenerateOrbitPath::tick()
{
  geometry_msgs::msg::PoseStamped center;
  if (!getInput("center", center)) return BT::NodeStatus::FAILURE;

  double radius = 3.5;
  getInput("radius", radius);
  
  auto tf_buffer = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  
  geometry_msgs::msg::PoseStamped current_pose;
  try {
    auto transform = tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
    current_pose.pose.position.x = transform.transform.translation.x;
    current_pose.pose.position.y = transform.transform.translation.y;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(node->get_logger(), "TF Error in GenerateOrbitPath: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  double start_angle = std::atan2(
    current_pose.pose.position.y - center.pose.position.y,
    current_pose.pose.position.x - center.pose.position.x
  );

  nav_msgs::msg::Path path;
  path.header = center.header;
  double resolution = 0.05; 
  
  double sweep_limit = start_angle + (345.0 * M_PI / 180.0);

  for (double t = start_angle; t <= sweep_limit; t += resolution) {
    geometry_msgs::msg::PoseStamped p;
    p.header = center.header;
    p.pose.position.x = center.pose.position.x + radius * std::cos(t);
    p.pose.position.y = center.pose.position.y + radius * std::sin(t);
    p.pose.position.z = center.pose.position.z;

    tf2::Quaternion q;
    q.setRPY(0, 0, t + M_PI_2); 
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();

    path.poses.push_back(p);
  }
  
  setOutput("output_path", path);
  return BT::NodeStatus::SUCCESS;
}

// ==============================================================================
// IsGoalCloserThan Implementation
// ==============================================================================
BT::NodeStatus IsGoalCloserThan::tick()
{
  double threshold;
  if (!getInput("distance", threshold)) return BT::NodeStatus::FAILURE;

  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("goal", goal)) return BT::NodeStatus::FAILURE;

  auto tf_buffer = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  
  geometry_msgs::msg::PoseStamped current_pose;
  try {
    auto transform = tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
    current_pose.pose.position.x = transform.transform.translation.x;
    current_pose.pose.position.y = transform.transform.translation.y;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(node->get_logger(), "TF Error in IsGoalCloserThan: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  double dx = current_pose.pose.position.x - goal.pose.position.x;
  double dy = current_pose.pose.position.y - goal.pose.position.y;
  double dist = std::sqrt(dx*dx + dy*dy);

  if (dist < threshold) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

// ==============================================================================
// GenerateOrbitWaypoints Implementation
// ==============================================================================
BT::NodeStatus GenerateOrbitWaypoints::tick()
{
  geometry_msgs::msg::PoseStamped center;
  if (!getInput("center", center)) return BT::NodeStatus::FAILURE;

  double radius = 3.5;
  getInput("radius", radius);

  int num_waypoints = 8;
  getInput("num_waypoints", num_waypoints);

  auto tf_buffer = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  
  geometry_msgs::msg::PoseStamped current_pose;
  try {
    auto transform = tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
    current_pose.pose.position.x = transform.transform.translation.x;
    current_pose.pose.position.y = transform.transform.translation.y;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(node->get_logger(), "TF Error in GenerateOrbitWaypoints: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  double start_angle = std::atan2(
    current_pose.pose.position.y - center.pose.position.y, 
    current_pose.pose.position.x - center.pose.position.x
  );

  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  double angle_step = (2.0 * M_PI) / num_waypoints;

  for (int i = 0; i < num_waypoints; ++i) {
    double angle = start_angle + (i * angle_step);
    
    geometry_msgs::msg::PoseStamped p;
    p.header = center.header; 
    
    p.pose.position.x = center.pose.position.x + radius * std::cos(angle);
    p.pose.position.y = center.pose.position.y + radius * std::sin(angle);
    p.pose.position.z = center.pose.position.z;

    double yaw = std::atan2(center.pose.position.y - p.pose.position.y, 
                            center.pose.position.x - p.pose.position.x);
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();

    waypoints.push_back(p);
  }

  setOutput("output_goals", waypoints);
  return BT::NodeStatus::SUCCESS;
}

// ==============================================================================
// DynamicWaitAction Implementation (THE LOUD FIX)
// ==============================================================================
DynamicWaitAction::DynamicWaitAction(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config)
{
  config.blackboard->get("node", node_);

  sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    "/wait_trigger", 
    10, 
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      if (msg->data > 0.0) {
        // MASSIVE TERMINAL LOG: Proves the network message arrived!
        RCLCPP_WARN(node_->get_logger(), "\n\n🛑 [EMERGENCY WAIT] COMMAND RECEIVED: %f seconds. HALTING ROBOT!\n", msg->data);
        
        wait_time_ = msg->data;
        start_time_ = std::chrono::steady_clock::now();
        trigger_active_.store(true);
      } else {
        RCLCPP_WARN(node_->get_logger(), "\n\n▶️ [EMERGENCY WAIT] CANCELED! Resuming mission.\n");
        trigger_active_.store(false);
      }
    }
  );
}

BT::NodeStatus DynamicWaitAction::tick()
{
  if (!trigger_active_.load()) {
    return BT::NodeStatus::FAILURE;
  }
  
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_).count();
  
  if (elapsed >= wait_time_) {
    RCLCPP_WARN(node_->get_logger(), "\n\n✅ [EMERGENCY WAIT] FINISHED! Resuming mission.\n");
    trigger_active_.store(false);
    return BT::NodeStatus::FAILURE;
  }
  
  // Print a small tick every 100hz so you visually see it holding the tree
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                       "⏳ [EMERGENCY WAIT] Holding... %.1f / %.1f seconds", elapsed, wait_time_);

  return BT::NodeStatus::RUNNING;
}

void DynamicWaitAction::halt()
{
  trigger_active_.store(false);
  setStatus(BT::NodeStatus::IDLE);
}

} // namespace nav2_custom_bt_nodes

// ==============================================================================
// REGISTRATION MACRO
// ==============================================================================
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_custom_bt_nodes::PopNextGoal>("PopNextGoal");
  factory.registerNodeType<nav2_custom_bt_nodes::CalculateApproachPose>("CalculateApproachPose");
  factory.registerNodeType<nav2_custom_bt_nodes::GenerateOrbitPath>("GenerateOrbitPath");
  factory.registerNodeType<nav2_custom_bt_nodes::IsGoalCloserThan>("IsGoalCloserThan");
  factory.registerNodeType<nav2_custom_bt_nodes::GenerateOrbitWaypoints>("GenerateOrbitWaypoints");
  factory.registerNodeType<nav2_custom_bt_nodes::DynamicWaitAction>("DynamicWaitAction");
}

