/**
 * =================================================================================================
 * UAS-DTU Ground Segmentation & Terrain-Aware LaserScan Projection Implementation
 * Author: Vortex
 *
 * Architecture Summary:
 *   1. Ingests raw 3D PointCloud2 (e.g. from Blickfeld 3D LiDAR).
 *   2. Transforms cloud into target_frame_ ('base_link') via TF2.
 *   3. Queries TF / Odometry with a staleness watchdog to extract current chassis pitch and roll.
 *   4. Constructs gravity-aligned, yaw-free level frame: R_level = RPY(roll, pitch, 0).
 *   5. Discretizes points into polar azimuth rays (in base_link frame) and radial distance bins.
 *   6. Along each ray in the flattened 1D bin grid:
 *        - Evaluates consecutive height steps (Delta_Z_local) and slope gradients.
 *        - Classifies smooth ramps (<= 30 deg slope and small steps) as TRAVERSABLE GROUND (discarded).
 *        - Classifies steep inclines (> 30 deg slope), tall steps (> 12 cm), and ditches (<-18 cm)
 *          as TRUE OBSTACLES (retained).
 *   7. Applies ceiling cutoff (level-frame pz > max_height discarded) to prevent overhead tree branches
 *      from being squished into the 2D plane.
 *   8. Squishes the remaining true obstacle points into sensor_msgs::msg::LaserScan ('scan').
 * =================================================================================================
 */

#include "ground_segmentation/ground_segmentation_node.hpp"

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <unordered_map>
#include <map>
#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "tf2/utils.h"
#include <Eigen/Dense>

namespace ground_segmentation
{

GroundSegmentationNode::GroundSegmentationNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("ground_segmentation", options)
{
  // -----------------------------------------------------------------------------------------------
  // 1. Declare standard pointcloud_to_laserscan parameters (100% backward compatible)
  // -----------------------------------------------------------------------------------------------
  target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");
  tolerance_ = this->declare_parameter<double>("transform_tolerance", 0.05);
  input_queue_size_ = this->declare_parameter<int>("queue_size", 10);

  min_height_ = this->declare_parameter<double>("min_height", -0.8);
  max_height_ = this->declare_parameter<double>("max_height", 1.2);

  angle_min_ = this->declare_parameter<double>("angle_min", -M_PI);
  angle_max_ = this->declare_parameter<double>("angle_max", M_PI);
  angle_increment_ = this->declare_parameter<double>("angle_increment", M_PI / 180.0);
  scan_time_ = this->declare_parameter<double>("scan_time", 1.0 / 30.0);
  range_min_ = this->declare_parameter<double>("range_min", 0.0);
  range_max_ = this->declare_parameter<double>("range_max", 50.0);
  inf_epsilon_ = this->declare_parameter<double>("inf_epsilon", 1.0);
  use_inf_ = this->declare_parameter<bool>("use_inf", true);

  // 3D Voxel Outlier Density Filter (Preserved from Vortex's fork)
  enable_3d_point_filter_ = this->declare_parameter<bool>("enable_3d_point_filter", true);
  filter_voxel_size_ = this->declare_parameter<double>("filter_voxel_size", 0.15);
  min_points_per_voxel_ = this->declare_parameter<int>("min_points_per_voxel", 2);

  // -----------------------------------------------------------------------------------------------
  // 2. Declare [NEW] Ground Segmentation & Dynamic Slope Filtering Parameters
  // -----------------------------------------------------------------------------------------------
  enable_ground_filtering_ = this->declare_parameter<bool>("enable_ground_filtering", true);
  gravity_frame_ = this->declare_parameter<std::string>("gravity_frame", "map");

  max_slope_angle_deg_ = this->declare_parameter<double>("max_slope_angle_deg", 30.0);
  if (max_slope_angle_deg_ <= 0.0 || max_slope_angle_deg_ >= 85.0) {
    RCLCPP_WARN(this->get_logger(),
      "Configured max_slope_angle_deg (%.1f) is out of bounds (0, 85 deg); resetting to 30.0 deg.",
      max_slope_angle_deg_);
    max_slope_angle_deg_ = 30.0;
  }
  max_slope_angle_rad_ = max_slope_angle_deg_ * (M_PI / 180.0);

  max_step_height_ = this->declare_parameter<double>("max_step_height", 0.12);
  if (max_step_height_ <= 0.0) {
    max_step_height_ = 0.12;
  }
  max_drop_height_ = this->declare_parameter<double>("max_drop_height", 0.18);
  if (max_drop_height_ <= 0.0) {
    max_drop_height_ = 0.18;
  }
  enable_drop_detection_ = this->declare_parameter<bool>("enable_drop_detection", true);
  max_drop_range_ = this->declare_parameter<double>("max_drop_range", 4.0);
  if (max_drop_range_ <= 0.0) {
    max_drop_range_ = 4.0;
  }

  expected_ground_z_ = this->declare_parameter<double>("expected_ground_z", 0.0);
  ground_start_clearance_ = this->declare_parameter<double>("ground_start_clearance", 0.15);
  anchor_near_range_ = this->declare_parameter<double>("anchor_near_range", 0.50);
  anchor_max_reach_ = this->declare_parameter<double>("anchor_max_reach", 1.20);
  plane_anchor_max_range_ = this->declare_parameter<double>("plane_anchor_max_range", 3.0);
  gap_slope_factor_ = this->declare_parameter<double>("gap_slope_factor", 0.5);
  radial_bin_size_ = this->declare_parameter<double>("radial_bin_size", 0.10);
  max_gap_distance_ = this->declare_parameter<double>("max_gap_distance", 0.40);
  z_noise_ = this->declare_parameter<double>("z_noise", 0.03);
  if (z_noise_ < 0.0) {
    z_noise_ = 0.03;
  }

  // Guard against invalid angle parameters or division by zero
  if (angle_increment_ <= 0.0) {
    angle_increment_ = M_PI / 180.0;
  }
  if (angle_max_ <= angle_min_) {
    angle_min_ = -M_PI;
    angle_max_ = M_PI;
  }
  if (radial_bin_size_ <= 0.0) {
    radial_bin_size_ = 0.10;
  }
  if (max_gap_distance_ <= 0.0) {
    max_gap_distance_ = 0.40;
  }
  if (anchor_near_range_ < 0.0) {
    anchor_near_range_ = 0.50;
  }
  if (anchor_max_reach_ < 0.0) {
    anchor_max_reach_ = 1.20;
  }
  if (plane_anchor_max_range_ <= 0.0) {
    plane_anchor_max_range_ = 3.0;
  }
  if (gap_slope_factor_ < 0.0 || gap_slope_factor_ > 1.0) {
    gap_slope_factor_ = 0.5;
  }

  // Pre-allocate flattened 1D terrain bin grid (O(1) lookups, zero allocations in loop)
  num_rays_ = std::ceil((angle_max_ - angle_min_) / angle_increment_);
  if (range_max_ <= 0.0 || range_max_ > 100.0) {
    RCLCPP_WARN(this->get_logger(),
      "Configured range_max (%.1fm) is out of bounds (0, 100m]; clamping to 50.0m.", range_max_);
    range_max_ = 50.0;
  }
  double max_ground_r = range_max_;
  num_bins_ = static_cast<int>(std::ceil(max_ground_r / std::max(0.01, radial_bin_size_))) + 1;
  radial_ray_bins_.resize(num_rays_ * num_bins_);
  all_valid_points_.reserve(30000);

  debug_publish_clouds_ = this->declare_parameter<bool>("debug_publish_clouds", false);

  // [NEW] Direct Odometry Topic Subscription (Bypasses TF lookup delays entirely)
  use_odom_topic_ = this->declare_parameter<bool>("use_odom_topic", true);
  odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/mavros/local_position/odom");

  // [NEW] 2D LiDAR Slope-Filtering Parameters (SF45/B Close-Range Blind Zone Assist)
  enable_2d_lidar_filtering_ = this->declare_parameter<bool>("enable_2d_lidar_filtering", true);
  scan_2d_in_topic_ = this->declare_parameter<std::string>("scan_2d_in_topic", "/lightwarelidar/scan");
  scan_2d_out_topic_ = this->declare_parameter<std::string>("scan_2d_out_topic", "/scan_2d_filtered");
  scan_2d_max_range_ = this->declare_parameter<double>("scan_2d_max_range", 50.0);
  ramp_hit_tolerance_ = this->declare_parameter<double>("ramp_hit_tolerance", 0.08);
  ramp_hit_tolerance_ = std::min(ramp_hit_tolerance_, std::max(0.04, max_step_height_ - 0.02));
  terrain_model_timeout_ = this->declare_parameter<double>("terrain_model_timeout", 0.25);
  if (terrain_model_timeout_ <= 0.0) {
    terrain_model_timeout_ = 0.25;
  }

  // -----------------------------------------------------------------------------------------------
  // 3. Initialize Publishers & Subscriptions
  // -----------------------------------------------------------------------------------------------
  scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());

  if (enable_2d_lidar_filtering_) {
    scan_2d_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      scan_2d_out_topic_, rclcpp::SensorDataQoS());

    scan_2d_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_2d_in_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        this->scan2dCallback(msg);
      });

    RCLCPP_INFO(this->get_logger(),
      "2D LiDAR Slope Filtering enabled: in='%s' -> out='%s' (MaxRange: %.1fm, Tol: %.2fm). Dynamic TF lookup active.",
      scan_2d_in_topic_.c_str(), scan_2d_out_topic_.c_str(),
      scan_2d_max_range_, ramp_hit_tolerance_);

    if (!enable_ground_filtering_) {
      RCLCPP_WARN(this->get_logger(),
        "enable_2d_lidar_filtering is true but enable_ground_filtering is false. Terrain models cannot be generated; 2D scan filtering will be inactive.");
    }
  }

  RCLCPP_INFO(this->get_logger(),
    "Expected ground Z: %.2fm relative to '%s' (set to negative chassis height if base_link is above ground).",
    expected_ground_z_, target_frame_.c_str());

  if (debug_publish_clouds_) {
    debug_ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "debug/ground_cloud", rclcpp::SensorDataQoS());
    debug_obstacle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "debug/obstacle_cloud", rclcpp::SensorDataQoS());
  }

  // Setup TF Buffer and TransformListener
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

  rclcpp::SensorDataQoS sub_qos;
  sub_qos.keep_last(input_queue_size_);

  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_in", sub_qos,
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      this->cloudCallback(msg);
    });

  // Direct Odometry Topic Subscription:
  // Subscribes using SensorDataQoS (matches MAVROS BEST_EFFORT) to completely bypass TF delays
  if (use_odom_topic_) {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        this->odomCallback(msg);
      });
    RCLCPP_INFO(this->get_logger(),
      "Direct Odometry Subscription enabled on '%s' (QoS: SensorData / Best Effort).",
      odom_topic_.c_str());
  }

  RCLCPP_INFO(this->get_logger(),
    "GroundSegmentationNode initialized successfully (Max Slope: %.1f deg, Step: %.2fm, Ceiling: %.2fm).",
    max_slope_angle_deg_, max_step_height_, max_height_);
}

GroundSegmentationNode::~GroundSegmentationNode() {}

void GroundSegmentationNode::odomCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);

  odom_stamp_ = odom_msg->header.stamp;
  odom_x_ = odom_msg->pose.pose.position.x;
  odom_y_ = odom_msg->pose.pose.position.y;
  odom_z_ = odom_msg->pose.pose.position.z;

  // Extract Euler RPY from orientation quaternion
  tf2::Quaternion q(
    odom_msg->pose.pose.orientation.x,
    odom_msg->pose.pose.orientation.y,
    odom_msg->pose.pose.orientation.z,
    odom_msg->pose.pose.orientation.w);

  tf2::Matrix3x3(q).getRPY(odom_roll_, odom_pitch_, odom_yaw_);
  odom_received_ = true;
}

bool GroundSegmentationNode::getTilt(const rclcpp::Time & stamp, double & roll, double & pitch)
{
  if (use_odom_topic_) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (odom_received_) {
      double odom_age = std::abs((stamp - odom_stamp_).seconds());
      if (odom_age <= 0.3) {
        roll = odom_roll_;
        pitch = odom_pitch_;
        return true;
      }
    }
  }

  if (!gravity_frame_.empty()) {
    try {
      geometry_msgs::msg::TransformStamped tf_stamped;
      try {
        tf_stamped = tf2_buffer_->lookupTransform(
          gravity_frame_, target_frame_, stamp, tf2::durationFromSec(tolerance_));
      } catch (const tf2::TransformException &) {
        tf_stamped = tf2_buffer_->lookupTransform(
          gravity_frame_, target_frame_, tf2::TimePointZero, tf2::durationFromSec(tolerance_));
        double tf_age = std::abs((stamp - tf_stamped.header.stamp).seconds());
        if (tf_age > 0.4) {
          return false;
        }
      }

      tf2::Quaternion q(
        tf_stamped.transform.rotation.x,
        tf_stamped.transform.rotation.y,
        tf_stamped.transform.rotation.z,
        tf_stamped.transform.rotation.w);

      double yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "getTilt: TF lookup failed from '" << gravity_frame_ << "' to '" << target_frame_
        << "': " << ex.what());
    }
  }

  return false;
}

void GroundSegmentationNode::cloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg)
{
  // -----------------------------------------------------------------------------------------------
  // Step 1: Pre-allocate and initialize the LaserScan message
  // -----------------------------------------------------------------------------------------------
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = cloud_msg->header;
  if (!target_frame_.empty()) {
    scan_msg->header.frame_id = target_frame_;
  }

  scan_msg->angle_min = angle_min_;
  scan_msg->angle_max = angle_max_;
  scan_msg->angle_increment = angle_increment_;
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;

  uint32_t ranges_size = std::ceil(
    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

  if (use_inf_) {
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  } else {
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
  }

  // -----------------------------------------------------------------------------------------------
  // Step 2: Transform incoming point cloud to target_frame_ ('base_link')
  // -----------------------------------------------------------------------------------------------
  sensor_msgs::msg::PointCloud2::ConstSharedPtr processed_cloud = cloud_msg;
  if (!target_frame_.empty() && scan_msg->header.frame_id != cloud_msg->header.frame_id) {
    try {
      auto transformed_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      tf2_buffer_->transform(*cloud_msg, *transformed_cloud, target_frame_, tf2::durationFromSec(tolerance_));
      processed_cloud = transformed_cloud;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF transform failure from " << cloud_msg->header.frame_id << " to " << target_frame_
        << ": " << ex.what());
      return; // Safely skip this single frame without blocking the pipeline
    }
  }

  // -----------------------------------------------------------------------------------------------
  // Step 3: Extract current chassis tilt (Pitch & Roll) and construct Gravity-Level Frame
  // -----------------------------------------------------------------------------------------------
  bool attitude_valid = true;
  double current_roll = 0.0;
  double current_pitch = 0.0;
  if (enable_ground_filtering_) {
    if (!getTilt(processed_cloud->header.stamp, current_roll, current_pitch)) {
      attitude_valid = false;
      current_roll = 0.0;
      current_pitch = 0.0;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Chassis tilt unavailable from TF/Odometry! Fail-safe: bypassing ground filtering to prevent false clearing.");
    }
  }

  // Fail-safe: if attitude is unavailable, do NOT run ground filter (prevents false terrain clearing)
  const bool run_ground_filter = enable_ground_filtering_ && attitude_valid;
  if (!run_ground_filter && enable_2d_lidar_filtering_) {
    std::lock_guard<std::mutex> lock(terrain_mutex_);
    latest_terrain_.valid = false;
  }

  // Rotation matrix from target_frame ('base_link') to gravity-aligned, yaw-free level frame
  tf2::Matrix3x3 R_level;
  R_level.setRPY(current_roll, current_pitch, 0.0);
  const double tan_max = std::tan(max_slope_angle_rad_);

  // -----------------------------------------------------------------------------------------------
  // Step 4 (Optional): 3D Spatial Voxel Outlier Filter (Preserved from Vortex's fork)
  // -----------------------------------------------------------------------------------------------
  const int64_t VOXEL_BIAS = 1048576; // 2^20 offset prevents negative coordinate two's-complement hash collisions
  if (enable_3d_point_filter_) {
    voxel_point_counts_.clear();
    const double inv_voxel_size = 1.0 / std::max(0.01, filter_voxel_size_);
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*processed_cloud, "x"),
      iter_y(*processed_cloud, "y"), iter_z(*processed_cloud, "z");
      iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) continue;
      // Ceiling and floor cutoff: when attitude is valid, evaluate in gravity level frame.
      // NOTE: Relaxed (+4.0m) to preserve sensing horizon on ramps/hills; local ceiling is enforced in Step 7.
      if (attitude_valid) {
        tf2::Vector3 pl = R_level * tf2::Vector3(*iter_x, *iter_y, *iter_z);
        if (pl.z() > (max_height_ + 4.0) || pl.z() < (min_height_ - 2.0)) continue;
      } else {
        if (*iter_z > (max_height_ + 2.0) || *iter_z < (min_height_ - 2.0)) continue;
      }

      int64_t gx = static_cast<int64_t>(std::floor(*iter_x * inv_voxel_size)) + VOXEL_BIAS;
      int64_t gy = static_cast<int64_t>(std::floor(*iter_y * inv_voxel_size)) + VOXEL_BIAS;
      int64_t gz = static_cast<int64_t>(std::floor(*iter_z * inv_voxel_size)) + VOXEL_BIAS;

      uint64_t key = (static_cast<uint64_t>(gx & 0x1FFFFF) << 42) |
                     (static_cast<uint64_t>(gy & 0x1FFFFF) << 21) |
                     (static_cast<uint64_t>(gz & 0x1FFFFF));
      voxel_point_counts_[key]++;
    }
  }

  // -----------------------------------------------------------------------------------------------
  // Step 5: Structure 3D Points into Polar Azimuth Rays & Radial Distance Bins (in Base Frame)
  //
  // CRITICAL: range and angle MUST be computed in base_link coordinates to avoid double-rotation
  // in downstream Nav2 costmap. Level-frame Z (pz) is preserved for height/slope terrain checks.
  // -----------------------------------------------------------------------------------------------
  if (num_rays_ <= 0 || num_bins_ <= 0) {
    num_rays_ = std::max(1, static_cast<int>(std::ceil((angle_max_ - angle_min_) / angle_increment_)));
    double max_ground_r = range_max_;
    num_bins_ = static_cast<int>(std::ceil(max_ground_r / std::max(0.01, radial_bin_size_))) + 1;
  }
  if (radial_ray_bins_.size() != static_cast<size_t>(num_rays_ * num_bins_)) {
    radial_ray_bins_.resize(num_rays_ * num_bins_);
  }
  std::fill(radial_ray_bins_.begin(), radial_ray_bins_.end(), BinData{});
  all_valid_points_.clear();

  const double inv_bin_size = 1.0 / std::max(0.01, radial_bin_size_);

  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*processed_cloud, "x"),
    iter_y(*processed_cloud, "y"), iter_z(*processed_cloud, "z");
    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) continue;

    // 1. Calculate base_link polar coordinates for final LaserScan projection
    double range_base = std::hypot(*iter_x, *iter_y);
    if (range_base < range_min_ || range_base > range_max_) continue;

    double angle_base = std::atan2(*iter_y, *iter_x);
    if (angle_base < scan_msg->angle_min || angle_base > scan_msg->angle_max) continue;

    int ray_idx = static_cast<int>(std::floor((angle_base - scan_msg->angle_min) / scan_msg->angle_increment));
    if (ray_idx < 0 || ray_idx >= static_cast<int>(ranges_size)) continue;

    // 2. Transform point into gravity-aligned level frame ONLY for height and slope evaluation
    tf2::Vector3 pl = R_level * tf2::Vector3(*iter_x, *iter_y, *iter_z);
    double pz = pl.z();

    // CEILING & FLOOR CUTOFF:
    // When attitude is valid, evaluate against gravity-level frame.
    // NOTE: To prevent collapsing the sensing horizon when climbing ramps/hills (e.g. 20-30 deg),
    // global bounding here is relaxed (+4.0m) so elevated ground/obstacles on slopes are retained.
    // The exact obstacle ceiling (max_height_ above local ground surface) is enforced in Step 7!
    if (attitude_valid) {
      if (pz > (max_height_ + 4.0) || pz < (min_height_ - 2.0)) continue;
    } else {
      if (*iter_z > (max_height_ + 2.0) || *iter_z < (min_height_ - 2.0)) continue;
    }

    // 3D Voxel Outlier check: Skip isolated points with insufficient neighbors
    if (enable_3d_point_filter_) {
      const double inv_voxel_size = 1.0 / std::max(0.01, filter_voxel_size_);
      int64_t gx = static_cast<int64_t>(std::floor(*iter_x * inv_voxel_size)) + VOXEL_BIAS;
      int64_t gy = static_cast<int64_t>(std::floor(*iter_y * inv_voxel_size)) + VOXEL_BIAS;
      int64_t gz = static_cast<int64_t>(std::floor(*iter_z * inv_voxel_size)) + VOXEL_BIAS;

      uint64_t key = (static_cast<uint64_t>(gx & 0x1FFFFF) << 42) |
                     (static_cast<uint64_t>(gy & 0x1FFFFF) << 21) |
                     (static_cast<uint64_t>(gz & 0x1FFFFF));

      if (voxel_point_counts_[key] < min_points_per_voxel_) {
        continue;
      }
    }

    double range_level = std::hypot(pl.x(), pl.y());
    double angle_level = std::atan2(pl.y(), pl.x());

    int terrain_ray_idx = static_cast<int>(std::floor((angle_level - scan_msg->angle_min) / scan_msg->angle_increment));
    if (scan_msg->angle_max - scan_msg->angle_min >= 2.0 * M_PI - 0.05) {
      terrain_ray_idx = (terrain_ray_idx % num_rays_ + num_rays_) % num_rays_;
    }
    int terrain_r_bin = static_cast<int>(std::floor(range_level * inv_bin_size));

    Point3D pt;
    pt.x = *iter_x;
    pt.y = *iter_y;
    pt.raw_z = *iter_z;
    pt.range = range_base;
    pt.ray_idx = ray_idx;
    pt.lx = static_cast<float>(pl.x());
    pt.ly = static_cast<float>(pl.y());
    pt.z = static_cast<float>(pz);
    pt.range_level = range_level;
    pt.terrain_ray_idx = terrain_ray_idx;
    pt.terrain_r_bin = terrain_r_bin;
    all_valid_points_.push_back(pt);

    if (run_ground_filter) {
      if (terrain_ray_idx >= 0 && terrain_ray_idx < num_rays_ &&
          terrain_r_bin >= 0 && terrain_r_bin < num_bins_)
      {
        int flat_idx = terrain_ray_idx * num_bins_ + terrain_r_bin;
        auto & bin = radial_ray_bins_[flat_idx];
        if (!bin.valid || pt.z < bin.pt.z) {
          bin.valid = true;
          bin.is_obstacle = false;
          bin.pt = pt; // Store lowest point in level-frame bin to capture ground contact
        }
      }
    }
  }

  // -----------------------------------------------------------------------------------------------
  // Step 6: Ground Segmentation & Terrain Classification along each Ray (in Level Frame)
  //
  // In the gravity-aligned level frame, allowable slope is uniformly tan(max_slope_angle_rad_)
  // in ALL directions (no cosine/sine blending or directional pitch swapping needed).
  // -----------------------------------------------------------------------------------------------
  if (run_ground_filter) {
    const double ray_up_budget = tan_max;
    const double ray_down_budget = tan_max;
    const double step_dr = max_step_height_ / tan_max;
    const double drop_dr = max_drop_height_ / tan_max;

    int rays_with_points = 0;
    int anchor_failed_rays = 0;

    // Anchor reachability:
    // (A) flat_ok: checks level-frame z with near blind-zone slope cone (<= 1.2m reach, smoothly tapering over 0.5m).
    // (B) plane_ok: checks raw_z in base_link against expected_ground_z_. When the rover is climbing a ramp,
    //     the terrain ahead is coplanar with the rover wheels in base_link (even though it rises in level frame).
    auto anchorOk = [&](const Point3D & pt) {
      const double reach = pt.range_level - anchor_near_range_;
      const double fade  = std::clamp(1.0 - (reach - anchor_max_reach_) / 0.5, 0.0, 1.0);
      const double cone  = tan_max * std::clamp(reach, 0.0, anchor_max_reach_) * fade;
      const bool flat_ok  = std::abs(pt.z - expected_ground_z_) <= (ground_start_clearance_ + cone);
      // Plane anchor is strictly for establishing ground contact in the near wheel zone when on a ramp.
      // Capped by plane_anchor_max_range_ to prevent far-field elevated/overhung obstacles from masquerading as ground
      // due to pitch lever-arm effects in base_link.
      const bool plane_ok = (pt.range_level <= plane_anchor_max_range_) &&
                            (std::abs(pt.raw_z - expected_ground_z_) <= ground_start_clearance_);
      // With drop detection off, a first return below ground past the blind zone is assumed to be a
      // descending ramp: accept it if it lies inside the legal downward slope cone (no taper).
      const double dzg = pt.z - expected_ground_z_;
      const bool down_ok = !enable_drop_detection_ &&
                           dzg <= ground_start_clearance_ &&
                           dzg >= -(ground_start_clearance_ + tan_max * std::max(0.0, reach));
      return flat_ok || plane_ok || down_ok;
    };

    for (int ray_idx = 0; ray_idx < num_rays_; ++ray_idx) {
      const int ray_offset = ray_idx * num_bins_;

      // Find the first populated bin along this ray
      int first_bin = -1;
      for (int r_bin = 0; r_bin < num_bins_; ++r_bin) {
        if (radial_ray_bins_[ray_offset + r_bin].valid) {
          first_bin = r_bin;
          break;
        }
      }
      if (first_bin < 0) continue; // No points on this ray
      rays_with_points++;

      auto & first = radial_ray_bins_[ray_offset + first_bin];
      // Ground anchor check: First surface point must start near ground or in wheel plane.
      // NOTE: Do NOT re-introduce single-ray ramp-foot extrapolation here. A fitted
      // line along one ray is indistinguishable from an inclined obstacle face
      // (angled vehicle hood, wedged barrier), which would be fatal to clear.
      // Occluded ramp feet are deliberately treated as obstacles until the rover
      // closes to anchor range. See design review, v6->v7.
      const bool anchor_ok = anchorOk(first.pt);
      first.is_obstacle = !anchor_ok;
      first.ground_z = anchor_ok ? first.pt.z : expected_ground_z_;
      first.has_ground = anchor_ok;
      if (!anchor_ok) {
        anchor_failed_rays++;
      }
      BinData * ref_bin = anchor_ok ? &first : nullptr;      // last confirmed ground bin
      const Point3D * ref = anchor_ok ? &first.pt : nullptr; // last confirmed ground contact
      const Point3D * trail_ref = ref;                       // trailing ground reference for multi-bin baseline slope check
      const Point3D * last_drop = nullptr;

      for (int r_bin = first_bin + 1; r_bin < num_bins_; ++r_bin) {
        auto & curr_bin = radial_ray_bins_[ray_offset + r_bin];
        if (!curr_bin.valid) continue;

        const Point3D & p = curr_bin.pt;

        if (!ref) {
          // No ground anchor established yet; can this bin serve as the ground anchor within slope reach?
          if (anchorOk(p)) {
            curr_bin.is_obstacle = false;
            curr_bin.ground_z = p.z;
            curr_bin.has_ground = true;
            ref_bin = &curr_bin;
            ref = &p;
            trail_ref = &p;
          } else {
            curr_bin.is_obstacle = true;
            curr_bin.ground_z = expected_ground_z_;
            curr_bin.has_ground = false;
          }
          continue;
        }

        // Check if previous bin was a drop and this bin can establish a new lower ground surface
        if (last_drop != nullptr) {
          double dr_drop = p.range_level - last_drop->range_level;
          double dz_drop = p.z - last_drop->z;
          // If the lower surface is locally consistent and within slope bounds
          if (dr_drop > 0.0 && dr_drop <= max_gap_distance_ &&
              std::abs(dz_drop) <= max_step_height_ &&
              (dz_drop - z_noise_) <= ray_up_budget * dr_drop &&
              (dz_drop + z_noise_) >= -ray_down_budget * dr_drop)
          {
            curr_bin.is_obstacle = false;
            curr_bin.ground_z = p.z;
            curr_bin.has_ground = true;
            ref_bin = &curr_bin;
            ref = &p; // Re-anchor on the lower surface!
            trail_ref = &p;
            last_drop = nullptr;
            continue;
          }
          last_drop = nullptr;
        }

        double dr = p.range_level - ref->range_level;
        double dz = p.z - ref->z;

        // Gap Bridging: test reachability across unobserved intervals (shadows / occlusion)
        if (dr > max_gap_distance_) {
          const double dz_raw = p.raw_z - ref->raw_z; // height change relative to wheel plane

          // 1. Negative obstacle / drop-off / ledge check:
          // If drop detection is enabled, flag drops deeper than max_drop_height_ (0.18m)
          // within max_drop_range_ (<= 4.0m) in BOTH level frame and wheel plane (raw_z).
          // Beyond max_drop_range_, distant hill crest shadows are not flagged as lethal cliffs.
          if (enable_drop_detection_ &&
              (ref->range_level <= max_drop_range_) &&
              (dz < -max_drop_height_) &&
              (dz_raw < -max_drop_height_))
          {
            if (ref_bin != nullptr) {
              ref_bin->is_obstacle = true; // The upper ledge edge itself becomes the obstacle!
            }
            curr_bin.is_obstacle = true;
            curr_bin.ground_z = ref->z;
            curr_bin.has_ground = true;
            last_drop = &p; // Lower surface hit: allows subsequent consecutive bins to re-anchor
            continue;
          }

          // Slope Trend Continuation:
          // Look backward along this ray for a confirmed ground bin at least 0.6m behind ref.
          // Extrapolating the verified macroscopic slope trend allows sparse LiDAR rings or shadowed patches
          // on a legal up-ramp seen from level ground to remain classified as traversable ground.
          const Point3D * trend_base = nullptr;
          int ref_r_bin = (ref_bin != nullptr) ? static_cast<int>(ref_bin - &radial_ray_bins_[ray_offset]) : first_bin;
          for (int back_b = ref_r_bin - 1; back_b >= first_bin; --back_b) {
            const auto & b_back = radial_ray_bins_[ray_offset + back_b];
            if (b_back.valid && !b_back.is_obstacle) {
              double dr_base = ref->range_level - b_back.pt.range_level;
              if (dr_base >= 0.6) {
                trend_base = &b_back.pt;
                break;
              }
            }
          }

          bool trend_step = false;
          if (trend_base != nullptr) {
            double dr_trend = ref->range_level - trend_base->range_level;
            double slope_trend = std::clamp((ref->z - trend_base->z) / dr_trend, -tan_max, tan_max);
            double pred_dz = slope_trend * dr;
            if (std::abs(dz - pred_dz) <= (max_step_height_ + z_noise_)) {
              trend_step = true;
            }
          }

          // 2. Positive elevation jump across an unobserved gap:
          // A continuous gentle slope across a gap cannot jump upward by more than max_step_height_ + z_noise_
          // in either the level frame, relative to the wheel plane (raw_z), or relative to the confirmed slope trend.
          const bool within_slope = (dz - z_noise_) <= ray_up_budget * dr;
          const bool gentle_step  = (dz <= (max_step_height_ + z_noise_)) ||
                                    (dz_raw <= (max_step_height_ + z_noise_)) ||
                                    trend_step;
          const bool reachable    = within_slope && gentle_step;
          if (reachable) {
            curr_bin.is_obstacle = false;
            curr_bin.ground_z = p.z;
            curr_bin.has_ground = true;
            ref_bin = &curr_bin;
            ref = &p; // Advance reference
            if (trail_ref && (p.range_level - trail_ref->range_level >= 0.35)) {
              trail_ref = &p;
            }
          } else {
            // Not reachable from previous ref across gap: can it independently re-anchor on ground?
            if (anchorOk(p)) {
              curr_bin.is_obstacle = false;
              curr_bin.ground_z = p.z;
              curr_bin.has_ground = true;
              ref_bin = &curr_bin;
              ref = &p; // Re-anchor on ground!
              trail_ref = &p;
            } else {
              curr_bin.is_obstacle = true;
              curr_bin.ground_z = ref->z;
              curr_bin.has_ground = true;
            }
          }
          continue;
        }

        if (dr <= 0.01) {
          // Points at virtually same range but different height (vertical wall/curb face)
          bool is_wall = std::abs(dz) > max_step_height_;
          curr_bin.is_obstacle = is_wall;
          curr_bin.ground_z = is_wall ? ref->z : p.z;
          curr_bin.has_ground = true;
          if (!is_wall) {
            ref_bin = &curr_bin;
            ref = &p;
          }
          continue;
        }

        // Local step/drop checks: only valid at ranges where legal slope cannot produce dz!
        // Note: consecutive returns across a true vertical step are assumed <= step_dr apart.
        // Sparsely sampled returns (> step_dr) are governed by the slope budget (steep_uphill / steep_ditch).
        bool tall_step  = (dr <= step_dr) && (dz > max_step_height_);
        bool steep_drop = (dr <= drop_dr) && (dz < -max_drop_height_);

        // Multiplied slope form with z_noise floor (prevents division by tiny dr and LiDAR jitter)
        bool steep_uphill = (dz - z_noise_) > ray_up_budget * dr;
        bool steep_ditch  = steep_drop || ((dz + z_noise_) < -ray_down_budget * dr);

        // Multi-bin trailing slope check: prevents z_noise_ from accumulating into ~41 deg slopes
        // across consecutive 10cm bins (enforces true max_slope over >= 0.35m baseline)
        bool trail_too_steep = false;
        if (trail_ref != nullptr) {
          double dr_trail = p.range_level - trail_ref->range_level;
          if (dr_trail >= 0.35) {
            double dz_trail = p.z - trail_ref->z;
            if ((dz_trail - z_noise_) > ray_up_budget * dr_trail ||
                (dz_trail + z_noise_) < -ray_down_budget * dr_trail)
            {
              trail_too_steep = true;
            }
          }
        }

        if (steep_uphill || tall_step || steep_ditch || trail_too_steep) {
          curr_bin.is_obstacle = true;
          curr_bin.ground_z = ref->z;
          curr_bin.has_ground = true;
          if (steep_ditch) {
            last_drop = &p; // Record drop edge to test if subsequent bins form a lower plateau
          }
          // Retain ref: ground behind an obstacle must compare against last confirmed ground contact
        } else {
          curr_bin.is_obstacle = false;
          curr_bin.ground_z = p.z;
          curr_bin.has_ground = true;
          ref_bin = &curr_bin;
          ref = &p; // Advance confirmed ground reference
          if (trail_ref && (p.range_level - trail_ref->range_level >= 0.35)) {
            trail_ref = &p; // Advance trailing baseline reference
          }
        }
      }
    }

    if (rays_with_points >= 10 && (anchor_failed_rays * 100 / rays_with_points) > 75) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "Ground segmentation: %d of %d active rays (%.0f%%) failed ground anchor check! "
        "Verify 'expected_ground_z' (%.2f) or chassis TF calibration.",
        anchor_failed_rays, rays_with_points,
        (100.0 * anchor_failed_rays / rays_with_points), expected_ground_z_);
    }
  }

  // -----------------------------------------------------------------------------------------------
  // Step 6b: Extract Terrain Model for 2D LiDAR Slope Assistance (in Level Frame)
  // -----------------------------------------------------------------------------------------------
  if (enable_2d_lidar_filtering_ && run_ground_filter) {
    TerrainModel new_terrain;
    new_terrain.stamp = cloud_msg->header.stamp;
    new_terrain.valid = false;

    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    double sum_xx = 0.0, sum_xy = 0.0, sum_yy = 0.0;
    double sum_xz = 0.0, sum_yz = 0.0, sum_zz = 0.0;
    size_t count = 0;
    double min_y = 1e9, max_y = -1e9;

    for (int ray_idx = 0; ray_idx < num_rays_; ++ray_idx) {
      const int ray_offset = ray_idx * num_bins_;
      double n = 0.0, sr = 0.0, sz = 0.0, srr = 0.0, srz = 0.0, szz_ray = 0.0;
      double r_lo = std::numeric_limits<double>::max(), r_hi = -1.0;

      const int max_6b_bin = std::min(num_bins_, static_cast<int>(std::ceil(4.6 * inv_bin_size)));
      for (int r_bin = 0; r_bin < max_6b_bin; ++r_bin) {
        const auto & b = radial_ray_bins_[ray_offset + r_bin];
        if (!b.valid || b.is_obstacle) continue;
        const auto & pt = b.pt;

        // Focus strictly on near-zone elevated ground contact points (0.2m to 4.5m) directly in front of the rover.
        // Filtering out flat ground points (z <= expected_ground_z_ + 2.0 * z_noise_) ensures that approaching a ramp
        // does not contaminate the plane/line fit with a flat-then-ramp piecewise kink (which blows up RMS residual).
        if (pt.lx > 0.2f && pt.range_level <= 4.5 &&
            pt.z > (expected_ground_z_ + 2.0 * z_noise_)) {
          sum_x += pt.lx;
          sum_y += pt.ly;
          sum_z += pt.z;
          sum_xx += pt.lx * pt.lx;
          sum_xy += pt.lx * pt.ly;
          sum_yy += pt.ly * pt.ly;
          sum_xz += pt.lx * pt.z;
          sum_yz += pt.ly * pt.z;
          sum_zz += pt.z * pt.z;
          min_y = std::min(min_y, static_cast<double>(pt.ly));
          max_y = std::max(max_y, static_cast<double>(pt.ly));
          count++;

          n += 1.0;
          sr += pt.range_level;
          sz += pt.z;
          srr += pt.range_level * pt.range_level;
          srz += pt.range_level * pt.z;
          szz_ray += pt.z * pt.z;
          r_lo = std::min(r_lo, pt.range_level);
          r_hi = std::max(r_hi, pt.range_level);
        }
      }

      // Along this ray, fit least-squares slope (requires >= 4 points and >= 0.6m radial span)
      if (n >= 4.0 && (r_hi - r_lo) >= 0.6) {
        const double den = n * srr - sr * sr;
        if (den > 1e-6) {
          const double m   = (n * srz - sr * sz) / den;
          const double c0  = (sz - m * sr) / n;
          const double residual_sq = szz_ray - m * srz - c0 * sz;
          const double rms = std::sqrt(std::max(0.0, residual_sq / n));
          if (m >= 0.08 && m <= tan_max + 0.05 && rms <= 0.03) {
            new_terrain.ray_slopes[ray_idx] = std::make_tuple(m, r_lo, c0 + m * r_lo, r_hi);
          }
        }
      }
    }

    if (count >= 8) {
      double N = static_cast<double>(count);
      const double mx = sum_x / N, my = sum_y / N;
      const double vxx = sum_xx / N - mx * mx;
      const double vyy = sum_yy / N - my * my;
      const double vxy = sum_xy / N - mx * my;
      const bool spread_ok = (vxx > 0.05) && (vyy > 0.02) && ((vxx * vyy - vxy * vxy) > 0.01 * vxx * vyy);

      if (spread_ok) {
        Eigen::Matrix3d A;
        A << sum_xx, sum_xy, sum_x,
             sum_xy, sum_yy, sum_y,
             sum_x,  sum_y,  N;

        Eigen::Vector3d rhs(sum_xz, sum_yz, sum_z);
        Eigen::LDLT<Eigen::Matrix3d> ldlt(A);

        if (ldlt.info() == Eigen::Success && ldlt.isPositive()) {
          Eigen::Vector3d abc = ldlt.solve(rhs);
          double a = abc[0];
          double b = abc[1];
          double c = abc[2];

          if (std::isfinite(a) && std::isfinite(b) && std::isfinite(c)) {
            // Check RMS residual of plane fit
            double residual_sum = sum_zz - (a * sum_xz + b * sum_yz + c * sum_z);
            double rms = std::sqrt(std::max(0.0, residual_sum / N));

            double slope_mag = std::hypot(a, b);
            double slope_ang = std::atan(slope_mag);

            // Plane must have low residual (no kink), forward upward incline, and traversable slope
            if (rms <= 0.04 && a > 0.03 && slope_ang >= 0.05 && slope_ang <= max_slope_angle_rad_ + 0.05) {
              new_terrain.has_slope_ahead = true;
              new_terrain.plane_a = a;
              new_terrain.plane_b = b;
              new_terrain.plane_c = c;
              new_terrain.slope_angle = slope_ang;
              new_terrain.min_y = min_y;
              new_terrain.max_y = max_y;
            }
          }
        }
      }
    }

    new_terrain.valid = true;
    {
      std::lock_guard<std::mutex> lock(terrain_mutex_);
      latest_terrain_ = std::move(new_terrain);
    }
  }

  // -----------------------------------------------------------------------------------------------
  // Step 7: Squish remaining true obstacles into 2D LaserScan
  // -----------------------------------------------------------------------------------------------
  const bool has_ground_sub = debug_publish_clouds_ && debug_ground_pub_ && (debug_ground_pub_->get_subscription_count() > 0);
  const bool has_obstacle_sub = debug_publish_clouds_ && debug_obstacle_pub_ && (debug_obstacle_pub_->get_subscription_count() > 0);
  const bool collect_debug_clouds = has_ground_sub || has_obstacle_sub;

  std::vector<Point3D> debug_ground_pts;
  std::vector<Point3D> debug_obstacle_pts;
  if (collect_debug_clouds) {
    debug_ground_pts.reserve(all_valid_points_.size());
    debug_obstacle_pts.reserve(all_valid_points_.size());
  }

  for (const auto & pt : all_valid_points_) {
    bool is_ground = false;
    double local_ground_z = expected_ground_z_;

    if (run_ground_filter) {
      if (pt.terrain_ray_idx >= 0 && pt.terrain_ray_idx < num_rays_ &&
          pt.terrain_r_bin >= 0 && pt.terrain_r_bin < num_bins_)
      {
        const auto & bin = radial_ray_bins_[pt.terrain_ray_idx * num_bins_ + pt.terrain_r_bin];
        if (bin.valid) {
          local_ground_z = bin.has_ground ? bin.ground_z : expected_ground_z_;
          if (!bin.is_obstacle && (pt.z - bin.pt.z <= max_step_height_)) {
            is_ground = true;
          }
        }
      }
    }

    if (is_ground) {
      if (has_ground_sub) debug_ground_pts.push_back(pt);
      continue; // Successfully filtered out traversable ground surface!
    }

    // Local ceiling check relative to terrain surface:
    // Discard overhead points (tree branches, overhead cables, ceilings) that are more than max_height_
    // above the local ground level. This preserves the sensing horizon on hills while maintaining overhead clearance.
    if (pt.z - local_ground_z > max_height_) {
      continue;
    }

    // Surviving point is a TRUE OBSTACLE! Project onto 2D LaserScan ranges[]
    if (has_obstacle_sub) debug_obstacle_pts.push_back(pt);

    if (pt.ray_idx >= 0 && pt.ray_idx < static_cast<int>(scan_msg->ranges.size())) {
      if (pt.range < scan_msg->ranges[pt.ray_idx]) {
        scan_msg->ranges[pt.ray_idx] = pt.range;
      }
    }
  }

  if (collect_debug_clouds) {
    auto create_cloud = [&](const std::vector<Point3D> & pts) {
      auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
      cloud->header = scan_msg->header;
      cloud->height = 1;
      cloud->width = pts.size();
      cloud->is_dense = true;
      cloud->is_bigendian = false;
      sensor_msgs::PointCloud2Modifier modifier(*cloud);
      modifier.setPointCloud2FieldsByString(1, "xyz");
      modifier.resize(pts.size());
      sensor_msgs::PointCloud2Iterator<float> it_x(*cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> it_y(*cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> it_z(*cloud, "z");
      for (const auto & p : pts) {
        *it_x = p.x;
        *it_y = p.y;
        *it_z = p.raw_z; // Matches raw base_link height perfectly in RViz!
        ++it_x; ++it_y; ++it_z;
      }
      return cloud;
    };
    if (has_ground_sub) debug_ground_pub_->publish(*create_cloud(debug_ground_pts));
    if (has_obstacle_sub) debug_obstacle_pub_->publish(*create_cloud(debug_obstacle_pts));
  }

  // -----------------------------------------------------------------------------------------------
  // Step 8: Publish the final 2D LaserScan
  // -----------------------------------------------------------------------------------------------
  scan_pub_->publish(std::move(scan_msg));
}

void GroundSegmentationNode::scan2dCallback(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan_msg)
{
  if (!enable_2d_lidar_filtering_) {
    scan_2d_pub_->publish(*scan_msg);
    return;
  }

  auto out_scan = std::make_unique<sensor_msgs::msg::LaserScan>(*scan_msg);

  TerrainModel terrain;
  {
    std::lock_guard<std::mutex> lock(terrain_mutex_);
    terrain = latest_terrain_;
  }

  // Staleness Guard: expire terrain model if older than configurable timeout relative to 2D scan stamp
  if (terrain.valid) {
    double age = std::abs((rclcpp::Time(scan_msg->header.stamp) - terrain.stamp).seconds());
    if (age > terrain_model_timeout_) {
      terrain.valid = false;
    }
  }

  // Early-out if terrain model is invalid or has no upcoming ramp/slopes
  if (!terrain.valid || (!terrain.has_slope_ahead && terrain.ray_slopes.empty())) {
    scan_2d_pub_->publish(std::move(out_scan));
    return;
  }

  // Query current attitude for this scan
  double roll = 0.0, pitch = 0.0;
  if (!getTilt(scan_msg->header.stamp, roll, pitch)) {
    // Unknown tilt -> fail-safe: do not suppress any obstacles!
    scan_2d_pub_->publish(std::move(out_scan));
    return;
  }

  tf2::Matrix3x3 R_level;
  R_level.setRPY(roll, pitch, 0.0);

  // Dynamic TF Lookup with fallback cache to survive transient delay drops
  double tx = 0.0, ty = 0.0, tz = 0.0;
  tf2::Matrix3x3 rot;
  rot.setIdentity();

  if (!scan_msg->header.frame_id.empty() && scan_msg->header.frame_id != target_frame_) {
    try {
      geometry_msgs::msg::TransformStamped tf_stamped;
      try {
        tf_stamped = tf2_buffer_->lookupTransform(
          target_frame_, scan_msg->header.frame_id,
          scan_msg->header.stamp, tf2::durationFromSec(tolerance_));
      } catch (const tf2::TransformException &) {
        tf_stamped = tf2_buffer_->lookupTransform(
          target_frame_, scan_msg->header.frame_id,
          tf2::TimePointZero, tf2::durationFromSec(tolerance_));
      }

      tx = tf_stamped.transform.translation.x;
      ty = tf_stamped.transform.translation.y;
      tz = tf_stamped.transform.translation.z;

      tf2::Quaternion q(
        tf_stamped.transform.rotation.x,
        tf_stamped.transform.rotation.y,
        tf_stamped.transform.rotation.z,
        tf_stamped.transform.rotation.w);
      rot.setRotation(q);

      // Cache latest valid transform
      cached_tx_ = tx;
      cached_ty_ = ty;
      cached_tz_ = tz;
      cached_rot_ = rot;
      cached_tf_frame_ = scan_msg->header.frame_id;
      cached_tf_stamp_ = scan_msg->header.stamp;
      has_cached_tf_2d_ = true;
    } catch (const tf2::TransformException & ex) {
      if (has_cached_tf_2d_ && cached_tf_frame_ == scan_msg->header.frame_id &&
          std::abs((rclcpp::Time(scan_msg->header.stamp) - cached_tf_stamp_).seconds()) < 1.0)
      {
        tx = cached_tx_;
        ty = cached_ty_;
        tz = cached_tz_;
        rot = cached_rot_;
      } else {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "TF lookup failed from '" << scan_msg->header.frame_id << "' to '" << target_frame_
          << "': " << ex.what());
        scan_2d_pub_->publish(std::move(out_scan));
        return;
      }
    }
  }

  const tf2::Vector3 t_base(tx, ty, tz);

  uint32_t ranges_size_3d = std::ceil((angle_max_ - angle_min_) / angle_increment_);
  const size_t num_readings = scan_msg->ranges.size();

  // Fast-path: terrain model only spans near-zone (0.2m to 4.5m + buffer).
  // Beams beyond 5.5m (or scan_2d_max_range_) can never be ramp hits; skip expensive TF matrix math!
  const float max_eval_r = std::min(static_cast<float>(scan_2d_max_range_), 5.5f);

  for (size_t i = 0; i < num_readings; ++i) {
    float r = scan_msg->ranges[i];
    if (!std::isfinite(r) || r < scan_msg->range_min || r > scan_msg->range_max || r > max_eval_r) {
      continue;
    }

    double beam_angle = scan_msg->angle_min + i * scan_msg->angle_increment;
    double lx = r * std::cos(beam_angle);
    double ly = r * std::sin(beam_angle);

    tf2::Vector3 pt_laser(lx, ly, 0.0);
    tf2::Vector3 pt_base = rot * pt_laser + t_base;              // Point in base_link
    tf2::Vector3 pt_level = R_level * pt_base;                   // Point in level frame

    bool is_fake_ramp_obstacle = false;

    // Check 1: Evaluated traversable slope plane (bounded to fitted near zone: 0.2m <= x <= 4.5m, y within observed span +/- 0.2m, range <= 5.0m)
    if (terrain.has_slope_ahead && pt_level.x() >= 0.2 && pt_level.x() <= 4.5 &&
        pt_level.y() >= (terrain.min_y - 0.2) && pt_level.y() <= (terrain.max_y + 0.2) &&
        r <= 5.0f)
    {
      // Direct vertical distance to predicted plane in Level Frame: z_plane = a*x + b*y + c
      double pred_z = terrain.plane_a * pt_level.x() + terrain.plane_b * pt_level.y() + terrain.plane_c;
      if (std::abs(pt_level.z() - pred_z) <= ramp_hit_tolerance_) {
        is_fake_ramp_obstacle = true;
      }
    }

    // Check 2: Ray-by-ray elevation profile (bounded to observed range span in Level Frame)
    if (!is_fake_ramp_obstacle && !terrain.ray_slopes.empty()) {
      double angle_level = std::atan2(pt_level.y(), pt_level.x());
      int ray_idx = static_cast<int>(std::floor((angle_level - angle_min_) / angle_increment_));
      int num_rays_int = static_cast<int>(ranges_size_3d);
      if (angle_max_ - angle_min_ >= 2.0 * M_PI - 0.05) {
        ray_idx = (ray_idx % num_rays_int + num_rays_int) % num_rays_int;
      }
      if (ray_idx >= 0 && ray_idx < num_rays_int) {
        auto ray_it = terrain.ray_slopes.find(ray_idx);
        if (ray_it != terrain.ray_slopes.end()) {
          double ray_slope = std::get<0>(ray_it->second);
          double first_r   = std::get<1>(ray_it->second);
          double first_z   = std::get<2>(ray_it->second);
          double last_r    = std::get<3>(ray_it->second);
          double r_level   = std::hypot(pt_level.x(), pt_level.y());

          // Strictly within observed radial span (no unconstrained extrapolation!)
          if (r_level >= first_r - 0.2 && r_level <= last_r + 0.2) {
            double expected_ground_z = first_z + ray_slope * (r_level - first_r);
            if (std::abs(pt_level.z() - expected_ground_z) <= ramp_hit_tolerance_) {
              is_fake_ramp_obstacle = true;
            }
          }
        }
      }
    }

    if (is_fake_ramp_obstacle) {
      if (use_inf_) {
        out_scan->ranges[i] = std::numeric_limits<float>::infinity();
      } else {
        out_scan->ranges[i] = static_cast<float>(out_scan->range_max + inf_epsilon_);
      }
      if (i < out_scan->intensities.size()) {
        out_scan->intensities[i] = 0.0f;
      }
    }
  }

  scan_2d_pub_->publish(std::move(out_scan));
}

}  // namespace ground_segmentation

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ground_segmentation::GroundSegmentationNode)
