/**
 * =================================================================================================
 * UAS-DTU Ground Segmentation & Terrain-Aware LaserScan Projection Node
 * Author: Vortex
 *
 * Description:
 *   Drop-in replacement for pointcloud_to_laserscan with integrated 3D ground segmentation,
 *   gravity-aligned level frame transformation (R_level), slope angle analysis (<=30 deg traversability),
 *   step-height filtering, and negative obstacle (ditch/drop-off) detection.
 *
 * Mathematical Foundations:
 *   1. Attitude Extraction & Level Frame Construction:
 *      Extracts chassis roll (phi) and pitch (theta) from TF / Odometry with a staleness watchdog.
 *      Constructs R_level = RPY(roll, pitch, 0) to rotate points into a gravity-aligned, yaw-free frame.
 *   2. Isotropic Traversability Slope:
 *      In the gravity-aligned level frame, allowable slope is uniformly tan(max_slope_angle) in all
 *      360 degree azimuth directions without body-frame azimuth budget distortions.
 *   3. Frame-Consistent Polar Projection:
 *      LaserScan range and azimuth are computed strictly from base_link coordinates to avoid double-rotation
 *      in downstream Nav2 costmaps, while level-frame Z is preserved for terrain slope classification.
 *   4. Differential Traversability:
 *      Along each radial ray in the flattened 1D bin grid:
 *      - Local steps (Delta_Z > max_step_height) or drops (Delta_Z < -max_drop_height) mark obstacles.
 *      - Verified traversable ramps (Delta_Z <= tan_max * Delta_R) are filtered out.
 *      - Gap reachability prevents occlusion shadows from poisoning distant ground returns.
 *   5. Ceiling Clearance:
 *      Any point with Z > max_height above the ray's confirmed local ground datum is discarded so overhead
 *      tree branches or doorframes are NEVER squished into the 2D obstacle scan!
 * =================================================================================================
 */

#ifndef GROUND_SEGMENTATION__GROUND_SEGMENTATION_NODE_HPP_
#define GROUND_SEGMENTATION__GROUND_SEGMENTATION_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <unordered_map>
#include <tuple>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <mutex>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "ground_segmentation/visibility_control.h"

namespace ground_segmentation
{

class GroundSegmentationNode : public rclcpp::Node
{
public:
  GROUND_SEGMENTATION_PUBLIC
  explicit GroundSegmentationNode(const rclcpp::NodeOptions & options);

  ~GroundSegmentationNode() override;

private:
  /**
   * @brief Main processing callback triggered upon receiving each 3D PointCloud2.
   * @param cloud_msg ConstSharedPtr to incoming 3D PointCloud2 message.
   */
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg);

  // -----------------------------------------------------------------------------------------------
  // TF2 Listeners and Buffers
  // -----------------------------------------------------------------------------------------------
  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

  // -----------------------------------------------------------------------------------------------
  // ROS 2 Publishers and Subscriptions
  // -----------------------------------------------------------------------------------------------
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

  // Optional debug point cloud publishers for RViz visualization
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_obstacle_pub_;

  // -----------------------------------------------------------------------------------------------
  // Standard pointcloud_to_laserscan Parameters (Preserved 1:1 for complete drop-in compatibility)
  // -----------------------------------------------------------------------------------------------
  std::string target_frame_;      // Frame to transform points into (typically 'base_link')
  double tolerance_;             // TF transform lookup tolerance in seconds (e.g. 0.05)
  int input_queue_size_;         // Queue size for incoming pointclouds (e.g. 10)

  double min_height_;            // Absolute lower height bound relative to target_frame (e.g. -0.5m)
  double max_height_;            // Absolute upper ceiling bound relative to target_frame (e.g. 1.1m)

  double angle_min_;             // Minimum angle for LaserScan in radians (e.g. -M_PI or -35 deg)
  double angle_max_;             // Maximum angle for LaserScan in radians (e.g. +M_PI or +35 deg)
  double angle_increment_;       // Angular resolution between laser rays in radians (e.g. 0.0055 rad)
  double scan_time_;             // Nominal scan duration in seconds (e.g. 0.1s)
  double range_min_;             // Minimum valid laser range in meters (e.g. 0.2m or 1.5m)
  double range_max_;             // Maximum valid laser range in meters (e.g. 40.0m)

  bool use_inf_;                 // If true, empty rays report infinity; if false, range_max + inf_epsilon
  double inf_epsilon_;           // Padding added to range_max when use_inf is false

  // 3D Spatial Voxel Outlier Filter (Preserved from Vortex's fork)
  bool enable_3d_point_filter_;  // If true, filters isolated points lacking sufficient voxel neighbors
  double filter_voxel_size_;     // 3D cube size for spatial density counting (e.g. 0.10m)
  int min_points_per_voxel_;     // Minimum point count in a voxel to be considered valid (e.g. 2)

  // -----------------------------------------------------------------------------------------------
  // [NEW] Ground Segmentation & Dynamic Slope Filtering Parameters
  // -----------------------------------------------------------------------------------------------
  bool enable_ground_filtering_; // Master switch to enable terrain-aware slope filtering (Default: true)
  std::string gravity_frame_;    // World/Gravity-aligned reference frame (e.g. 'map', 'odom', 'base_footprint')

  double max_slope_angle_deg_;   // Maximum traversable hill/ramp incline in degrees (Default: 30.0 deg)
  double max_slope_angle_rad_;   // Computed in radians: max_slope_angle_deg * M_PI / 180.0

  double max_step_height_;       // Maximum vertical step jump considered traversable (Default: 0.12m)
  double max_drop_height_;       // Maximum downward vertical drop before marking a ditch edge (Default: 0.18m)
  double z_noise_{0.03};         // Vertical elevation noise floor to prevent false triggers (Default: 0.03m)
  bool enable_drop_detection_{true}; // Flag to enable negative obstacle / drop-off detection (Default: true)
  double max_drop_range_{4.0};   // Maximum radial range to mark drop-off ledges across gaps (Default: 4.0m)

  double expected_ground_z_{0.0}; // Nominal height of ground plane relative to target_frame in level frame (Default: 0.0m)
  double ground_start_clearance_; // Maximum distance from rover ground plane for first ray return (Default: 0.15m)
  double anchor_near_range_{0.50}; // Base origin distance to where wheels touch ground (Default: 0.50m)
  double anchor_max_reach_{1.20};  // Maximum radial reach beyond wheel contact zone for slope cone tolerance expansion (Default: 1.20m)
  double plane_anchor_max_range_{3.0}; // Maximum radial range for base_link wheel-plane ground anchoring (Default: 3.0m)
  double gap_slope_factor_{0.5}; // Fraction of max_slope trusted across unobserved gaps (Default: 0.5)
  double radial_bin_size_{0.10}; // Bin size along radial rays for slope calculation (Default: 0.10m)
  double max_gap_distance_{0.40}; // Maximum radial distance between consecutive ground points before breaking continuity (Default: 0.40m)
  bool debug_publish_clouds_{false}; // Flag to publish debug point clouds (Default: false)
  bool visualise_{false};            // Flag to publish color-coded traversable (green) & non-traversable (red) pointclouds (Default: false)
  // -----------------------------------------------------------------------------------------------
  // [NEW] Direct Odometry Topic Subscription (Bypasses TF lookup delays entirely)
  // -----------------------------------------------------------------------------------------------
  bool use_odom_topic_;          // If true, reads pitch/roll and position directly from topic (Default: true)
  std::string odom_topic_;       // Topic name for robot odometry (Default: "/mavros/local_position/odom")
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Thread-safe storage for latest odometry pose and attitude
  std::mutex odom_mutex_;
  bool odom_received_{false};
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME};
  double odom_roll_{0.0};
  double odom_pitch_{0.0};
  double odom_yaw_{0.0};
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_z_{0.0};

  /**
   * @brief Odometry callback to ingest true 3D chassis attitude without any TF delay.
   * @param odom_msg ConstSharedPtr to nav_msgs::msg::Odometry.
   */
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg);

  /**
   * @brief Extracts current chassis roll and pitch relative to gravity frame.
   * @param stamp Timestamp to query attitude for.
   * @param roll Output roll in radians.
   * @param pitch Output pitch in radians.
   * @return true if attitude was successfully obtained, false otherwise.
   */
  bool getTilt(const rclcpp::Time & stamp, double & roll, double & pitch);

  // -----------------------------------------------------------------------------------------------
  // [NEW] 2D LiDAR Slope-Filtering Parameters & Handlers (SF45/B Close-Range Blind Zone Assist)
  // -----------------------------------------------------------------------------------------------
  bool enable_2d_lidar_filtering_{true};
  std::string scan_2d_in_topic_{"/lightwarelidar/scan"};
  std::string scan_2d_out_topic_{"/scan_2d_filtered"};
  double scan_2d_max_range_{50.0};
  double ramp_hit_tolerance_{0.08};
  double terrain_model_timeout_{0.25};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_2d_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_2d_pub_;

  // Thread-safe terrain profile modeled from 3D LiDAR point cloud
  struct TerrainModel {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool valid{false};
    bool has_slope_ahead{false};
    double plane_a{0.0};
    double plane_b{0.0};
    double plane_c{0.0};
    double slope_angle{0.0};
    double min_y{-1.5};
    double max_y{1.5};
    // Map ray_idx -> {slope_dr_dz, first_r, first_z, last_r}
    std::unordered_map<int, std::tuple<double, double, double, double>> ray_slopes;
  };

  std::mutex terrain_mutex_;
  TerrainModel latest_terrain_;

  // Cached transform for 2D LiDAR frame to survive transient TF glitches
  bool has_cached_tf_2d_{false};
  std::string cached_tf_frame_{""};
  rclcpp::Time cached_tf_stamp_{0, 0, RCL_ROS_TIME};
  double cached_tx_{0.0};
  double cached_ty_{0.0};
  double cached_tz_{0.0};
  tf2::Matrix3x3 cached_rot_{tf2::Matrix3x3::getIdentity()};

  /**
   * @brief 2D LaserScan callback to filter out fake ramp obstacles and publish to dedicated topic.
   * @param scan_msg ConstSharedPtr to incoming 2D sensor_msgs::msg::LaserScan.
   */
  void scan2dCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan_msg);

  // -----------------------------------------------------------------------------------------------
  // High-performance flattened 1D radial terrain grid (O(1) lookups, zero heap allocations in loop)
  // -----------------------------------------------------------------------------------------------
  struct Point3D {
    // base_link coordinates (strictly used for LaserScan ranges[] projection and RViz debug clouds)
    float x{0.0f};            // base_link x
    float y{0.0f};            // base_link y
    float raw_z{0.0f};        // base_link z
    double range{0.0};        // base_link range
    int ray_idx{0};           // base_link azimuth ray index

    // Gravity-aligned level-frame coordinates (used for terrain analysis, slope checks, line fit, and plane fit)
    float lx{0.0f};           // gravity-aligned level-frame x
    float ly{0.0f};           // gravity-aligned level-frame y
    float z{0.0f};            // gravity-aligned level-frame z
    double range_level{0.0};  // gravity-aligned level-frame radial distance

    // Terrain grid indexing (strictly organized in gravity-aligned level frame)
    int terrain_ray_idx{-1};  // gravity-aligned level azimuth ray index
    int terrain_r_bin{-1};    // gravity-aligned level radial distance bin index
  };

  struct BinData {
    bool valid{false};
    bool is_obstacle{false};
    Point3D pt;
    double ground_z{0.0};     // Estimated or confirmed ground height at this bin
    bool has_ground{false};   // True if ground datum was established on this ray
  };

  int num_rays_{0};
  int num_bins_{0};
  std::vector<BinData> radial_ray_bins_;
  std::vector<Point3D> all_valid_points_;
  std::unordered_map<uint64_t, int> voxel_point_counts_;

  // Pre-allocated buffers for two-pass ray processing (zero heap allocations in callback)
  std::vector<int> first_bins_;
  std::vector<uint8_t> pass1_anchored_;
};

}  // namespace ground_segmentation

#endif  // GROUND_SEGMENTATION__GROUND_SEGMENTATION_NODE_HPP_
