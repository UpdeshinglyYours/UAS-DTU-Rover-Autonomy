#ifndef PC_WS_SYNTHETIC_POINT_CLOUD_NODE_HPP
#define PC_WS_SYNTHETIC_POINT_CLOUD_NODE_HPP

#include <boost/shared_ptr.hpp>
#include <chrono>
#include <memory>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "patchwork_plusplus/patchworkpp.hpp"
#include "patchwork_plusplus/utils.hpp"

namespace patchwork_plusplus {

using PointType = pcl::PointXYZI;
using namespace std;

class PatchworkPlusPlus : public rclcpp::Node {
public:
  explicit PatchworkPlusPlus(const rclcpp::NodeOptions &node_options);

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pc_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pc_ground_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pc_nonground_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pc_cloud_subscriber_;

  void point_cloud_callback(
      const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud);

  void show_parameters();

  std::shared_ptr<PatchWorkpp<PointType>> PatchworkppGroundSeg_;

  // Patchwork parameters
  double sensor_height_;
  int num_iter_;
  int num_lpr_;
  int num_min_pts_;
  double th_seeds_;
  double th_dist_;
  double th_seeds_v_;
  double th_dist_v_;
  double max_range_;
  double min_range_;
  double uprightness_thr_;
  double adaptive_seed_selection_margin_;
  double RNR_ver_angle_thr_;
  double RNR_intensity_thr_;
  int max_flatness_storage_, max_elevation_storage_;
  bool enable_RNR_;
  bool enable_RVPF_;
  bool enable_TGR_;

  int num_zones_;
  std::vector<int> num_sectors_each_zone_;
  std::vector<int>num_rings_each_zone_;
  std::vector<double> elevation_thr_;
  std::vector<double> flatness_thr_;
  //  int num_rings_of_interest_;
  //  double min_range_z2_; // 12.3625
  //  double min_range_z3_; // 22.025
  //  double min_range_z4_; // 41.35
  //  bool verbose_;
};
} // namespace patchwork_plusplus
#endif // PC_WS_SYNTHETIC_POINT_CLOUD_NODE_HPP
