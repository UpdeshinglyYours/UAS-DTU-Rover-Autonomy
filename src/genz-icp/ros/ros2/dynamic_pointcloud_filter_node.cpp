// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss.
// Modified by Daehan Lee, Hyungtae Lim, and Soohee Han, 2024

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace genz_icp_ros {
namespace {

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;

constexpr int kDefaultBufferSize = 5;
constexpr double kDefaultDynamicThresholdMeters = 0.35;
constexpr int kDefaultMinHistoryMatches = 2;
constexpr double kDefaultTransformTimeoutSec = 0.2;

struct XyzOffsets {
    size_t x{0};
    size_t y{0};
    size_t z{0};
};

struct VoxelKey {
    std::int64_t x{0};
    std::int64_t y{0};
    std::int64_t z{0};

    bool operator==(const VoxelKey &other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash {
    size_t operator()(const VoxelKey &key) const {
        const size_t hx = std::hash<std::int64_t>{}(key.x);
        const size_t hy = std::hash<std::int64_t>{}(key.y);
        const size_t hz = std::hash<std::int64_t>{}(key.z);
        return hx ^ (hy + 0x9e3779b97f4a7c15ULL + (hx << 6) + (hx >> 2)) ^
               (hz + 0x9e3779b97f4a7c15ULL + (hy << 6) + (hy >> 2));
    }
};

struct HistoryCloud {
    std::unordered_map<VoxelKey, std::vector<Eigen::Vector3d>, VoxelKeyHash> voxels;
    size_t point_count{0};
};

const PointField *FindField(const PointCloud2 &cloud, const std::string &name) {
    const auto it = std::find_if(cloud.fields.begin(), cloud.fields.end(),
                                 [&](const PointField &field) {
                                     return field.name == name && field.count > 0;
                                 });
    return it == cloud.fields.end() ? nullptr : &(*it);
}

std::optional<XyzOffsets> GetXyzOffsets(const PointCloud2 &cloud) {
    const auto *x = FindField(cloud, "x");
    const auto *y = FindField(cloud, "y");
    const auto *z = FindField(cloud, "z");
    if (x == nullptr || y == nullptr || z == nullptr) {
        return std::nullopt;
    }
    if (x->datatype != PointField::FLOAT32 || y->datatype != PointField::FLOAT32 ||
        z->datatype != PointField::FLOAT32) {
        return std::nullopt;
    }
    if (x->offset + sizeof(float) > cloud.point_step ||
        y->offset + sizeof(float) > cloud.point_step ||
        z->offset + sizeof(float) > cloud.point_step) {
        return std::nullopt;
    }
    return XyzOffsets{x->offset, y->offset, z->offset};
}

std::optional<size_t> PointCount(const PointCloud2 &cloud) {
    const size_t width = static_cast<size_t>(cloud.width);
    const size_t height = static_cast<size_t>(cloud.height);
    if (height != 0 && width > std::numeric_limits<size_t>::max() / height) {
        return std::nullopt;
    }
    return width * height;
}

bool HasValidLayout(const PointCloud2 &cloud) {
    const auto point_count = PointCount(cloud);
    if (!point_count) {
        return false;
    }
    if (*point_count == 0) {
        return true;
    }
    if (cloud.width == 0 || cloud.height == 0 || cloud.point_step == 0) {
        return false;
    }

    const size_t row_step = static_cast<size_t>(cloud.row_step);
    const size_t point_step = static_cast<size_t>(cloud.point_step);
    const size_t width = static_cast<size_t>(cloud.width);
    const size_t height = static_cast<size_t>(cloud.height);
    if (width > std::numeric_limits<size_t>::max() / point_step) {
        return false;
    }
    if (row_step < width * point_step) {
        return false;
    }
    if (height - 1 > (std::numeric_limits<size_t>::max() - width * point_step) / row_step) {
        return false;
    }

    const size_t required_data_size = (height - 1) * row_step + width * point_step;
    return cloud.data.size() >= required_data_size;
}

size_t PointByteOffset(const PointCloud2 &cloud, const size_t point_index) {
    const size_t row = point_index / static_cast<size_t>(cloud.width);
    const size_t col = point_index % static_cast<size_t>(cloud.width);
    return row * static_cast<size_t>(cloud.row_step) + col * static_cast<size_t>(cloud.point_step);
}

bool ReadPoint(const PointCloud2 &cloud,
               const XyzOffsets &offsets,
               const size_t point_index,
               Eigen::Vector3d *point) {
    const size_t base = PointByteOffset(cloud, point_index);
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    std::memcpy(&x, cloud.data.data() + base + offsets.x, sizeof(float));
    std::memcpy(&y, cloud.data.data() + base + offsets.y, sizeof(float));
    std::memcpy(&z, cloud.data.data() + base + offsets.z, sizeof(float));

    *point = Eigen::Vector3d(static_cast<double>(x),
                             static_cast<double>(y),
                             static_cast<double>(z));
    return point->allFinite();
}

std::optional<Eigen::Isometry3d> TransformToEigen(
    const geometry_msgs::msg::TransformStamped &transform) {
    const Eigen::Vector3d translation(transform.transform.translation.x,
                                      transform.transform.translation.y,
                                      transform.transform.translation.z);
    Eigen::Quaterniond rotation(transform.transform.rotation.w,
                                transform.transform.rotation.x,
                                transform.transform.rotation.y,
                                transform.transform.rotation.z);
    const double rotation_norm = rotation.norm();
    if (!translation.allFinite() || !rotation.coeffs().allFinite() ||
        !(rotation_norm > std::numeric_limits<double>::epsilon())) {
        return std::nullopt;
    }

    rotation.normalize();
    return Eigen::Translation3d(translation) * rotation;
}

}  // namespace

class DynamicPointCloudFilterNode final : public rclcpp::Node {
public:
    DynamicPointCloudFilterNode()
        : rclcpp::Node("dynamic_pointcloud_filter"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_) {
        input_topic_ =
            declare_parameter<std::string>("input_topic", "/bf_lidar/point_cloud_deskewed");
        output_topic_ =
            declare_parameter<std::string>("output_topic", "/bf_lidar/point_cloud_filtered");
        fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
        lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lidar");
        buffer_size_ = declare_parameter<int>("buffer_size", kDefaultBufferSize);
        dynamic_threshold_meters_ =
            declare_parameter<double>("dynamic_threshold_meters",
                                      kDefaultDynamicThresholdMeters);
        min_history_matches_ =
            declare_parameter<int>("min_history_matches", kDefaultMinHistoryMatches);
        transform_timeout_sec_ =
            declare_parameter<double>("transform_timeout_sec", kDefaultTransformTimeoutSec);
        publish_in_original_frame_ =
            declare_parameter<bool>("publish_in_original_frame", true);
        warmup_publish_unfiltered_ =
            declare_parameter<bool>("warmup_publish_unfiltered", true);
        use_filtered_cloud_for_history_ =
            declare_parameter<bool>("use_filtered_cloud_for_history", true);
        debug_print_ = declare_parameter<bool>("debug_print", false);

        SanitizeParameters();
        dynamic_threshold_squared_ =
            dynamic_threshold_meters_ * dynamic_threshold_meters_;

        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(), this->get_node_timers_interface());
        tf_buffer_.setCreateTimerInterface(timer_interface);

        publisher_ = create_publisher<PointCloud2>(output_topic_, rclcpp::SensorDataQoS());
        subscriber_ = create_subscription<PointCloud2>(
            input_topic_, rclcpp::SensorDataQoS(),
            std::bind(&DynamicPointCloudFilterNode::CloudCallback, this,
                      std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "Dynamic PointCloud2 filter: input=%s output=%s fixed_frame=%s "
                    "buffer_size=%d threshold=%.3f m min_history_matches=%d",
                    input_topic_.c_str(), output_topic_.c_str(), fixed_frame_.c_str(),
                    buffer_size_, dynamic_threshold_meters_, min_history_matches_);
        RCLCPP_INFO(get_logger(),
                    "publish_in_original_frame=%s warmup_publish_unfiltered=%s "
                    "use_filtered_cloud_for_history=%s transform_timeout=%.3f s",
                    publish_in_original_frame_ ? "true" : "false",
                    warmup_publish_unfiltered_ ? "true" : "false",
                    use_filtered_cloud_for_history_ ? "true" : "false",
                    transform_timeout_sec_);
    }

private:
    void CloudCallback(const PointCloud2::ConstSharedPtr msg) {
        const auto point_count = PointCount(*msg);
        const auto xyz_offsets = GetXyzOffsets(*msg);
        if (!point_count || !xyz_offsets || !HasValidLayout(*msg)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Invalid PointCloud2 layout or missing FLOAT32 x/y/z fields; publishing unchanged");
            publisher_->publish(*msg);
            return;
        }

        if (*point_count == 0) {
            publisher_->publish(*msg);
            return;
        }

        const std::string source_frame = ResolveSourceFrame(*msg);
        if (source_frame.empty() || fixed_frame_.empty()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Cannot filter cloud with empty source or fixed frame; publishing unchanged");
            publisher_->publish(*msg);
            return;
        }

        const auto source_to_fixed =
            LookupTransform(source_frame, fixed_frame_, msg->header.stamp);
        if (!source_to_fixed) {
            publisher_->publish(*msg);
            return;
        }

        std::vector<Eigen::Vector3d> current_fixed_points;
        std::vector<bool> valid_points;
        ExtractFixedFramePoints(*msg, *xyz_offsets, *source_to_fixed,
                                current_fixed_points, valid_points);

        if (history_.empty() && warmup_publish_unfiltered_) {
            publisher_->publish(*msg);
            AddHistoryFromAllCurrent(current_fixed_points, valid_points);
            if (debug_print_) {
                RCLCPP_INFO(get_logger(),
                            "Dynamic filter warmup: input=%zu kept=%zu removed=0 history=%zu",
                            *point_count, *point_count, history_.size());
            }
            return;
        }

        const size_t required_matches =
            std::min(history_.size(), static_cast<size_t>(min_history_matches_));
        std::vector<size_t> kept_indices;
        kept_indices.reserve(*point_count);

        for (size_t i = 0; i < *point_count; ++i) {
            if (!valid_points[i] || HasRequiredHistorySupport(current_fixed_points[i],
                                                              required_matches)) {
                kept_indices.push_back(i);
            }
        }

        PointCloud2 output =
            BuildFilteredCloud(*msg, source_frame, *xyz_offsets, kept_indices,
                               current_fixed_points, valid_points);
        publisher_->publish(output);

        if (use_filtered_cloud_for_history_) {
            AddHistoryFromKeptCurrent(current_fixed_points, valid_points, kept_indices);
        } else {
            AddHistoryFromAllCurrent(current_fixed_points, valid_points);
        }

        if (debug_print_) {
            RCLCPP_INFO(get_logger(),
                        "Dynamic filter: input=%zu kept=%zu removed=%zu history=%zu",
                        *point_count, kept_indices.size(), *point_count - kept_indices.size(),
                        history_.size());
        }
    }

    void SanitizeParameters() {
        if (buffer_size_ < 1) {
            RCLCPP_WARN(get_logger(), "Invalid buffer_size=%d; using %d",
                        buffer_size_, kDefaultBufferSize);
            buffer_size_ = kDefaultBufferSize;
        }
        if (min_history_matches_ < 1) {
            RCLCPP_WARN(get_logger(), "Invalid min_history_matches=%d; using %d",
                        min_history_matches_, kDefaultMinHistoryMatches);
            min_history_matches_ = kDefaultMinHistoryMatches;
        }
        if (!std::isfinite(dynamic_threshold_meters_) ||
            !(dynamic_threshold_meters_ > 0.0)) {
            RCLCPP_WARN(get_logger(), "Invalid dynamic_threshold_meters; using %.3f",
                        kDefaultDynamicThresholdMeters);
            dynamic_threshold_meters_ = kDefaultDynamicThresholdMeters;
        }
        if (!std::isfinite(transform_timeout_sec_) || transform_timeout_sec_ < 0.0) {
            RCLCPP_WARN(get_logger(), "Invalid transform_timeout_sec; using %.3f",
                        kDefaultTransformTimeoutSec);
            transform_timeout_sec_ = kDefaultTransformTimeoutSec;
        }
    }

    std::string ResolveSourceFrame(const PointCloud2 &cloud) const {
        if (!cloud.header.frame_id.empty()) {
            return cloud.header.frame_id;
        }
        return lidar_frame_;
    }

    std::optional<Eigen::Isometry3d> LookupTransform(const std::string &source_frame,
                                                     const std::string &target_frame,
                                                     const rclcpp::Time &stamp) {
        if (source_frame == target_frame) {
            return Eigen::Isometry3d::Identity();
        }

        try {
            const auto transform = tf_buffer_.lookupTransform(
                target_frame, source_frame, stamp,
                rclcpp::Duration::from_seconds(transform_timeout_sec_));
            const auto eigen_transform = TransformToEigen(transform);
            if (!eigen_transform) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "TF transform %s -> %s has non-finite translation or rotation; "
                    "publishing unchanged",
                    source_frame.c_str(), target_frame.c_str());
            }
            return eigen_transform;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "TF transform %s -> %s failed: %s; publishing unchanged",
                                 source_frame.c_str(), target_frame.c_str(), ex.what());
            return std::nullopt;
        }
    }

    void ExtractFixedFramePoints(const PointCloud2 &cloud,
                                 const XyzOffsets &offsets,
                                 const Eigen::Isometry3d &source_to_fixed,
                                 std::vector<Eigen::Vector3d> &fixed_points,
                                 std::vector<bool> &valid_points) const {
        const size_t point_count = *PointCount(cloud);
        const double quiet_nan = std::numeric_limits<double>::quiet_NaN();
        fixed_points.assign(point_count, Eigen::Vector3d(quiet_nan, quiet_nan, quiet_nan));
        valid_points.assign(point_count, false);

        for (size_t i = 0; i < point_count; ++i) {
            Eigen::Vector3d source_point;
            if (!ReadPoint(cloud, offsets, i, &source_point)) {
                continue;
            }

            const Eigen::Vector3d fixed_point = source_to_fixed * source_point;
            if (!fixed_point.allFinite()) {
                continue;
            }

            fixed_points[i] = fixed_point;
            valid_points[i] = true;
        }
    }

    PointCloud2 BuildFilteredCloud(const PointCloud2 &input,
                                   const std::string &source_frame,
                                   const XyzOffsets &offsets,
                                   const std::vector<size_t> &kept_indices,
                                   const std::vector<Eigen::Vector3d> &fixed_points,
                                   const std::vector<bool> &valid_points) const {
        PointCloud2 output;
        output.header = input.header;
        output.header.frame_id = publish_in_original_frame_ ? source_frame : fixed_frame_;
        output.height = 1;
        output.width = static_cast<std::uint32_t>(kept_indices.size());
        output.fields = input.fields;
        output.is_bigendian = input.is_bigendian;
        output.point_step = input.point_step;
        output.row_step = output.point_step * output.width;
        output.is_dense = false;
        output.data.resize(static_cast<size_t>(output.row_step));

        for (size_t out_index = 0; out_index < kept_indices.size(); ++out_index) {
            const size_t in_index = kept_indices[out_index];
            const size_t input_base = PointByteOffset(input, in_index);
            const size_t output_base = out_index * static_cast<size_t>(output.point_step);
            std::memcpy(output.data.data() + output_base,
                        input.data.data() + input_base,
                        static_cast<size_t>(input.point_step));

            if (!publish_in_original_frame_ && valid_points[in_index]) {
                const float x = static_cast<float>(fixed_points[in_index].x());
                const float y = static_cast<float>(fixed_points[in_index].y());
                const float z = static_cast<float>(fixed_points[in_index].z());
                std::memcpy(output.data.data() + output_base + offsets.x, &x, sizeof(float));
                std::memcpy(output.data.data() + output_base + offsets.y, &y, sizeof(float));
                std::memcpy(output.data.data() + output_base + offsets.z, &z, sizeof(float));
            }
        }

        return output;
    }

    bool HasRequiredHistorySupport(const Eigen::Vector3d &point,
                                   const size_t required_matches) const {
        if (required_matches == 0) {
            return true;
        }

        size_t support_count = 0;
        for (const auto &history_cloud : history_) {
            if (HasSupportInHistoryCloud(point, history_cloud)) {
                ++support_count;
                if (support_count >= required_matches) {
                    return true;
                }
            }
        }
        return false;
    }

    bool HasSupportInHistoryCloud(const Eigen::Vector3d &point,
                                  const HistoryCloud &history_cloud) const {
        const VoxelKey center = VoxelFor(point);
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    const VoxelKey key{center.x + dx, center.y + dy, center.z + dz};
                    const auto voxel_it = history_cloud.voxels.find(key);
                    if (voxel_it == history_cloud.voxels.end()) {
                        continue;
                    }
                    for (const auto &candidate : voxel_it->second) {
                        if ((candidate - point).squaredNorm() <= dynamic_threshold_squared_) {
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    void AddHistoryFromAllCurrent(const std::vector<Eigen::Vector3d> &fixed_points,
                                  const std::vector<bool> &valid_points) {
        std::vector<Eigen::Vector3d> history_points;
        history_points.reserve(fixed_points.size());
        for (size_t i = 0; i < fixed_points.size(); ++i) {
            if (valid_points[i]) {
                history_points.push_back(fixed_points[i]);
            }
        }
        AddToHistory(std::move(history_points));
    }

    void AddHistoryFromKeptCurrent(const std::vector<Eigen::Vector3d> &fixed_points,
                                   const std::vector<bool> &valid_points,
                                   const std::vector<size_t> &kept_indices) {
        std::vector<Eigen::Vector3d> history_points;
        history_points.reserve(kept_indices.size());
        for (const size_t index : kept_indices) {
            if (valid_points[index]) {
                history_points.push_back(fixed_points[index]);
            }
        }
        AddToHistory(std::move(history_points));
    }

    void AddToHistory(std::vector<Eigen::Vector3d> history_points) {
        if (history_points.empty()) {
            return;
        }

        HistoryCloud history_cloud;
        history_cloud.point_count = history_points.size();
        for (const auto &point : history_points) {
            history_cloud.voxels[VoxelFor(point)].push_back(point);
        }

        history_.push_back(std::move(history_cloud));
        while (history_.size() > static_cast<size_t>(buffer_size_)) {
            history_.pop_front();
        }
    }

    VoxelKey VoxelFor(const Eigen::Vector3d &point) const {
        return VoxelKey{
            static_cast<std::int64_t>(std::floor(point.x() / dynamic_threshold_meters_)),
            static_cast<std::int64_t>(std::floor(point.y() / dynamic_threshold_meters_)),
            static_cast<std::int64_t>(std::floor(point.z() / dynamic_threshold_meters_))};
    }

    std::string input_topic_;
    std::string output_topic_;
    std::string fixed_frame_;
    std::string lidar_frame_;
    int buffer_size_{kDefaultBufferSize};
    double dynamic_threshold_meters_{kDefaultDynamicThresholdMeters};
    double dynamic_threshold_squared_{
        kDefaultDynamicThresholdMeters * kDefaultDynamicThresholdMeters};
    int min_history_matches_{kDefaultMinHistoryMatches};
    double transform_timeout_sec_{kDefaultTransformTimeoutSec};
    bool publish_in_original_frame_{true};
    bool warmup_publish_unfiltered_{true};
    bool use_filtered_cloud_for_history_{true};
    bool debug_print_{false};

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::deque<HistoryCloud> history_;

    rclcpp::Subscription<PointCloud2>::SharedPtr subscriber_;
    rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
};

}  // namespace genz_icp_ros

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<genz_icp_ros::DynamicPointCloudFilterNode>());
    rclcpp::shutdown();
    return 0;
}
