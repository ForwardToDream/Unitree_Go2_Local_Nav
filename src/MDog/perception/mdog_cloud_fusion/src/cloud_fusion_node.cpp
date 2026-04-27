#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace {
struct Point3 {
  float x{};
  float y{};
  float z{};
};

struct CachedCloud {
  sensor_msgs::msg::PointCloud2::SharedPtr msg;
  rclcpp::Time received_at;
};
}  // namespace

class MDogCloudFusionNode : public rclcpp::Node {
 public:
  MDogCloudFusionNode() : Node("mdog_cloud_fusion") {
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    lidar_input_topic_ = declare_parameter<std::string>("lidar_input_topic", "/mdog/lidar_points_filtered");
    depth_input_topic_ = declare_parameter<std::string>("depth_input_topic", "/mdog/depth_points_filtered");
    output_topic_ = declare_parameter<std::string>("output_topic", "/mdog/fused_points");
    voxel_size_ = declare_parameter<double>("voxel_size_m", 0.05);
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 0.75);
    publish_rate_ = declare_parameter<double>("publish_rate", 10.0);

    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_input_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_lidar_ = CachedCloud{msg, get_clock()->now()};
        });
    depth_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        depth_input_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_depth_ = CachedCloud{msg, get_clock()->now()};
        });
    fused_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        [this]() { publishFusedCloud(); });

    RCLCPP_INFO(get_logger(), "Cloud fusion: %s + %s -> %s",
                lidar_input_topic_.c_str(), depth_input_topic_.c_str(), output_topic_.c_str());
  }

 private:
  void publishFusedCloud() {
    CachedCloud lidar;
    CachedCloud depth;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      lidar = latest_lidar_;
      depth = latest_depth_;
    }

    std::vector<Point3> points;
    points.reserve(20000);
    std::unordered_set<std::string> occupied_voxels;
    occupied_voxels.reserve(20000);
    const auto now = get_clock()->now();
    appendCloudIfFresh(lidar, now, points, occupied_voxels);
    appendCloudIfFresh(depth, now, points, occupied_voxels);
    if (!points.empty()) {
      fused_pub_->publish(makeCloud(points, now));
    }
  }

  void appendCloudIfFresh(
      const CachedCloud& cached_cloud,
      const rclcpp::Time& now,
      std::vector<Point3>& points,
      std::unordered_set<std::string>& occupied_voxels) {
    const auto& cloud = cached_cloud.msg;
    if (!cloud) {
      return;
    }
    if ((now - cached_cloud.received_at).seconds() > stale_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Filtered cloud on %s is stale",
                           cloud->header.frame_id.c_str());
      return;
    }
    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
          continue;
        }
        const std::string key = voxelKey(*iter_x, *iter_y, *iter_z);
        if (!occupied_voxels.insert(key).second) {
          continue;
        }
        points.push_back(Point3{*iter_x, *iter_y, *iter_z});
      }
    } catch (const std::runtime_error& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Malformed filtered cloud: %s", ex.what());
    }
  }

  std::string voxelKey(double x, double y, double z) const {
    const int ix = static_cast<int>(std::floor(x / voxel_size_));
    const int iy = static_cast<int>(std::floor(y / voxel_size_));
    const int iz = static_cast<int>(std::floor(z / voxel_size_));
    return std::to_string(ix) + ":" + std::to_string(iy) + ":" + std::to_string(iz);
  }

  sensor_msgs::msg::PointCloud2 makeCloud(
      const std::vector<Point3>& points, const rclcpp::Time& stamp) const {
    sensor_msgs::msg::PointCloud2 out;
    out.header.stamp = stamp;
    out.header.frame_id = target_frame_;
    out.height = 1;
    out.width = static_cast<uint32_t>(points.size());
    sensor_msgs::PointCloud2Modifier modifier(out);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> out_x(out, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(out, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(out, "z");
    for (const auto& point : points) {
      *out_x = point.x;
      *out_y = point.y;
      *out_z = point.z;
      ++out_x;
      ++out_y;
      ++out_z;
    }
    return out;
  }

  std::string target_frame_;
  std::string lidar_input_topic_;
  std::string depth_input_topic_;
  std::string output_topic_;
  double voxel_size_{};
  double stale_timeout_sec_{};
  double publish_rate_{};

  std::mutex mutex_;
  CachedCloud latest_lidar_;
  CachedCloud latest_depth_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fused_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MDogCloudFusionNode>());
  rclcpp::shutdown();
  return 0;
}
