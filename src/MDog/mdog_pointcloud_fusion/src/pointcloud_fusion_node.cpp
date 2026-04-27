#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

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

class MDogPointCloudFusionNode : public rclcpp::Node {
 public:
  MDogPointCloudFusionNode() : Node("mdog_pointcloud_fusion") {
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    lidar_topic_ = declare_parameter<std::string>("lidar_cloud_topic", "/unilidar/cloud");
    depth_topic_ = declare_parameter<std::string>("depth_cloud_topic", "/go2/camera/depth/points");
    output_topic_ = declare_parameter<std::string>("output_topic", "/mdog/fused_points");
    front_m_ = declare_parameter<double>("roi_front_m", 4.0);
    back_m_ = declare_parameter<double>("roi_back_m", 0.8);
    side_m_ = declare_parameter<double>("roi_side_m", 2.0);
    min_z_ = declare_parameter<double>("roi_min_z", -0.6);
    max_z_ = declare_parameter<double>("roi_max_z", 1.8);
    voxel_size_ = declare_parameter<double>("voxel_size_m", 0.05);
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 0.75);
    publish_rate_ = declare_parameter<double>("publish_rate", 10.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    auto qos = rclcpp::SensorDataQoS();
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_lidar_.msg = msg;
          latest_lidar_.received_at = get_clock()->now();
        });
    depth_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        depth_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_depth_.msg = msg;
          latest_depth_.received_at = get_clock()->now();
        });
    fused_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        [this]() { publishFusedCloud(); });

    RCLCPP_INFO(get_logger(), "MDog pointcloud fusion: %s + %s -> %s in %s",
                lidar_topic_.c_str(), depth_topic_.c_str(), output_topic_.c_str(),
                target_frame_.c_str());
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
    const rclcpp::Time now = get_clock()->now();
    appendCloudIfFresh(lidar, now, points);
    appendCloudIfFresh(depth, now, points);
    if (points.empty()) {
      return;
    }

    auto output = makePointCloud(points, now);
    fused_pub_->publish(output);
  }

  void appendCloudIfFresh(
      const CachedCloud& cached_cloud,
      const rclcpp::Time& now, std::vector<Point3>& points) {
    const auto& cloud = cached_cloud.msg;
    if (!cloud) {
      return;
    }
    if ((now - cached_cloud.received_at).seconds() > stale_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000, "Cloud on %s is stale by arrival time",
          cloud->header.frame_id.c_str());
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_->lookupTransform(
          target_frame_, cloud->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000, "No transform %s -> %s: %s",
          cloud->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      return;
    }

    tf2::Transform transform;
    tf2::fromMsg(tf_msg.transform, transform);
    std::unordered_set<std::string> occupied_voxels;
    occupied_voxels.reserve(cloud->width * cloud->height / 4 + 1);

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
          continue;
        }
        const tf2::Vector3 p = transform * tf2::Vector3(*iter_x, *iter_y, *iter_z);
        const double x = p.x();
        const double y = p.y();
        const double z = p.z();
        if (x < -back_m_ || x > front_m_ || std::abs(y) > side_m_ || z < min_z_ || z > max_z_) {
          continue;
        }
        const std::string key = voxelKey(x, y, z);
        if (!occupied_voxels.insert(key).second) {
          continue;
        }
        points.push_back(Point3{static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
      }
    } catch (const std::runtime_error& ex) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000, "Skipping malformed cloud on %s: %s",
          cloud->header.frame_id.c_str(), ex.what());
    }
  }

  std::string voxelKey(double x, double y, double z) const {
    const int ix = static_cast<int>(std::floor(x / voxel_size_));
    const int iy = static_cast<int>(std::floor(y / voxel_size_));
    const int iz = static_cast<int>(std::floor(z / voxel_size_));
    return std::to_string(ix) + ":" + std::to_string(iy) + ":" + std::to_string(iz);
  }

  sensor_msgs::msg::PointCloud2 makePointCloud(
      const std::vector<Point3>& points, const rclcpp::Time& stamp) const {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = target_frame_;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(points.size());
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
    for (const auto& point : points) {
      *out_x = point.x;
      *out_y = point.y;
      *out_z = point.z;
      ++out_x;
      ++out_y;
      ++out_z;
    }
    return cloud;
  }

  std::string target_frame_;
  std::string lidar_topic_;
  std::string depth_topic_;
  std::string output_topic_;
  double front_m_{};
  double back_m_{};
  double side_m_{};
  double min_z_{};
  double max_z_{};
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
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MDogPointCloudFusionNode>());
  rclcpp::shutdown();
  return 0;
}
