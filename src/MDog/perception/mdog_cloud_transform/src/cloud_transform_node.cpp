#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace {
struct Point3 {
  float x{};
  float y{};
  float z{};
};
}  // namespace

class MDogCloudTransformNode : public rclcpp::Node {
 public:
  MDogCloudTransformNode() : Node("mdog_cloud_transform") {
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    lidar_input_topic_ = declare_parameter<std::string>("lidar_input_topic", "/unilidar/cloud");
    depth_input_topic_ = declare_parameter<std::string>("depth_input_topic", "/go2/camera/depth/points");
    lidar_output_topic_ = declare_parameter<std::string>("lidar_output_topic", "/mdog/lidar_points_base");
    depth_output_topic_ = declare_parameter<std::string>("depth_output_topic", "/mdog/depth_points_base");
    front_m_ = declare_parameter<double>("roi_front_m", 4.0);
    back_m_ = declare_parameter<double>("roi_back_m", 0.8);
    side_m_ = declare_parameter<double>("roi_side_m", 2.0);
    min_z_ = declare_parameter<double>("roi_min_z", -0.6);
    max_z_ = declare_parameter<double>("roi_max_z", 1.8);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    lidar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(lidar_output_topic_, 10);
    depth_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(depth_output_topic_, 10);
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_input_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { transformAndPublish(*msg, *lidar_pub_); });
    depth_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        depth_input_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { transformAndPublish(*msg, *depth_pub_); });

    RCLCPP_INFO(get_logger(), "Cloud transform: %s,%s -> %s in %s",
                lidar_input_topic_.c_str(), depth_input_topic_.c_str(),
                lidar_output_topic_.c_str(), target_frame_.c_str());
  }

 private:
  void transformAndPublish(
      const sensor_msgs::msg::PointCloud2& cloud,
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>& publisher) {
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_->lookupTransform(target_frame_, cloud.header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "No transform %s -> %s: %s",
                           cloud.header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      return;
    }

    tf2::Transform transform;
    tf2::fromMsg(tf_msg.transform, transform);

    std::vector<Point3> points;
    points.reserve(cloud.width * cloud.height);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
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
        points.push_back(Point3{static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
      }
    } catch (const std::runtime_error& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Malformed cloud on %s: %s",
                           cloud.header.frame_id.c_str(), ex.what());
      return;
    }

    publisher.publish(makeCloud(points, cloud.header.stamp));
  }

  sensor_msgs::msg::PointCloud2 makeCloud(
      const std::vector<Point3>& points, const builtin_interfaces::msg::Time& stamp) const {
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
  std::string lidar_output_topic_;
  std::string depth_output_topic_;
  double front_m_{};
  double back_m_{};
  double side_m_{};
  double min_z_{};
  double max_z_{};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MDogCloudTransformNode>());
  rclcpp::shutdown();
  return 0;
}
