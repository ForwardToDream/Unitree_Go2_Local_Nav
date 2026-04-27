#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "mdog_body_msgs/msg/body_envelope.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace {
struct Point3 {
  float x{};
  float y{};
  float z{};
};
}  // namespace

class MDogCloudSelfFilterNode : public rclcpp::Node {
 public:
  MDogCloudSelfFilterNode() : Node("mdog_cloud_self_filter") {
    dog_body_topic_ = declare_parameter<std::string>("dog_body_topic", "/mdog/dog_body");
    lidar_input_topic_ = declare_parameter<std::string>("lidar_input_topic", "/mdog/lidar_points_base");
    depth_input_topic_ = declare_parameter<std::string>("depth_input_topic", "/mdog/depth_points_base");
    lidar_output_topic_ = declare_parameter<std::string>("lidar_output_topic", "/mdog/lidar_points_filtered");
    depth_output_topic_ = declare_parameter<std::string>("depth_output_topic", "/mdog/depth_points_filtered");

    lidar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(lidar_output_topic_, 10);
    depth_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(depth_output_topic_, 10);
    dog_body_sub_ = create_subscription<mdog_body_msgs::msg::BodyEnvelope>(
        dog_body_topic_, 10,
        [this](mdog_body_msgs::msg::BodyEnvelope::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          dog_body_ = msg;
        });
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_input_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { filterAndPublish(*msg, *lidar_pub_); });
    depth_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        depth_input_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { filterAndPublish(*msg, *depth_pub_); });

    RCLCPP_INFO(get_logger(), "Cloud self filter: dog=%s", dog_body_topic_.c_str());
  }

 private:
  void filterAndPublish(
      const sensor_msgs::msg::PointCloud2& cloud,
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>& publisher) {
    mdog_body_msgs::msg::BodyEnvelope::SharedPtr dog_body;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      dog_body = dog_body_;
    }

    if (!dog_body || !dog_body->enabled) {
      publisher.publish(cloud);
      return;
    }

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
        if (insideBody(*iter_x, *iter_y, *iter_z, *dog_body)) {
          continue;
        }
        points.push_back(Point3{*iter_x, *iter_y, *iter_z});
      }
    } catch (const std::runtime_error& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Malformed cloud for self-filter: %s", ex.what());
      return;
    }
    publisher.publish(makeCloud(points, cloud.header));
  }

  bool insideBody(
      double x, double y, double z,
      const mdog_body_msgs::msg::BodyEnvelope& body) const {
    const double half_x = body.size.x * 0.5;
    const double half_y = body.size.y * 0.5;
    const double half_z = body.size.z * 0.5;
    return x >= body.pose.position.x - half_x && x <= body.pose.position.x + half_x &&
           y >= body.pose.position.y - half_y && y <= body.pose.position.y + half_y &&
           z >= body.pose.position.z - half_z && z <= body.pose.position.z + half_z;
  }

  sensor_msgs::msg::PointCloud2 makeCloud(
      const std::vector<Point3>& points, const std_msgs::msg::Header& header) const {
    sensor_msgs::msg::PointCloud2 out;
    out.header = header;
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

  std::string dog_body_topic_;
  std::string lidar_input_topic_;
  std::string depth_input_topic_;
  std::string lidar_output_topic_;
  std::string depth_output_topic_;

  std::mutex mutex_;
  mdog_body_msgs::msg::BodyEnvelope::SharedPtr dog_body_;
  rclcpp::Subscription<mdog_body_msgs::msg::BodyEnvelope>::SharedPtr dog_body_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MDogCloudSelfFilterNode>());
  rclcpp::shutdown();
  return 0;
}
