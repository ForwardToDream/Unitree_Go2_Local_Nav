#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "mdog_grid_msgs/msg/height_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace {
struct CellAccumulator {
  float min_z{std::numeric_limits<float>::infinity()};
  float max_z{-std::numeric_limits<float>::infinity()};
  uint32_t point_count{0};
};
}  // namespace

class MDogHeightGridBuilderNode : public rclcpp::Node {
 public:
  MDogHeightGridBuilderNode() : Node("mdog_height_grid_builder") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/mdog/fused_points");
    output_topic_ = declare_parameter<std::string>("output_topic", "/mdog/height_grid");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    resolution_ = declare_parameter<double>("resolution", 0.10);
    front_m_ = declare_parameter<double>("roi_front_m", 4.0);
    back_m_ = declare_parameter<double>("roi_back_m", 0.8);
    side_m_ = declare_parameter<double>("roi_side_m", 2.0);

    width_ = static_cast<uint32_t>(std::ceil((front_m_ + back_m_) / resolution_));
    height_ = static_cast<uint32_t>(std::ceil((side_m_ * 2.0) / resolution_));

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { processCloud(*msg); });
    grid_pub_ = create_publisher<mdog_grid_msgs::msg::HeightGrid>(output_topic_, 10);

    RCLCPP_INFO(get_logger(), "Height grid: %s -> %s (%ux%u @ %.2fm)",
                input_topic_.c_str(), output_topic_.c_str(), width_, height_, resolution_);
  }

 private:
  void processCloud(const sensor_msgs::msg::PointCloud2& cloud) {
    std::vector<CellAccumulator> accum(width_ * height_);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        const double x = *iter_x;
        const double y = *iter_y;
        const double z = *iter_z;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }
        const int gx = static_cast<int>(std::floor((x + back_m_) / resolution_));
        const int gy = static_cast<int>(std::floor((y + side_m_) / resolution_));
        if (gx < 0 || gy < 0 || gx >= static_cast<int>(width_) || gy >= static_cast<int>(height_)) {
          continue;
        }
        auto& cell = accum[index(gx, gy)];
        cell.min_z = std::min(cell.min_z, static_cast<float>(z));
        cell.max_z = std::max(cell.max_z, static_cast<float>(z));
        cell.point_count += 1;
      }
    } catch (const std::runtime_error& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Malformed fused cloud: %s", ex.what());
      return;
    }
    grid_pub_->publish(makeGrid(accum));
  }

  mdog_grid_msgs::msg::HeightGrid makeGrid(const std::vector<CellAccumulator>& accum) {
    mdog_grid_msgs::msg::HeightGrid grid;
    grid.header.stamp = get_clock()->now();
    grid.header.frame_id = frame_id_;
    grid.resolution = static_cast<float>(resolution_);
    grid.width = width_;
    grid.height = height_;
    grid.origin.position.x = -back_m_;
    grid.origin.position.y = -side_m_;
    grid.origin.orientation.w = 1.0;
    grid.cells.resize(accum.size());
    for (size_t i = 0; i < accum.size(); ++i) {
      auto& out = grid.cells[i];
      const auto& in = accum[i];
      out.point_count = static_cast<uint16_t>(std::min<uint32_t>(in.point_count, 65535));
      out.min_z = in.point_count == 0 ? 0.0F : in.min_z;
      out.max_z = in.point_count == 0 ? 0.0F : in.max_z;
    }
    return grid;
  }

  size_t index(uint32_t gx, uint32_t gy) const {
    return static_cast<size_t>(gy) * width_ + gx;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double resolution_{};
  double front_m_{};
  double back_m_{};
  double side_m_{};
  uint32_t width_{};
  uint32_t height_{};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<mdog_grid_msgs::msg::HeightGrid>::SharedPtr grid_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MDogHeightGridBuilderNode>());
  rclcpp::shutdown();
  return 0;
}
