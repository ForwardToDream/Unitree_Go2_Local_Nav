#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "mdog_interfaces/msg/grid_cell.hpp"
#include "mdog_interfaces/msg/local_grid.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace {
using GridCell = mdog_interfaces::msg::GridCell;

struct CellAccumulator {
  float min_z{std::numeric_limits<float>::infinity()};
  float max_z{-std::numeric_limits<float>::infinity()};
  uint32_t point_count{0};
};
}  // namespace

class MDogSemanticMapNode : public rclcpp::Node {
 public:
  MDogSemanticMapNode() : Node("mdog_semantic_map") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/mdog/fused_points");
    local_grid_topic_ = declare_parameter<std::string>("local_grid_topic", "/mdog/local_grid");
    occupancy_topic_ = declare_parameter<std::string>("occupancy_topic", "/mdog/local_occupancy");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/mdog/semantic_markers");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    resolution_ = declare_parameter<double>("resolution", 0.10);
    front_m_ = declare_parameter<double>("roi_front_m", 4.0);
    back_m_ = declare_parameter<double>("roi_back_m", 0.8);
    side_m_ = declare_parameter<double>("roi_side_m", 2.0);
    human_height_m_ = declare_parameter<double>("human_height_m", 1.8);
    ground_z_ = declare_parameter<double>("ground_z_in_base_m", -0.35);
    low_obstacle_height_m_ = declare_parameter<double>("low_obstacle_height_m", 0.35);
    min_obstacle_height_m_ = declare_parameter<double>("min_obstacle_height_m", 0.05);

    width_ = static_cast<uint32_t>(std::ceil((front_m_ + back_m_) / resolution_));
    height_ = static_cast<uint32_t>(std::ceil((side_m_ * 2.0) / resolution_));

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { processCloud(*msg); });
    grid_pub_ = create_publisher<mdog_interfaces::msg::LocalGrid>(local_grid_topic_, 10);
    occupancy_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(occupancy_topic_, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);

    RCLCPP_INFO(get_logger(), "MDog semantic map: %s -> %s (%ux%u @ %.2fm)",
                input_topic_.c_str(), local_grid_topic_.c_str(), width_, height_, resolution_);
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

    auto grid = makeLocalGrid(nowMessage(), accum);
    auto occupancy = makeOccupancyGrid(grid);
    auto markers = makeMarkers(grid);
    grid_pub_->publish(grid);
    occupancy_pub_->publish(occupancy);
    marker_pub_->publish(markers);
  }

  mdog_interfaces::msg::LocalGrid makeLocalGrid(
      const builtin_interfaces::msg::Time& stamp,
      const std::vector<CellAccumulator>& accum) const {
    mdog_interfaces::msg::LocalGrid grid;
    grid.header.stamp = stamp;
    grid.header.frame_id = frame_id_;
    grid.resolution = static_cast<float>(resolution_);
    grid.width = width_;
    grid.height = height_;
    grid.origin.position.x = -back_m_;
    grid.origin.position.y = -side_m_;
    grid.origin.position.z = 0.0;
    grid.origin.orientation.w = 1.0;
    grid.cells.resize(accum.size());

    const double head_z = ground_z_ + human_height_m_;
    const double min_obstacle_z = ground_z_ + min_obstacle_height_m_;
    const double low_obstacle_z = ground_z_ + low_obstacle_height_m_;

    for (size_t i = 0; i < accum.size(); ++i) {
      auto& out = grid.cells[i];
      const auto& in = accum[i];
      out.point_count = static_cast<uint16_t>(std::min<uint32_t>(in.point_count, 65535));
      if (in.point_count == 0) {
        out.label = GridCell::FREE;
        out.min_z = 0.0F;
        out.max_z = 0.0F;
        continue;
      }

      out.min_z = in.min_z;
      out.max_z = in.max_z;
      if (in.max_z < min_obstacle_z || in.min_z > head_z) {
        out.label = GridCell::FREE;
      } else if (in.max_z < low_obstacle_z) {
        out.label = GridCell::LOW_OBSTACLE;
      } else if (in.min_z <= head_z) {
        out.label = GridCell::BODY_OBSTACLE;
      } else {
        out.label = GridCell::OBSTACLE;
      }
    }
    return grid;
  }

  builtin_interfaces::msg::Time nowMessage() {
    const int64_t nanoseconds = get_clock()->now().nanoseconds();
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
    return stamp;
  }

  nav_msgs::msg::OccupancyGrid makeOccupancyGrid(
      const mdog_interfaces::msg::LocalGrid& grid) const {
    nav_msgs::msg::OccupancyGrid occupancy;
    occupancy.header = grid.header;
    occupancy.info.resolution = grid.resolution;
    occupancy.info.width = grid.width;
    occupancy.info.height = grid.height;
    occupancy.info.origin = grid.origin;
    occupancy.data.resize(grid.cells.size(), 0);
    for (size_t i = 0; i < grid.cells.size(); ++i) {
      const auto label = grid.cells[i].label;
      if (label == GridCell::UNKNOWN) {
        occupancy.data[i] = -1;
      } else if (label == GridCell::FREE) {
        occupancy.data[i] = 0;
      } else {
        occupancy.data[i] = 100;
      }
    }
    return occupancy;
  }

  visualization_msgs::msg::MarkerArray makeMarkers(
      const mdog_interfaces::msg::LocalGrid& grid) const {
    visualization_msgs::msg::MarkerArray markers;
    auto low_marker = makeCubeListMarker(grid, "mdog_low_obstacle", 0, color(1.0F, 0.75F, 0.05F, 0.85F));
    auto body_marker = makeCubeListMarker(grid, "mdog_body_obstacle", 1, color(0.95F, 0.10F, 0.10F, 0.85F));
    auto obstacle_marker = makeCubeListMarker(grid, "mdog_obstacle", 2, color(0.70F, 0.10F, 0.90F, 0.85F));

    for (uint32_t gy = 0; gy < grid.height; ++gy) {
      for (uint32_t gx = 0; gx < grid.width; ++gx) {
        const auto& cell = grid.cells[index(gx, gy)];
        if (cell.label == GridCell::FREE || cell.label == GridCell::UNKNOWN) {
          continue;
        }
        geometry_msgs::msg::Point p;
        p.x = grid.origin.position.x + (gx + 0.5) * resolution_;
        p.y = grid.origin.position.y + (gy + 0.5) * resolution_;
        p.z = std::max(0.02F, cell.max_z);

        if (cell.label == GridCell::LOW_OBSTACLE) {
          low_marker.points.push_back(p);
        } else if (cell.label == GridCell::BODY_OBSTACLE) {
          body_marker.points.push_back(p);
        } else {
          obstacle_marker.points.push_back(p);
        }
      }
    }

    markers.markers.push_back(low_marker);
    markers.markers.push_back(body_marker);
    markers.markers.push_back(obstacle_marker);
    markers.markers.push_back(makeTextMarker(
        grid, 10, "LOW_OBSTACLE: low trip hazard", 0.25, -1.85, 1.15,
        color(1.0F, 0.75F, 0.05F, 1.0F)));
    markers.markers.push_back(makeTextMarker(
        grid, 11, "BODY_OBSTACLE: blocks owner envelope", 0.25, -1.85, 1.35,
        color(0.95F, 0.10F, 0.10F, 1.0F)));
    markers.markers.push_back(makeTextMarker(
        grid, 12, "OBSTACLE: occupied/unknown height", 0.25, -1.85, 1.55,
        color(0.70F, 0.10F, 0.90F, 1.0F)));
    return markers;
  }

  visualization_msgs::msg::Marker makeCubeListMarker(
      const mdog_interfaces::msg::LocalGrid& grid, const std::string& ns, int id,
      const std_msgs::msg::ColorRGBA& marker_color) const {
    visualization_msgs::msg::Marker marker;
    marker.header = grid.header;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = resolution_;
    marker.scale.y = resolution_;
    marker.scale.z = 0.08;
    marker.color = marker_color;
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    return marker;
  }

  visualization_msgs::msg::Marker makeTextMarker(
      const mdog_interfaces::msg::LocalGrid& grid, int id, const std::string& text,
      double x, double y, double z, const std_msgs::msg::ColorRGBA& marker_color) const {
    visualization_msgs::msg::Marker marker;
    marker.header = grid.header;
    marker.ns = "mdog_semantic_legend";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.14;
    marker.color = marker_color;
    marker.text = text;
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    return marker;
  }

  std_msgs::msg::ColorRGBA color(float r, float g, float b, float a) const {
    std_msgs::msg::ColorRGBA out;
    out.r = r;
    out.g = g;
    out.b = b;
    out.a = a;
    return out;
  }

  size_t index(uint32_t gx, uint32_t gy) const {
    return static_cast<size_t>(gy) * width_ + gx;
  }

  std::string input_topic_;
  std::string local_grid_topic_;
  std::string occupancy_topic_;
  std::string marker_topic_;
  std::string frame_id_;
  double resolution_{};
  double front_m_{};
  double back_m_{};
  double side_m_{};
  double human_height_m_{};
  double ground_z_{};
  double low_obstacle_height_m_{};
  double min_obstacle_height_m_{};
  uint32_t width_{};
  uint32_t height_{};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<mdog_interfaces::msg::LocalGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MDogSemanticMapNode>());
  rclcpp::shutdown();
  return 0;
}
