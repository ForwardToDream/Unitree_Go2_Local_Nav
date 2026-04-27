#include <algorithm>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "mdog_grid_msgs/msg/grid_cell.hpp"
#include "mdog_grid_msgs/msg/local_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class MDogSemanticMarkerVizNode : public rclcpp::Node {
 public:
  MDogSemanticMarkerVizNode() : Node("mdog_semantic_marker_viz") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/mdog/local_grid");
    output_topic_ = declare_parameter<std::string>("output_topic", "/mdog/semantic_markers");
    grid_sub_ = create_subscription<mdog_grid_msgs::msg::LocalGrid>(
        input_topic_, 10,
        [this](mdog_grid_msgs::msg::LocalGrid::SharedPtr msg) { marker_pub_->publish(makeMarkers(*msg)); });
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(output_topic_, 10);
    RCLCPP_INFO(get_logger(), "Semantic marker viz: %s -> %s", input_topic_.c_str(), output_topic_.c_str());
  }

 private:
  visualization_msgs::msg::MarkerArray makeMarkers(const mdog_grid_msgs::msg::LocalGrid& grid) const {
    visualization_msgs::msg::MarkerArray markers;
    auto low_marker = makeCubeListMarker(grid, "mdog_low_obstacle", 0, color(1.0F, 0.75F, 0.05F, 0.85F));
    auto body_marker = makeCubeListMarker(grid, "mdog_body_obstacle", 1, color(0.95F, 0.10F, 0.10F, 0.85F));
    auto obstacle_marker = makeCubeListMarker(grid, "mdog_obstacle", 2, color(0.70F, 0.10F, 0.90F, 0.85F));

    for (uint32_t gy = 0; gy < grid.height; ++gy) {
      for (uint32_t gx = 0; gx < grid.width; ++gx) {
        const auto& cell = grid.cells[index(grid, gx, gy)];
        if (cell.label == mdog_grid_msgs::msg::GridCell::FREE ||
            cell.label == mdog_grid_msgs::msg::GridCell::UNKNOWN) {
          continue;
        }
        geometry_msgs::msg::Point p;
        p.x = grid.origin.position.x + (gx + 0.5) * grid.resolution;
        p.y = grid.origin.position.y + (gy + 0.5) * grid.resolution;
        p.z = std::max(0.02F, cell.max_z);
        if (cell.label == mdog_grid_msgs::msg::GridCell::LOW_OBSTACLE) {
          low_marker.points.push_back(p);
        } else if (cell.label == mdog_grid_msgs::msg::GridCell::BODY_OBSTACLE) {
          body_marker.points.push_back(p);
        } else {
          obstacle_marker.points.push_back(p);
        }
      }
    }

    markers.markers.push_back(low_marker);
    markers.markers.push_back(body_marker);
    markers.markers.push_back(obstacle_marker);
    markers.markers.push_back(makeTextMarker(grid, 10, "LOW_OBSTACLE: low trip hazard", 0.25, -1.85, 1.15,
                                             color(1.0F, 0.75F, 0.05F, 1.0F)));
    markers.markers.push_back(makeTextMarker(grid, 11, "BODY_OBSTACLE: blocks owner envelope", 0.25, -1.85, 1.35,
                                             color(0.95F, 0.10F, 0.10F, 1.0F)));
    markers.markers.push_back(makeTextMarker(grid, 12, "OBSTACLE: occupied/unknown height", 0.25, -1.85, 1.55,
                                             color(0.70F, 0.10F, 0.90F, 1.0F)));
    return markers;
  }

  visualization_msgs::msg::Marker makeCubeListMarker(
      const mdog_grid_msgs::msg::LocalGrid& grid,
      const std::string& ns,
      int id,
      const std_msgs::msg::ColorRGBA& marker_color) const {
    visualization_msgs::msg::Marker marker;
    marker.header = grid.header;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = grid.resolution;
    marker.scale.y = grid.resolution;
    marker.scale.z = 0.08;
    marker.color = marker_color;
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    return marker;
  }

  visualization_msgs::msg::Marker makeTextMarker(
      const mdog_grid_msgs::msg::LocalGrid& grid,
      int id,
      const std::string& text,
      double x,
      double y,
      double z,
      const std_msgs::msg::ColorRGBA& marker_color) const {
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

  size_t index(const mdog_grid_msgs::msg::LocalGrid& grid, uint32_t gx, uint32_t gy) const {
    return static_cast<size_t>(gy) * grid.width + gx;
  }

  std::string input_topic_;
  std::string output_topic_;
  rclcpp::Subscription<mdog_grid_msgs::msg::LocalGrid>::SharedPtr grid_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MDogSemanticMarkerVizNode>());
  rclcpp::shutdown();
  return 0;
}
