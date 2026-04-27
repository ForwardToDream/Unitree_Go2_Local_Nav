#include <memory>
#include <string>

#include "mdog_grid_msgs/msg/grid_cell.hpp"
#include "mdog_grid_msgs/msg/local_grid.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

class MDogOccupancyProjectionNode : public rclcpp::Node {
 public:
  MDogOccupancyProjectionNode() : Node("mdog_occupancy_projection") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/mdog/local_grid");
    output_topic_ = declare_parameter<std::string>("output_topic", "/mdog/local_occupancy");

    grid_sub_ = create_subscription<mdog_grid_msgs::msg::LocalGrid>(
        input_topic_, 10,
        [this](mdog_grid_msgs::msg::LocalGrid::SharedPtr msg) { publishOccupancy(*msg); });
    occupancy_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, 10);
    RCLCPP_INFO(get_logger(), "Occupancy projection: %s -> %s", input_topic_.c_str(), output_topic_.c_str());
  }

 private:
  void publishOccupancy(const mdog_grid_msgs::msg::LocalGrid& grid) {
    nav_msgs::msg::OccupancyGrid occupancy;
    occupancy.header = grid.header;
    occupancy.info.resolution = grid.resolution;
    occupancy.info.width = grid.width;
    occupancy.info.height = grid.height;
    occupancy.info.origin = grid.origin;
    occupancy.data.resize(grid.cells.size(), 0);
    for (size_t i = 0; i < grid.cells.size(); ++i) {
      const auto label = grid.cells[i].label;
      if (label == mdog_grid_msgs::msg::GridCell::UNKNOWN) {
        occupancy.data[i] = -1;
      } else if (label == mdog_grid_msgs::msg::GridCell::FREE) {
        occupancy.data[i] = 0;
      } else {
        occupancy.data[i] = 100;
      }
    }
    occupancy_pub_->publish(occupancy);
  }

  std::string input_topic_;
  std::string output_topic_;
  rclcpp::Subscription<mdog_grid_msgs::msg::LocalGrid>::SharedPtr grid_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MDogOccupancyProjectionNode>());
  rclcpp::shutdown();
  return 0;
}
