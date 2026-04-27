#include <algorithm>
#include <memory>
#include <string>

#include "mdog_grid_msgs/msg/grid_cell.hpp"
#include "mdog_grid_msgs/msg/height_grid.hpp"
#include "mdog_grid_msgs/msg/local_grid.hpp"
#include "rclcpp/rclcpp.hpp"

class MDogSemanticLabelerNode : public rclcpp::Node {
 public:
  MDogSemanticLabelerNode() : Node("mdog_semantic_labeler") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/mdog/height_grid");
    output_topic_ = declare_parameter<std::string>("output_topic", "/mdog/local_grid");
    human_height_m_ = declare_parameter<double>("human_height_m", 1.8);
    ground_z_ = declare_parameter<double>("ground_z_in_base_m", -0.35);
    low_obstacle_height_m_ = declare_parameter<double>("low_obstacle_height_m", 0.35);
    min_obstacle_height_m_ = declare_parameter<double>("min_obstacle_height_m", 0.05);
    unknown_is_free_ = declare_parameter<bool>("unknown_is_free", true);

    grid_sub_ = create_subscription<mdog_grid_msgs::msg::HeightGrid>(
        input_topic_, 10,
        [this](mdog_grid_msgs::msg::HeightGrid::SharedPtr msg) { processGrid(*msg); });
    local_grid_pub_ = create_publisher<mdog_grid_msgs::msg::LocalGrid>(output_topic_, 10);

    RCLCPP_INFO(get_logger(), "Semantic labeler: %s -> %s", input_topic_.c_str(), output_topic_.c_str());
  }

 private:
  void processGrid(const mdog_grid_msgs::msg::HeightGrid& height_grid) {
    mdog_grid_msgs::msg::LocalGrid grid;
    grid.header = height_grid.header;
    grid.resolution = height_grid.resolution;
    grid.width = height_grid.width;
    grid.height = height_grid.height;
    grid.origin = height_grid.origin;
    grid.cells.resize(height_grid.cells.size());

    const double head_z = ground_z_ + human_height_m_;
    const double min_obstacle_z = ground_z_ + min_obstacle_height_m_;
    const double low_obstacle_z = ground_z_ + low_obstacle_height_m_;

    for (size_t i = 0; i < height_grid.cells.size(); ++i) {
      const auto& in = height_grid.cells[i];
      auto& out = grid.cells[i];
      out.min_z = in.min_z;
      out.max_z = in.max_z;
      out.point_count = in.point_count;
      if (in.point_count == 0) {
        out.label = unknown_is_free_ ? mdog_grid_msgs::msg::GridCell::FREE
                                     : mdog_grid_msgs::msg::GridCell::UNKNOWN;
      } else if (in.max_z < min_obstacle_z || in.min_z > head_z) {
        out.label = mdog_grid_msgs::msg::GridCell::FREE;
      } else if (in.max_z < low_obstacle_z) {
        out.label = mdog_grid_msgs::msg::GridCell::LOW_OBSTACLE;
      } else if (in.min_z <= head_z) {
        out.label = mdog_grid_msgs::msg::GridCell::BODY_OBSTACLE;
      } else {
        out.label = mdog_grid_msgs::msg::GridCell::OBSTACLE;
      }
    }
    local_grid_pub_->publish(grid);
  }

  std::string input_topic_;
  std::string output_topic_;
  double human_height_m_{};
  double ground_z_{};
  double low_obstacle_height_m_{};
  double min_obstacle_height_m_{};
  bool unknown_is_free_{true};

  rclcpp::Subscription<mdog_grid_msgs::msg::HeightGrid>::SharedPtr grid_sub_;
  rclcpp::Publisher<mdog_grid_msgs::msg::LocalGrid>::SharedPtr local_grid_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MDogSemanticLabelerNode>());
  rclcpp::shutdown();
  return 0;
}
