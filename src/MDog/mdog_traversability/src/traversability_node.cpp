#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "mdog_interfaces/msg/grid_cell.hpp"
#include "mdog_interfaces/msg/local_grid.hpp"
#include "mdog_interfaces/msg/traversability.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
using GridCell = mdog_interfaces::msg::GridCell;

struct CorridorResult {
  float score{1.0F};
  bool passable{true};
  float min_clearance{0.0F};
};
}  // namespace

class MDogTraversabilityNode : public rclcpp::Node {
 public:
  MDogTraversabilityNode() : Node("mdog_traversability") {
    local_grid_topic_ = declare_parameter<std::string>("local_grid_topic", "/mdog/local_grid");
    output_topic_ = declare_parameter<std::string>("output_topic", "/mdog/traversability");
    human_width_m_ = declare_parameter<double>("human_width_m", 0.8);
    safety_margin_m_ = declare_parameter<double>("safety_margin_m", 0.15);
    lookahead_m_ = declare_parameter<double>("lookahead_m", 2.0);
    side_lookahead_m_ = declare_parameter<double>("side_lookahead_m", 1.0);
    robot_blind_zone_m_ = declare_parameter<double>("robot_blind_zone_m", 0.15);
    pass_score_threshold_ = declare_parameter<double>("pass_score_threshold", 0.45);

    grid_sub_ = create_subscription<mdog_interfaces::msg::LocalGrid>(
        local_grid_topic_, 10,
        [this](mdog_interfaces::msg::LocalGrid::SharedPtr msg) { processGrid(*msg); });
    trav_pub_ = create_publisher<mdog_interfaces::msg::Traversability>(output_topic_, 10);

    RCLCPP_INFO(get_logger(), "MDog traversability: %s -> %s", local_grid_topic_.c_str(), output_topic_.c_str());
  }

 private:
  void processGrid(const mdog_interfaces::msg::LocalGrid& grid) {
    const auto front = evaluateCorridor(grid, 0.0, lookahead_m_);
    const auto left_front = evaluateCorridor(grid, 0.45, lookahead_m_);
    const auto right_front = evaluateCorridor(grid, -0.45, lookahead_m_);
    const auto left = evaluateCorridor(grid, 1.57079632679, side_lookahead_m_);
    const auto right = evaluateCorridor(grid, -1.57079632679, side_lookahead_m_);

    mdog_interfaces::msg::Traversability msg;
    msg.header = grid.header;
    msg.front_score = front.score;
    msg.left_score = left.score;
    msg.right_score = right.score;
    msg.left_front_score = left_front.score;
    msg.right_front_score = right_front.score;
    msg.front_passable = front.passable;
    msg.left_passable = left.passable || left_front.passable;
    msg.right_passable = right.passable || right_front.passable;
    msg.min_clearance_m = std::min({front.min_clearance, left_front.min_clearance, right_front.min_clearance});

    if (front.passable) {
      msg.recommended_heading_rad = 0.0F;
      msg.reason = "front_clear";
    } else if (left_front.passable && left_front.score >= right_front.score) {
      msg.recommended_heading_rad = 0.45F;
      msg.reason = "front_blocked_left_front_clear";
    } else if (right_front.passable) {
      msg.recommended_heading_rad = -0.45F;
      msg.reason = "front_blocked_right_front_clear";
    } else {
      msg.recommended_heading_rad = 0.0F;
      msg.reason = "blocked";
    }
    trav_pub_->publish(msg);
  }

  CorridorResult evaluateCorridor(
      const mdog_interfaces::msg::LocalGrid& grid, double heading_rad,
      double distance_m) const {
    CorridorResult result;
    const double half_width = human_width_m_ * 0.5 + safety_margin_m_;
    double closest_obstacle = distance_m;
    bool blocked = false;

    const double cos_h = std::cos(heading_rad);
    const double sin_h = std::sin(heading_rad);

    for (uint32_t gy = 0; gy < grid.height; ++gy) {
      for (uint32_t gx = 0; gx < grid.width; ++gx) {
        const auto& cell = grid.cells[index(grid, gx, gy)];
        if (isPassableLabel(cell.label)) {
          continue;
        }

        const double x = grid.origin.position.x + (gx + 0.5) * grid.resolution;
        const double y = grid.origin.position.y + (gy + 0.5) * grid.resolution;
        const double along = x * cos_h + y * sin_h;
        const double lateral = -x * sin_h + y * cos_h;
        if (along < robot_blind_zone_m_ || along > distance_m || std::abs(lateral) > half_width) {
          continue;
        }
        closest_obstacle = std::min(closest_obstacle, along);
        blocked = true;
      }
    }

    result.min_clearance = static_cast<float>(closest_obstacle);
    result.score = static_cast<float>(std::clamp(closest_obstacle / distance_m, 0.0, 1.0));
    result.passable = !blocked || result.score >= pass_score_threshold_;
    return result;
  }

  bool isPassableLabel(uint8_t label) const {
    return label == GridCell::FREE || label == GridCell::UNKNOWN;
  }

  size_t index(const mdog_interfaces::msg::LocalGrid& grid, uint32_t gx, uint32_t gy) const {
    return static_cast<size_t>(gy) * grid.width + gx;
  }

  std::string local_grid_topic_;
  std::string output_topic_;
  double human_width_m_{};
  double safety_margin_m_{};
  double lookahead_m_{};
  double side_lookahead_m_{};
  double robot_blind_zone_m_{};
  double pass_score_threshold_{};

  rclcpp::Subscription<mdog_interfaces::msg::LocalGrid>::SharedPtr grid_sub_;
  rclcpp::Publisher<mdog_interfaces::msg::Traversability>::SharedPtr trav_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MDogTraversabilityNode>());
  rclcpp::shutdown();
  return 0;
}
