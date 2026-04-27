#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "common/ros2_sport_client.h"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_go/msg/wireless_controller.hpp"

class GuideDogManager : public rclcpp::Node {
 public:
  GuideDogManager() : Node("guide_dog_manager"), sport_client_(this) {
    motion_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
        "lf/sportmodestate", 10,
        [this](const unitree_go::msg::SportModeState::SharedPtr msg) {
          motion_state_ = *msg;
        });

    remote_sub_ = this->create_subscription<unitree_go::msg::WirelessController>(
        "/wirelesscontroller", 10,
        [this](const unitree_go::msg::WirelessController::SharedPtr msg) {
          remote_state_ = *msg;
        });

    // The control loop is intentionally conservative and only emits logs.
    // You can evolve it into: perception -> planner -> motion command output.
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), [this]() { ControlLoop(); });
  }

 private:
  void ControlLoop() {
    // TODO: replace this with your guide-dog navigation logic.
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Guide-dog manager alive | pos=(%.2f, %.2f) vel=(%.2f, %.2f) keys=%d",
        motion_state_.position[0], motion_state_.position[1],
        motion_state_.velocity[0], motion_state_.velocity[1],
        remote_state_.keys);

    // Example placeholder:
    // unitree_api::msg::Request req;
    // sport_client_.Move(req, 0.2F, 0.0F, 0.0F);
  }

  SportClient sport_client_;
  unitree_go::msg::SportModeState motion_state_{};
  unitree_go::msg::WirelessController remote_state_{};

  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr motion_sub_;
  rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr
      remote_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GuideDogManager>());
  rclcpp::shutdown();
  return 0;
}
