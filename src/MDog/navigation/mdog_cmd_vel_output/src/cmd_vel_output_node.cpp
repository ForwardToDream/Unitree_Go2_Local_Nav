#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "mdog_planning_msgs/msg/nav_decision.hpp"
#include "rclcpp/rclcpp.hpp"

class MDogCmdVelOutputNode : public rclcpp::Node {
 public:
  MDogCmdVelOutputNode() : Node("mdog_cmd_vel_output") {
    decision_topic_ = declare_parameter<std::string>("decision_topic", "/mdog/nav_decision");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 0.5);
    publish_rate_ = declare_parameter<double>("publish_rate", 20.0);

    decision_sub_ = create_subscription<mdog_planning_msgs::msg::NavDecision>(
        decision_topic_, 10,
        [this](mdog_planning_msgs::msg::NavDecision::SharedPtr msg) {
          latest_decision_ = msg;
          latest_received_at_ = get_clock()->now();
        });
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        [this]() { publishCmd(); });

    RCLCPP_INFO(get_logger(), "Cmd vel output: %s -> %s", decision_topic_.c_str(), cmd_vel_topic_.c_str());
  }

 private:
  void publishCmd() {
    geometry_msgs::msg::Twist cmd;
    const auto now = get_clock()->now();
    if (latest_decision_ && (now - latest_received_at_).seconds() <= stale_timeout_sec_) {
      cmd = latest_decision_->cmd_vel;
    }
    cmd_pub_->publish(cmd);
  }

  std::string decision_topic_;
  std::string cmd_vel_topic_;
  double stale_timeout_sec_{};
  double publish_rate_{};
  rclcpp::Time latest_received_at_{0, 0, RCL_ROS_TIME};
  mdog_planning_msgs::msg::NavDecision::SharedPtr latest_decision_;
  rclcpp::Subscription<mdog_planning_msgs::msg::NavDecision>::SharedPtr decision_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MDogCmdVelOutputNode>());
  rclcpp::shutdown();
  return 0;
}
