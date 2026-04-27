#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "mdog_interfaces/msg/nav_decision.hpp"
#include "mdog_interfaces/msg/nav_feedback.hpp"
#include "mdog_interfaces/msg/owner_intent.hpp"
#include "mdog_interfaces/msg/traversability.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MDogLocalPlannerNode : public rclcpp::Node {
 public:
  MDogLocalPlannerNode() : Node("mdog_local_planner") {
    intent_topic_ = declare_parameter<std::string>("intent_topic", "/mdog/owner_intent");
    traversability_topic_ = declare_parameter<std::string>("traversability_topic", "/mdog/traversability");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    decision_topic_ = declare_parameter<std::string>("decision_topic", "/mdog/nav_decision");
    feedback_topic_ = declare_parameter<std::string>("feedback_topic", "/mdog/nav_feedback");
    max_linear_x_ = declare_parameter<double>("max_linear_x", 0.25);
    max_lateral_y_ = declare_parameter<double>("max_lateral_y", 0.15);
    max_angular_z_ = declare_parameter<double>("max_angular_z", 0.6);
    adjusted_linear_x_ = declare_parameter<double>("adjusted_linear_x", 0.12);
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 0.5);
    publish_rate_ = declare_parameter<double>("publish_rate", 20.0);

    intent_sub_ = create_subscription<mdog_interfaces::msg::OwnerIntent>(
        intent_topic_, 10, [this](mdog_interfaces::msg::OwnerIntent::SharedPtr msg) {
          latest_intent_ = msg;
        });
    traversability_sub_ = create_subscription<mdog_interfaces::msg::Traversability>(
        traversability_topic_, 10,
        [this](mdog_interfaces::msg::Traversability::SharedPtr msg) {
          latest_traversability_ = msg;
        });

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    decision_pub_ = create_publisher<mdog_interfaces::msg::NavDecision>(decision_topic_, 10);
    feedback_pub_ = create_publisher<mdog_interfaces::msg::NavFeedback>(feedback_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        [this]() { controlTick(); });

    RCLCPP_INFO(get_logger(), "MDog local planner: %s + %s -> %s",
                intent_topic_.c_str(), traversability_topic_.c_str(), cmd_vel_topic_.c_str());
  }

 private:
  void controlTick() {
    const auto now = get_clock()->now();
    if (!latest_intent_) {
      publishDecision(now, zeroTwist(), mdog_interfaces::msg::NavDecision::REJECTED,
                      mdog_interfaces::msg::NavFeedback::SENSOR_STALE, "waiting_for_owner_intent");
      return;
    }

    if (latest_intent_->command == mdog_interfaces::msg::OwnerIntent::STOP) {
      publishDecision(now, zeroTwist(), mdog_interfaces::msg::NavDecision::ACCEPTED,
                      mdog_interfaces::msg::NavFeedback::CLEAR, "owner_stop");
      return;
    }

    if (!latest_traversability_ || isStale(latest_traversability_->header.stamp, now)) {
      publishDecision(now, zeroTwist(), mdog_interfaces::msg::NavDecision::REJECTED,
                      mdog_interfaces::msg::NavFeedback::SENSOR_STALE, "traversability_stale");
      return;
    }

    const auto& intent = *latest_intent_;
    const auto& trav = *latest_traversability_;
    auto cmd = zeroTwist();
    uint8_t handling = mdog_interfaces::msg::NavDecision::ACCEPTED;
    uint8_t feedback = mdog_interfaces::msg::NavFeedback::CLEAR;
    std::string reason = "clear";

    const double strength = std::clamp(static_cast<double>(intent.strength), 0.0, 1.0);
    const double preferred_speed = intent.preferred_speed > 0.0 ? intent.preferred_speed : max_linear_x_;
    const double forward_speed = std::min(max_linear_x_, preferred_speed) * strength;

    switch (intent.command) {
      case mdog_interfaces::msg::OwnerIntent::FORWARD:
        planForward(trav, forward_speed, cmd, handling, feedback, reason);
        break;
      case mdog_interfaces::msg::OwnerIntent::LEFT:
        planLateral(trav.left_passable, max_lateral_y_ * strength, "left", cmd, handling, feedback, reason);
        break;
      case mdog_interfaces::msg::OwnerIntent::RIGHT:
        planLateral(trav.right_passable, -max_lateral_y_ * strength, "right", cmd, handling, feedback, reason);
        break;
      case mdog_interfaces::msg::OwnerIntent::TURN_LEFT:
        cmd.angular.z = max_angular_z_ * strength;
        reason = "turn_left";
        break;
      case mdog_interfaces::msg::OwnerIntent::TURN_RIGHT:
        cmd.angular.z = -max_angular_z_ * strength;
        reason = "turn_right";
        break;
      case mdog_interfaces::msg::OwnerIntent::BACK:
        cmd.linear.x = -std::min(0.10, max_linear_x_) * strength;
        handling = mdog_interfaces::msg::NavDecision::ADJUSTED;
        feedback = mdog_interfaces::msg::NavFeedback::ADJUSTING;
        reason = "back_cautious_unmapped_rear";
        break;
      default:
        handling = mdog_interfaces::msg::NavDecision::REJECTED;
        feedback = mdog_interfaces::msg::NavFeedback::BLOCKED;
        reason = "unknown_owner_intent";
        break;
    }

    publishDecision(now, cmd, handling, feedback, reason);
  }

  void planForward(
      const mdog_interfaces::msg::Traversability& trav, double forward_speed,
      geometry_msgs::msg::Twist& cmd, uint8_t& handling, uint8_t& feedback,
      std::string& reason) const {
    if (trav.front_passable) {
      cmd.linear.x = forward_speed * std::clamp(static_cast<double>(trav.front_score), 0.3, 1.0);
      handling = mdog_interfaces::msg::NavDecision::ACCEPTED;
      feedback = trav.front_score < 0.65F ? mdog_interfaces::msg::NavFeedback::NARROW_PASSAGE
                                          : mdog_interfaces::msg::NavFeedback::CLEAR;
      reason = trav.front_score < 0.65F ? "front_narrow_slow" : "front_clear";
      return;
    }

    if (trav.left_passable && trav.recommended_heading_rad > 0.0F) {
      cmd.linear.x = adjusted_linear_x_;
      cmd.angular.z = std::min(max_angular_z_, std::abs(static_cast<double>(trav.recommended_heading_rad)));
      handling = mdog_interfaces::msg::NavDecision::ADJUSTED;
      feedback = mdog_interfaces::msg::NavFeedback::ADJUSTING;
      reason = "front_blocked_adjust_left";
      return;
    }

    if (trav.right_passable && trav.recommended_heading_rad < 0.0F) {
      cmd.linear.x = adjusted_linear_x_;
      cmd.angular.z = -std::min(max_angular_z_, std::abs(static_cast<double>(trav.recommended_heading_rad)));
      handling = mdog_interfaces::msg::NavDecision::ADJUSTED;
      feedback = mdog_interfaces::msg::NavFeedback::ADJUSTING;
      reason = "front_blocked_adjust_right";
      return;
    }

    handling = mdog_interfaces::msg::NavDecision::REJECTED;
    feedback = mdog_interfaces::msg::NavFeedback::BLOCKED;
    reason = "front_blocked";
  }

  void planLateral(
      bool passable, double lateral_speed, const std::string& side,
      geometry_msgs::msg::Twist& cmd, uint8_t& handling, uint8_t& feedback,
      std::string& reason) const {
    if (passable) {
      cmd.linear.y = lateral_speed;
      handling = mdog_interfaces::msg::NavDecision::ACCEPTED;
      feedback = mdog_interfaces::msg::NavFeedback::CLEAR;
      reason = side + "_clear";
      return;
    }
    handling = mdog_interfaces::msg::NavDecision::REJECTED;
    feedback = mdog_interfaces::msg::NavFeedback::BLOCKED;
    reason = side + "_blocked";
  }

  bool isStale(const builtin_interfaces::msg::Time& stamp, const rclcpp::Time& now) const {
    return (now - rclcpp::Time(stamp)).seconds() > stale_timeout_sec_;
  }

  geometry_msgs::msg::Twist zeroTwist() const {
    return geometry_msgs::msg::Twist();
  }

  void publishDecision(
      const rclcpp::Time& now, const geometry_msgs::msg::Twist& cmd,
      uint8_t handling, uint8_t feedback_status, const std::string& reason) {
    cmd_pub_->publish(cmd);

    mdog_interfaces::msg::NavDecision decision;
    decision.header.stamp = now;
    decision.header.frame_id = "base_link";
    decision.intent_handling = handling;
    decision.cmd_vel = cmd;
    decision.reason = reason;
    decision_pub_->publish(decision);

    mdog_interfaces::msg::NavFeedback feedback;
    feedback.header = decision.header;
    feedback.status = feedback_status;
    feedback.message = reason;
    feedback_pub_->publish(feedback);
  }

  std::string intent_topic_;
  std::string traversability_topic_;
  std::string cmd_vel_topic_;
  std::string decision_topic_;
  std::string feedback_topic_;
  double max_linear_x_{};
  double max_lateral_y_{};
  double max_angular_z_{};
  double adjusted_linear_x_{};
  double stale_timeout_sec_{};
  double publish_rate_{};

  mdog_interfaces::msg::OwnerIntent::SharedPtr latest_intent_;
  mdog_interfaces::msg::Traversability::SharedPtr latest_traversability_;
  rclcpp::Subscription<mdog_interfaces::msg::OwnerIntent>::SharedPtr intent_sub_;
  rclcpp::Subscription<mdog_interfaces::msg::Traversability>::SharedPtr traversability_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<mdog_interfaces::msg::NavDecision>::SharedPtr decision_pub_;
  rclcpp::Publisher<mdog_interfaces::msg::NavFeedback>::SharedPtr feedback_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MDogLocalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
