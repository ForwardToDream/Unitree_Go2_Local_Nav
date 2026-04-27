#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "mdog_intent_msgs/msg/owner_intent.hpp"
#include "mdog_planning_msgs/msg/nav_decision.hpp"
#include "mdog_planning_msgs/msg/nav_feedback.hpp"
#include "mdog_traversability_msgs/msg/traversability.hpp"
#include "rclcpp/rclcpp.hpp"

class MDogIntentArbitratorNode : public rclcpp::Node {
 public:
  MDogIntentArbitratorNode() : Node("mdog_intent_arbitrator") {
    intent_topic_ = declare_parameter<std::string>("intent_topic", "/mdog/owner_intent");
    traversability_topic_ = declare_parameter<std::string>("traversability_topic", "/mdog/traversability");
    decision_topic_ = declare_parameter<std::string>("decision_topic", "/mdog/nav_decision");
    feedback_topic_ = declare_parameter<std::string>("feedback_topic", "/mdog/nav_feedback");
    max_linear_x_ = declare_parameter<double>("max_linear_x", 0.25);
    max_lateral_y_ = declare_parameter<double>("max_lateral_y", 0.15);
    max_angular_z_ = declare_parameter<double>("max_angular_z", 0.6);
    adjusted_linear_x_ = declare_parameter<double>("adjusted_linear_x", 0.12);
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 0.5);
    publish_rate_ = declare_parameter<double>("publish_rate", 20.0);

    intent_sub_ = create_subscription<mdog_intent_msgs::msg::OwnerIntent>(
        intent_topic_, 10,
        [this](mdog_intent_msgs::msg::OwnerIntent::SharedPtr msg) { latest_intent_ = msg; });
    traversability_sub_ = create_subscription<mdog_traversability_msgs::msg::Traversability>(
        traversability_topic_, 10,
        [this](mdog_traversability_msgs::msg::Traversability::SharedPtr msg) {
          latest_traversability_ = msg;
        });
    decision_pub_ = create_publisher<mdog_planning_msgs::msg::NavDecision>(decision_topic_, 10);
    feedback_pub_ = create_publisher<mdog_planning_msgs::msg::NavFeedback>(feedback_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        [this]() { controlTick(); });

    RCLCPP_INFO(get_logger(), "Intent arbitrator: %s + %s -> %s",
                intent_topic_.c_str(), traversability_topic_.c_str(), decision_topic_.c_str());
  }

 private:
  void controlTick() {
    const auto now = get_clock()->now();
    if (!latest_intent_) {
      publishDecision(now, zeroTwist(), mdog_planning_msgs::msg::NavDecision::REJECTED,
                      mdog_planning_msgs::msg::NavFeedback::SENSOR_STALE, "waiting_for_owner_intent");
      return;
    }

    if (latest_intent_->command == mdog_intent_msgs::msg::OwnerIntent::STOP) {
      publishDecision(now, zeroTwist(), mdog_planning_msgs::msg::NavDecision::ACCEPTED,
                      mdog_planning_msgs::msg::NavFeedback::CLEAR, "owner_stop");
      return;
    }

    if (!latest_traversability_ || isStale(latest_traversability_->header.stamp, now)) {
      publishDecision(now, zeroTwist(), mdog_planning_msgs::msg::NavDecision::REJECTED,
                      mdog_planning_msgs::msg::NavFeedback::SENSOR_STALE, "traversability_stale");
      return;
    }

    const auto& intent = *latest_intent_;
    const auto& trav = *latest_traversability_;
    auto cmd = zeroTwist();
    uint8_t handling = mdog_planning_msgs::msg::NavDecision::ACCEPTED;
    uint8_t feedback = mdog_planning_msgs::msg::NavFeedback::CLEAR;
    std::string reason = "clear";

    const double strength = std::clamp(static_cast<double>(intent.strength), 0.0, 1.0);
    const double preferred_speed = intent.preferred_speed > 0.0 ? intent.preferred_speed : max_linear_x_;
    const double forward_speed = std::min(max_linear_x_, preferred_speed) * strength;

    switch (intent.command) {
      case mdog_intent_msgs::msg::OwnerIntent::FORWARD:
        planForward(trav, forward_speed, cmd, handling, feedback, reason);
        break;
      case mdog_intent_msgs::msg::OwnerIntent::LEFT:
        planLateral(trav.left_passable, max_lateral_y_ * strength, "left", cmd, handling, feedback, reason);
        break;
      case mdog_intent_msgs::msg::OwnerIntent::RIGHT:
        planLateral(trav.right_passable, -max_lateral_y_ * strength, "right", cmd, handling, feedback, reason);
        break;
      case mdog_intent_msgs::msg::OwnerIntent::TURN_LEFT:
        cmd.angular.z = max_angular_z_ * strength;
        reason = "turn_left";
        break;
      case mdog_intent_msgs::msg::OwnerIntent::TURN_RIGHT:
        cmd.angular.z = -max_angular_z_ * strength;
        reason = "turn_right";
        break;
      case mdog_intent_msgs::msg::OwnerIntent::BACK:
        cmd.linear.x = -std::min(0.10, max_linear_x_) * strength;
        handling = mdog_planning_msgs::msg::NavDecision::ADJUSTED;
        feedback = mdog_planning_msgs::msg::NavFeedback::ADJUSTING;
        reason = "back_cautious_unmapped_rear";
        break;
      default:
        handling = mdog_planning_msgs::msg::NavDecision::REJECTED;
        feedback = mdog_planning_msgs::msg::NavFeedback::BLOCKED;
        reason = "unknown_owner_intent";
        break;
    }

    publishDecision(now, cmd, handling, feedback, reason);
  }

  void planForward(
      const mdog_traversability_msgs::msg::Traversability& trav,
      double forward_speed,
      geometry_msgs::msg::Twist& cmd,
      uint8_t& handling,
      uint8_t& feedback,
      std::string& reason) const {
    if (trav.front_passable) {
      cmd.linear.x = forward_speed * std::clamp(static_cast<double>(trav.front_score), 0.3, 1.0);
      handling = mdog_planning_msgs::msg::NavDecision::ACCEPTED;
      feedback = trav.front_score < 0.65F ? mdog_planning_msgs::msg::NavFeedback::NARROW_PASSAGE
                                          : mdog_planning_msgs::msg::NavFeedback::CLEAR;
      reason = trav.front_score < 0.65F ? "front_narrow_slow" : "front_clear";
      return;
    }

    if (trav.left_passable && trav.recommended_heading_rad > 0.0F) {
      cmd.linear.x = adjusted_linear_x_;
      cmd.angular.z = std::min(max_angular_z_, std::abs(static_cast<double>(trav.recommended_heading_rad)));
      handling = mdog_planning_msgs::msg::NavDecision::ADJUSTED;
      feedback = mdog_planning_msgs::msg::NavFeedback::ADJUSTING;
      reason = "front_blocked_adjust_left";
      return;
    }

    if (trav.right_passable && trav.recommended_heading_rad < 0.0F) {
      cmd.linear.x = adjusted_linear_x_;
      cmd.angular.z = -std::min(max_angular_z_, std::abs(static_cast<double>(trav.recommended_heading_rad)));
      handling = mdog_planning_msgs::msg::NavDecision::ADJUSTED;
      feedback = mdog_planning_msgs::msg::NavFeedback::ADJUSTING;
      reason = "front_blocked_adjust_right";
      return;
    }

    handling = mdog_planning_msgs::msg::NavDecision::REJECTED;
    feedback = mdog_planning_msgs::msg::NavFeedback::BLOCKED;
    reason = "front_blocked";
  }

  void planLateral(
      bool passable,
      double lateral_speed,
      const std::string& side,
      geometry_msgs::msg::Twist& cmd,
      uint8_t& handling,
      uint8_t& feedback,
      std::string& reason) const {
    if (passable) {
      cmd.linear.y = lateral_speed;
      handling = mdog_planning_msgs::msg::NavDecision::ACCEPTED;
      feedback = mdog_planning_msgs::msg::NavFeedback::CLEAR;
      reason = side + "_clear";
      return;
    }
    handling = mdog_planning_msgs::msg::NavDecision::REJECTED;
    feedback = mdog_planning_msgs::msg::NavFeedback::BLOCKED;
    reason = side + "_blocked";
  }

  bool isStale(const builtin_interfaces::msg::Time& stamp, const rclcpp::Time& now) const {
    return (now - rclcpp::Time(stamp)).seconds() > stale_timeout_sec_;
  }

  geometry_msgs::msg::Twist zeroTwist() const {
    return geometry_msgs::msg::Twist();
  }

  void publishDecision(
      const rclcpp::Time& now,
      const geometry_msgs::msg::Twist& cmd,
      uint8_t handling,
      uint8_t feedback_status,
      const std::string& reason) {
    mdog_planning_msgs::msg::NavDecision decision;
    decision.header.stamp = now;
    decision.header.frame_id = "base_link";
    decision.intent_handling = handling;
    decision.cmd_vel = cmd;
    decision.reason = reason;
    decision_pub_->publish(decision);

    mdog_planning_msgs::msg::NavFeedback feedback;
    feedback.header = decision.header;
    feedback.status = feedback_status;
    feedback.message = reason;
    feedback_pub_->publish(feedback);
  }

  std::string intent_topic_;
  std::string traversability_topic_;
  std::string decision_topic_;
  std::string feedback_topic_;
  double max_linear_x_{};
  double max_lateral_y_{};
  double max_angular_z_{};
  double adjusted_linear_x_{};
  double stale_timeout_sec_{};
  double publish_rate_{};

  mdog_intent_msgs::msg::OwnerIntent::SharedPtr latest_intent_;
  mdog_traversability_msgs::msg::Traversability::SharedPtr latest_traversability_;
  rclcpp::Subscription<mdog_intent_msgs::msg::OwnerIntent>::SharedPtr intent_sub_;
  rclcpp::Subscription<mdog_traversability_msgs::msg::Traversability>::SharedPtr traversability_sub_;
  rclcpp::Publisher<mdog_planning_msgs::msg::NavDecision>::SharedPtr decision_pub_;
  rclcpp::Publisher<mdog_planning_msgs::msg::NavFeedback>::SharedPtr feedback_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MDogIntentArbitratorNode>());
  rclcpp::shutdown();
  return 0;
}
