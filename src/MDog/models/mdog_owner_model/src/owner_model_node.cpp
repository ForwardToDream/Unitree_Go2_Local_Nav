#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "mdog_body_msgs/msg/body_envelope.hpp"
#include "rclcpp/rclcpp.hpp"

class MDogOwnerModelNode : public rclcpp::Node {
 public:
  MDogOwnerModelNode() : Node("mdog_owner_model") {
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    body_topic_ = declare_parameter<std::string>("body_topic", "/mdog/owner_body");
    enabled_ = declare_parameter<bool>("enabled", true);
    dog_rear_x_m_ = declare_parameter<double>("dog_rear_x_m", -0.65);
    follow_gap_m_ = declare_parameter<double>("follow_gap_m", 0.30);
    owner_depth_m_ = declare_parameter<double>("owner_depth_m", 0.35);
    owner_width_m_ = declare_parameter<double>("owner_width_m", 0.80);
    owner_height_m_ = declare_parameter<double>("owner_height_m", 1.80);
    owner_ground_z_m_ = declare_parameter<double>("owner_ground_z_m", -0.35);
    publish_rate_ = declare_parameter<double>("publish_rate", 10.0);

    body_pub_ = create_publisher<mdog_body_msgs::msg::BodyEnvelope>(body_topic_, 10);
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        [this]() { publishBody(); });

    RCLCPP_INFO(get_logger(), "Owner body model -> %s, gap=%.2fm behind dog", body_topic_.c_str(), follow_gap_m_);
  }

 private:
  void publishBody() {
    const double owner_front_x = dog_rear_x_m_ - follow_gap_m_;

    mdog_body_msgs::msg::BodyEnvelope msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = frame_id_;
    msg.body_type = mdog_body_msgs::msg::BodyEnvelope::OWNER;
    msg.name = "owner";
    msg.enabled = enabled_;
    msg.pose.position.x = owner_front_x - owner_depth_m_ * 0.5;
    msg.pose.position.y = 0.0;
    msg.pose.position.z = owner_ground_z_m_ + owner_height_m_ * 0.5;
    msg.pose.orientation.w = 1.0;
    msg.size.x = owner_depth_m_;
    msg.size.y = owner_width_m_;
    msg.size.z = owner_height_m_;
    body_pub_->publish(msg);
  }

  std::string frame_id_;
  std::string body_topic_;
  bool enabled_{true};
  double dog_rear_x_m_{};
  double follow_gap_m_{};
  double owner_depth_m_{};
  double owner_width_m_{};
  double owner_height_m_{};
  double owner_ground_z_m_{};
  double publish_rate_{};

  rclcpp::Publisher<mdog_body_msgs::msg::BodyEnvelope>::SharedPtr body_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MDogOwnerModelNode>());
  rclcpp::shutdown();
  return 0;
}
