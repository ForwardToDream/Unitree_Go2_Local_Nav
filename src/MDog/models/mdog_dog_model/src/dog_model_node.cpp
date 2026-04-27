#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "mdog_body_msgs/msg/body_envelope.hpp"
#include "rclcpp/rclcpp.hpp"

class MDogDogModelNode : public rclcpp::Node {
 public:
  MDogDogModelNode() : Node("mdog_dog_model") {
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    body_topic_ = declare_parameter<std::string>("body_topic", "/mdog/dog_body");
    enabled_ = declare_parameter<bool>("enabled", true);
    front_m_ = declare_parameter<double>("front_m", 0.75);
    back_m_ = declare_parameter<double>("back_m", 0.65);
    side_m_ = declare_parameter<double>("side_m", 0.45);
    min_z_ = declare_parameter<double>("min_z", -0.60);
    max_z_ = declare_parameter<double>("max_z", 0.80);
    publish_rate_ = declare_parameter<double>("publish_rate", 10.0);

    body_pub_ = create_publisher<mdog_body_msgs::msg::BodyEnvelope>(body_topic_, 10);
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        [this]() { publishBody(); });

    RCLCPP_INFO(get_logger(), "Dog body model -> %s, x=[-%.2f, %.2f], y=+-%.2f, z=[%.2f, %.2f]",
                body_topic_.c_str(), back_m_, front_m_, side_m_, min_z_, max_z_);
  }

 private:
  void publishBody() {
    mdog_body_msgs::msg::BodyEnvelope msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = frame_id_;
    msg.body_type = mdog_body_msgs::msg::BodyEnvelope::DOG;
    msg.name = "dog";
    msg.enabled = enabled_;
    msg.pose.position.x = (front_m_ - back_m_) * 0.5;
    msg.pose.position.y = 0.0;
    msg.pose.position.z = (min_z_ + max_z_) * 0.5;
    msg.pose.orientation.w = 1.0;
    msg.size.x = front_m_ + back_m_;
    msg.size.y = side_m_ * 2.0;
    msg.size.z = max_z_ - min_z_;
    body_pub_->publish(msg);
  }

  std::string frame_id_;
  std::string body_topic_;
  bool enabled_{true};
  double front_m_{};
  double back_m_{};
  double side_m_{};
  double min_z_{};
  double max_z_{};
  double publish_rate_{};

  rclcpp::Publisher<mdog_body_msgs::msg::BodyEnvelope>::SharedPtr body_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MDogDogModelNode>());
  rclcpp::shutdown();
  return 0;
}
