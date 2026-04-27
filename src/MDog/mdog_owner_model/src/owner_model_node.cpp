#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class MDogOwnerModelNode : public rclcpp::Node {
 public:
  MDogOwnerModelNode() : Node("mdog_owner_model") {
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/mdog/owner_envelope");
    dog_rear_x_m_ = declare_parameter<double>("dog_rear_x_m", -0.65);
    follow_gap_m_ = declare_parameter<double>("follow_gap_m", 0.30);
    owner_depth_m_ = declare_parameter<double>("owner_depth_m", 0.35);
    owner_width_m_ = declare_parameter<double>("owner_width_m", 0.80);
    owner_height_m_ = declare_parameter<double>("owner_height_m", 1.80);
    owner_ground_z_m_ = declare_parameter<double>("owner_ground_z_m", -0.35);
    publish_rate_ = declare_parameter<double>("publish_rate", 10.0);

    marker_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    const auto period =
        std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        [this]() { publishOwnerEnvelope(); });

    RCLCPP_INFO(
        get_logger(),
        "MDog owner model: owner front %.2fm behind dog rear %.2fm, size=%.2fx%.2fx%.2f, topic=%s",
        follow_gap_m_, dog_rear_x_m_, owner_depth_m_, owner_width_m_,
        owner_height_m_, marker_topic_.c_str());
  }

 private:
  void publishOwnerEnvelope() {
    const auto stamp = get_clock()->now();
    const double owner_front_x = dog_rear_x_m_ - follow_gap_m_;
    const double owner_center_x = owner_front_x - owner_depth_m_ * 0.5;

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker box;
    box.header.stamp = stamp;
    box.header.frame_id = frame_id_;
    box.ns = "owner_envelope";
    box.id = 0;
    box.type = visualization_msgs::msg::Marker::CUBE;
    box.action = visualization_msgs::msg::Marker::ADD;
    box.pose.position.x = owner_center_x;
    box.pose.position.y = 0.0;
    box.pose.position.z = owner_ground_z_m_ + owner_height_m_ * 0.5;
    box.pose.orientation.w = 1.0;
    box.scale.x = owner_depth_m_;
    box.scale.y = owner_width_m_;
    box.scale.z = owner_height_m_;
    box.color.r = 0.2F;
    box.color.g = 1.0F;
    box.color.b = 0.35F;
    box.color.a = 0.18F;
    box.lifetime = rclcpp::Duration::from_seconds(0.5);
    markers.markers.push_back(box);

    visualization_msgs::msg::Marker text;
    text.header = box.header;
    text.ns = "owner_envelope_label";
    text.id = 1;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = owner_center_x;
    text.pose.position.y = 0.0;
    text.pose.position.z = owner_ground_z_m_ + owner_height_m_ + 0.15;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.16;
    text.color.r = 0.2F;
    text.color.g = 1.0F;
    text.color.b = 0.35F;
    text.color.a = 1.0F;
    text.text = "owner envelope";
    text.lifetime = rclcpp::Duration::from_seconds(0.5);
    markers.markers.push_back(text);

    marker_pub_->publish(markers);
  }

  std::string frame_id_;
  std::string marker_topic_;
  double dog_rear_x_m_{};
  double follow_gap_m_{};
  double owner_depth_m_{};
  double owner_width_m_{};
  double owner_height_m_{};
  double owner_ground_z_m_{};
  double publish_rate_{};

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MDogOwnerModelNode>());
  rclcpp::shutdown();
  return 0;
}
