#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "mdog_body_msgs/msg/body_envelope.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class MDogBodyMarkerVizNode : public rclcpp::Node {
 public:
  MDogBodyMarkerVizNode() : Node("mdog_body_marker_viz") {
    dog_body_topic_ = declare_parameter<std::string>("dog_body_topic", "/mdog/dog_body");
    owner_body_topic_ = declare_parameter<std::string>("owner_body_topic", "/mdog/owner_body");
    output_topic_ = declare_parameter<std::string>("output_topic", "/mdog/body_envelopes");
    publish_rate_ = declare_parameter<double>("publish_rate", 10.0);

    dog_sub_ = create_subscription<mdog_body_msgs::msg::BodyEnvelope>(
        dog_body_topic_, 10,
        [this](mdog_body_msgs::msg::BodyEnvelope::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          dog_body_ = msg;
        });
    owner_sub_ = create_subscription<mdog_body_msgs::msg::BodyEnvelope>(
        owner_body_topic_, 10,
        [this](mdog_body_msgs::msg::BodyEnvelope::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          owner_body_ = msg;
        });
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(output_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        [this]() { publishMarkers(); });

    RCLCPP_INFO(get_logger(), "Body marker viz -> %s", output_topic_.c_str());
  }

 private:
  void publishMarkers() {
    mdog_body_msgs::msg::BodyEnvelope::SharedPtr dog;
    mdog_body_msgs::msg::BodyEnvelope::SharedPtr owner;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      dog = dog_body_;
      owner = owner_body_;
    }

    visualization_msgs::msg::MarkerArray markers;
    const auto now = get_clock()->now();
    if (dog && dog->enabled) {
      markers.markers.push_back(makeBox(*dog, now, 0, 0.10F, 0.45F, 1.00F, 0.20F));
      markers.markers.push_back(makeText(*dog, now, 10, "dog body"));
    }
    if (owner && owner->enabled) {
      markers.markers.push_back(makeBox(*owner, now, 1, 0.10F, 0.95F, 0.45F, 0.18F));
      markers.markers.push_back(makeText(*owner, now, 11, "owner envelope"));
    }
    marker_pub_->publish(markers);
  }

  visualization_msgs::msg::Marker makeBox(
      const mdog_body_msgs::msg::BodyEnvelope& body,
      const rclcpp::Time& stamp,
      int id,
      float r,
      float g,
      float b,
      float a) const {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = body.header.frame_id;
    marker.ns = "mdog_body_envelopes";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = body.pose;
    marker.scale = body.size;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    return marker;
  }

  visualization_msgs::msg::Marker makeText(
      const mdog_body_msgs::msg::BodyEnvelope& body,
      const rclcpp::Time& stamp,
      int id,
      const std::string& text) const {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = body.header.frame_id;
    marker.ns = "mdog_body_labels";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = body.pose;
    marker.pose.position.z += body.size.z * 0.5 + 0.15;
    marker.scale.z = 0.16;
    marker.color.r = 1.0F;
    marker.color.g = 1.0F;
    marker.color.b = 1.0F;
    marker.color.a = 0.95F;
    marker.text = text;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    return marker;
  }

  std::string dog_body_topic_;
  std::string owner_body_topic_;
  std::string output_topic_;
  double publish_rate_{};

  std::mutex mutex_;
  mdog_body_msgs::msg::BodyEnvelope::SharedPtr dog_body_;
  mdog_body_msgs::msg::BodyEnvelope::SharedPtr owner_body_;
  rclcpp::Subscription<mdog_body_msgs::msg::BodyEnvelope>::SharedPtr dog_sub_;
  rclcpp::Subscription<mdog_body_msgs::msg::BodyEnvelope>::SharedPtr owner_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MDogBodyMarkerVizNode>());
  rclcpp::shutdown();
  return 0;
}
