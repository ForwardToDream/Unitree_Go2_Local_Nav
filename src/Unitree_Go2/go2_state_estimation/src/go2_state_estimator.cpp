#include <algorithm>
#include <chrono>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "unitree_go/msg/sport_mode_state.hpp"

class Go2StateEstimator : public rclcpp::Node {
 public:
  Go2StateEstimator() : Node("go2_state_estimator") {
    odom_source_ =
        this->declare_parameter<std::string>("odom_source", "cmd_vel");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/go2/odom");
    cmd_vel_topic_ =
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    sport_state_topic_ = this->declare_parameter<std::string>(
        "sport_state_topic", "lf/sportmodestate");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ =
        this->declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
    cmd_timeout_sec_ = this->declare_parameter<double>("cmd_timeout_sec", 0.5);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 20);
    tf_broadcaster_ =
        std::make_shared<tf2_ros::TransformBroadcaster>(this);

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, 50, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          latest_cmd_ = *msg;
          last_cmd_time_ = this->now();
        });

    sport_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
        sport_state_topic_, 20,
        [this](const unitree_go::msg::SportModeState::SharedPtr msg) {
          OnSportState(*msg);
        });

    last_update_time_ = this->now();
    last_cmd_time_ = this->now();
    update_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), [this]() { UpdateFromCmdVel(); });

    RCLCPP_INFO(this->get_logger(),
                "go2_state_estimator ready. source=%s odom=%s frame=%s->%s",
                odom_source_.c_str(), odom_topic_.c_str(), odom_frame_.c_str(),
                base_frame_.c_str());
  }

 private:
  void UpdateFromCmdVel() {
    if (odom_source_ != "cmd_vel") {
      return;
    }

    const rclcpp::Time now = this->now();
    const double dt = std::max((now - last_update_time_).seconds(), 0.0);
    last_update_time_ = now;
    if (dt <= 0.0) {
      return;
    }

    const bool cmd_expired = (now - last_cmd_time_).seconds() > cmd_timeout_sec_;
    const double vx_body = cmd_expired ? 0.0 : latest_cmd_.linear.x;
    const double vy_body = cmd_expired ? 0.0 : latest_cmd_.linear.y;
    const double wz = cmd_expired ? 0.0 : latest_cmd_.angular.z;

    const double cos_yaw = std::cos(yaw_);
    const double sin_yaw = std::sin(yaw_);
    const double vx_world = cos_yaw * vx_body - sin_yaw * vy_body;
    const double vy_world = sin_yaw * vx_body + cos_yaw * vy_body;

    x_ += vx_world * dt;
    y_ += vy_world * dt;
    yaw_ += wz * dt;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);
    q.normalize();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = z_;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = vx_body;
    odom.twist.twist.linear.y = vy_body;
    odom.twist.twist.angular.z = wz;
    odom_pub_->publish(odom);

    PublishTf(now, q.x(), q.y(), q.z(), q.w());
  }

  void OnSportState(const unitree_go::msg::SportModeState& msg) {
    if (odom_source_ != "sport_mode_state") {
      return;
    }

    const rclcpp::Time now = this->now();
    x_ = msg.position[0];
    y_ = msg.position[1];
    z_ = msg.position[2];
    yaw_ = msg.imu_state.rpy[2];

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = z_;
    odom.pose.pose.orientation.x = msg.imu_state.quaternion[1];
    odom.pose.pose.orientation.y = msg.imu_state.quaternion[2];
    odom.pose.pose.orientation.z = msg.imu_state.quaternion[3];
    odom.pose.pose.orientation.w = msg.imu_state.quaternion[0];
    odom.twist.twist.linear.x = msg.velocity[0];
    odom.twist.twist.linear.y = msg.velocity[1];
    odom.twist.twist.linear.z = msg.velocity[2];
    odom.twist.twist.angular.z = msg.yaw_speed;
    odom_pub_->publish(odom);

    PublishTf(now, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
              odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
  }

  void PublishTf(const rclcpp::Time& stamp, double qx, double qy, double qz,
                 double qw) {
    if (!publish_tf_) {
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = z_;
    tf_msg.transform.rotation.x = qx;
    tf_msg.transform.rotation.y = qy;
    tf_msg.transform.rotation.z = qz;
    tf_msg.transform.rotation.w = qw;
    tf_broadcaster_->sendTransform(tf_msg);
  }

  std::string odom_source_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string sport_state_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_{true};
  double cmd_timeout_sec_{};

  double x_{0.0};
  double y_{0.0};
  double z_{0.0};
  double yaw_{0.0};

  geometry_msgs::msg::Twist latest_cmd_;
  rclcpp::Time last_update_time_;
  rclcpp::Time last_cmd_time_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_sub_;
  rclcpp::TimerBase::SharedPtr update_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2StateEstimator>());
  rclcpp::shutdown();
  return 0;
}
