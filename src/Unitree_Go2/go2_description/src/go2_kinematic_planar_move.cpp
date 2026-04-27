#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gazebo
{
class Go2KinematicPlanarMove : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    node_ = gazebo_ros::Node::Get(sdf);

    cmd_vel_topic_ = GetSdfString(sdf, "cmd_vel_topic", "/cmd_vel");
    base_frame_ = GetSdfString(sdf, "robot_base_frame", "base_link");
    cmd_timeout_ = GetSdfDouble(sdf, "cmd_timeout", 0.5);
    max_linear_ = GetSdfDouble(sdf, "max_linear", 1.0);
    max_angular_ = GetSdfDouble(sdf, "max_angular", 2.0);
    lock_z_ = GetSdfBool(sdf, "lock_z", true);

    const auto pose = model_->WorldPose();
    yaw_ = pose.Rot().Yaw();
    locked_z_ = sdf->HasElement("z") ? sdf->Get<double>("z") : pose.Pos().Z();
    last_update_time_ = model_->GetWorld()->SimTime();
    last_cmd_time_ = last_update_time_;

    model_->SetGravityMode(false);
    model_->SetAngularVel({0.0, 0.0, 0.0});
    model_->SetLinearVel({0.0, 0.0, 0.0});

    cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) { this->OnCmdVel(*msg); });

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      [this](const common::UpdateInfo & info) { this->OnUpdate(info); });

    RCLCPP_INFO(
      node_->get_logger(),
      "go2_kinematic_planar_move ready. cmd_vel=%s base=%s lock_z=%.3f",
      cmd_vel_topic_.c_str(), base_frame_.c_str(), locked_z_);
  }

private:
  static std::string GetSdfString(
    const sdf::ElementPtr & sdf, const std::string & name,
    const std::string & default_value)
  {
    return sdf->HasElement(name) ? sdf->Get<std::string>(name) : default_value;
  }

  static double GetSdfDouble(
    const sdf::ElementPtr & sdf, const std::string & name,
    const double default_value)
  {
    return sdf->HasElement(name) ? sdf->Get<double>(name) : default_value;
  }

  static bool GetSdfBool(
    const sdf::ElementPtr & sdf, const std::string & name,
    const bool default_value)
  {
    return sdf->HasElement(name) ? sdf->Get<bool>(name) : default_value;
  }

  void OnCmdVel(const geometry_msgs::msg::Twist & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_cmd_ = msg;
    last_cmd_time_ = model_->GetWorld()->SimTime();
  }

  void OnUpdate(const common::UpdateInfo & info)
  {
    if (!model_) {
      return;
    }

    const double dt = (info.simTime - last_update_time_).Double();
    last_update_time_ = info.simTime;
    if (dt <= 0.0) {
      return;
    }

    geometry_msgs::msg::Twist cmd;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool expired = (info.simTime - last_cmd_time_).Double() > cmd_timeout_;
      if (!expired) {
        cmd = latest_cmd_;
      }
    }

    const double vx_body = std::clamp(cmd.linear.x, -max_linear_, max_linear_);
    const double vy_body = std::clamp(cmd.linear.y, -max_linear_, max_linear_);
    const double wz = std::clamp(cmd.angular.z, -max_angular_, max_angular_);

    auto pose = model_->WorldPose();
    yaw_ += wz * dt;

    const double cos_yaw = std::cos(yaw_);
    const double sin_yaw = std::sin(yaw_);
    const double vx_world = cos_yaw * vx_body - sin_yaw * vy_body;
    const double vy_world = sin_yaw * vx_body + cos_yaw * vy_body;

    pose.Pos().X() += vx_world * dt;
    pose.Pos().Y() += vy_world * dt;
    if (lock_z_) {
      pose.Pos().Z() = locked_z_;
    }
    pose.Rot() = ignition::math::Quaterniond(0.0, 0.0, yaw_);

    model_->SetWorldPose(pose);
    model_->SetLinearVel({0.0, 0.0, 0.0});
    model_->SetAngularVel({0.0, 0.0, 0.0});
  }

  physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr node_;
  event::ConnectionPtr update_connection_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

  std::mutex mutex_;
  geometry_msgs::msg::Twist latest_cmd_;
  common::Time last_update_time_;
  common::Time last_cmd_time_;

  std::string cmd_vel_topic_;
  std::string base_frame_;
  double cmd_timeout_{0.5};
  double max_linear_{1.0};
  double max_angular_{2.0};
  double locked_z_{0.0};
  double yaw_{0.0};
  bool lock_z_{true};
};

GZ_REGISTER_MODEL_PLUGIN(Go2KinematicPlanarMove)
}  // namespace gazebo
