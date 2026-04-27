#include <algorithm>
#include <chrono>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"

namespace {
constexpr int64_t kSportMoveApiId = 1008;
constexpr int64_t kSportStopApiId = 1003;
}

class Go2CmdVelBridge : public rclcpp::Node {
 public:
  Go2CmdVelBridge() : Node("go2_cmd_vel_bridge") {
    cmd_vel_topic_ =
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    request_topic_ = this->declare_parameter<std::string>(
        "sport_request_topic", "/api/sport/request");
    stop_timeout_sec_ =
        this->declare_parameter<double>("stop_timeout_sec", 0.5);
    max_vx_ = this->declare_parameter<double>("max_vx", 0.8);
    max_vy_ = this->declare_parameter<double>("max_vy", 0.5);
    max_wz_ = this->declare_parameter<double>("max_wz", 1.5);

    request_pub_ =
        this->create_publisher<unitree_api::msg::Request>(request_topic_, 10);
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, 50,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          OnCmdVel(*msg);
        });

    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), [this]() { WatchdogTick(); });

    last_cmd_time_ = this->now();
    RCLCPP_INFO(this->get_logger(),
                "go2_cmd_vel_bridge ready. cmd_vel: %s -> request: %s",
                cmd_vel_topic_.c_str(), request_topic_.c_str());
  }

 private:
  void OnCmdVel(const geometry_msgs::msg::Twist& cmd) {
    last_cmd_ = cmd;
    last_cmd_time_ = this->now();
    sent_stop_ = false;
    PublishMove(last_cmd_);
  }

  void WatchdogTick() {
    const double dt = (this->now() - last_cmd_time_).seconds();
    if (dt > stop_timeout_sec_ && !sent_stop_) {
      PublishStop();
      sent_stop_ = true;
    }
  }

  void PublishMove(const geometry_msgs::msg::Twist& cmd) {
    const double vx = std::clamp(cmd.linear.x, -max_vx_, max_vx_);
    const double vy = std::clamp(cmd.linear.y, -max_vy_, max_vy_);
    const double wz = std::clamp(cmd.angular.z, -max_wz_, max_wz_);

    unitree_api::msg::Request req;
    req.header.identity.id = this->now().nanoseconds();
    req.header.identity.api_id = kSportMoveApiId;

    std::ostringstream oss;
    oss << "{\"x\":" << vx << ",\"y\":" << vy << ",\"z\":" << wz << "}";
    req.parameter = oss.str();
    request_pub_->publish(req);
  }

  void PublishStop() {
    unitree_api::msg::Request req;
    req.header.identity.id = this->now().nanoseconds();
    req.header.identity.api_id = kSportStopApiId;
    request_pub_->publish(req);
  }

  std::string cmd_vel_topic_;
  std::string request_topic_;
  double stop_timeout_sec_{};
  double max_vx_{};
  double max_vy_{};
  double max_wz_{};
  bool sent_stop_{true};

  rclcpp::Time last_cmd_time_;
  geometry_msgs::msg::Twist last_cmd_;

  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2CmdVelBridge>());
  rclcpp::shutdown();
  return 0;
}
