#include <chrono>
#include <algorithm>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "sdk2_robot_control.h"

namespace legged_control
{

class SDK2ControlNode : public rclcpp::Node {
public:
    SDK2ControlNode()
    : rclcpp::Node("go2_node"),
      last_cmd_time_(this->now()),
      last_vx_(0.0), last_vy_(0.0), last_wz_(0.0),
      sent_stop_(false) {
        // Parameters
        std::string network_interface = this->declare_parameter<std::string>("network_interface", "eth0");
        int control_rate_hz = this->declare_parameter<int>("control_rate_hz", 50);
        bool auto_stand = this->declare_parameter<bool>("auto_stand", true);

        max_vx_ = this->declare_parameter<float>("max_vx", 1.5);
        max_vy_ = this->declare_parameter<float>("max_vy", 0.5);
        max_wz_ = this->declare_parameter<float>("max_wz", 1.5);
        stale_timeout_s_ = this->declare_parameter<float>("stale_timeout_s", 0.5);

        std::string config_path = this->declare_parameter<std::string>("config_path", "");
        // Instantiate the main controller directly
        robot_control_ = std::make_unique<SDK2RobotControl>(network_interface, auto_stand, config_path);

        // Subscribe to velocity commands
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::QoS(10),
            std::bind(&SDK2ControlNode::onTwist, this, std::placeholders::_1));

        // Control timer
        using namespace std::chrono_literals;
        auto period = std::chrono::microseconds(static_cast<int64_t>(1'000'000 / std::max(1, control_rate_hz)));
        control_timer_ = this->create_wall_timer(
            period, std::bind(&SDK2ControlNode::controlLoop, this));

        RCLCPP_INFO(get_logger(), "Go2/W Control Node started. rate=%dHz, stale_timeout=%.2fs", control_rate_hz, stale_timeout_s_);
    }

    ~SDK2ControlNode() override {
        RCLCPP_INFO(this->get_logger(), "Go2/W Control Node shutting down. Performing cleanup.");
        if (robot_control_) {
            robot_control_->shutdown();
        }
    }

private:
    void onTwist(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // ROS2 messages use double, so we cast to float for our internal variables.
        last_vx_ = std::max(-max_vx_, std::min(static_cast<float>(msg->linear.x), max_vx_));
        last_vy_ = std::max(-max_vy_, std::min(static_cast<float>(msg->linear.y), max_vy_));
        last_wz_ = std::max(-max_wz_, std::min(static_cast<float>(msg->angular.z), max_wz_));
        last_cmd_time_ = this->now();
        sent_stop_ = false;
    }

    void controlLoop() {
        const auto now = this->now();
        const float dt = (now - last_cmd_time_).seconds();

        if (dt <= stale_timeout_s_) {
            // If commands are fresh, send them to the robot controller
            robot_control_->processVelCmd(last_vx_, last_vy_, last_wz_);
        } else {
            // If commands are stale, send a stop command once
            if (!sent_stop_) {
                robot_control_->processVelCmd(0.0f, 0.0f, 0.0f);
                sent_stop_ = true;
            }
        }
    }

    std::unique_ptr<SDK2RobotControl> robot_control_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    rclcpp::Time last_cmd_time_;
    float last_vx_, last_vy_, last_wz_;
    bool sent_stop_;
    float stale_timeout_s_;
    float max_vx_, max_vy_, max_wz_;
};

}  // namespace legged_control

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<legged_control::SDK2ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}