#include <chrono>
#include <algorithm>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "sdk2/sdk2_robot_control.h"

namespace unitree_control
{

class VelocityNode : public rclcpp::Node
{
public:
    VelocityNode()
    : rclcpp::Node("velocity_node"),
      last_cmd_time_(this->now()),
      last_vx_(0.0), last_vy_(0.0), last_wz_(0.0),
      sent_stop_(false), high_level_(false)
    {
        // Parameters
        std::string network_interface = this->declare_parameter<std::string>("network_interface", "eth0");
        double timeout_s = this->declare_parameter<double>("timeout_s", 20.0);
        int control_rate_hz = this->declare_parameter<int>("control_rate_hz", 50);
        bool auto_stand = this->declare_parameter<bool>("auto_stand", true);

        max_vx_ = this->declare_parameter<double>("max_vx", 1.5);
        max_vy_ = this->declare_parameter<double>("max_vy", 0.5);
        max_wz_ = this->declare_parameter<double>("max_wz", 1.5);
        stale_timeout_s_ = this->declare_parameter<double>("stale_timeout_s", 0.5);

        std::string config_path = this->declare_parameter<std::string>("config_path", "");
        robot_control_ = std::make_unique<SDK2RobotControl>(network_interface, timeout_s, auto_stand, config_path);

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::QoS(10),
            std::bind(&VelocityNode::onTwist, this, std::placeholders::_1));

        using namespace std::chrono_literals;
        auto period = std::chrono::microseconds(static_cast<int64_t>(1'000'000 / std::max(1, control_rate_hz)));
        control_timer_ = this->create_wall_timer(
            period, std::bind(&VelocityNode::controlLoop, this));

        RCLCPP_INFO(get_logger(), "VelocityNode started. rate=%dHz, stale_timeout=%.2fs", control_rate_hz, stale_timeout_s_);
    }

private:
    static double clamp(double value, double min_val, double max_val)
    {
        return std::max(min_val, std::min(max_val, value));
    }

    void onTwist(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_vx_ = clamp(msg->linear.x, -max_vx_, max_vx_);
        last_vy_ = clamp(msg->linear.y, -max_vy_, max_vy_);
        last_wz_ = clamp(msg->angular.z, -max_wz_, max_wz_);
        last_cmd_time_ = this->now();
        sent_stop_ = false;
    }

    void controlLoop()
    {
        const auto now = this->now();
        const double dt = (now - last_cmd_time_).seconds();

        if (dt <= stale_timeout_s_) {
            // Continue sending Move commands to maintain motion
            if (high_level_) {
                int rc = robot_control_->move(last_vx_, last_vy_, last_wz_);
                if (rc < 0) {
                    RCLCPP_WARN(get_logger(), "Move failed rc=%d", rc);
                }
            } else {
                robot_control_->applyVelCmdControl(last_vx_, last_vy_, last_wz_);
            }
        } else {
            // Command expired, send StopMove once
            if (!sent_stop_) {
                int rc = robot_control_->stopMove();
                RCLCPP_INFO(get_logger(), "StopMove rc=%d (cmd stale for %.2fs)", rc, dt);
                sent_stop_ = true;
                last_vx_ = last_vy_ = last_wz_ = 0.0;
            }
        }
    }

private:
    std::unique_ptr<SDK2RobotControl> robot_control_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    rclcpp::Time last_cmd_time_;
    double last_vx_;
    double last_vy_;
    double last_wz_;
    bool sent_stop_;
    bool high_level_;

    double max_vx_;
    double max_vy_;
    double max_wz_;
    double stale_timeout_s_;
};

}  // namespace unitree_control

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<unitree_control::VelocityNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}