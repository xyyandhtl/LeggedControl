/*****************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "sdk1/sdk1_robot_control.h" // Include the robot control logic

class SDK1ControlNode : public rclcpp::Node
{
public:
    SDK1ControlNode()
    : rclcpp::Node("sdk1_position_node"),
      robot_control_(8081, "192.168.123.220", 8082), // Unified UDP connection
      pos_cmd_time_(this->now()),
      last_vx_(0.0), last_vy_(0.0), last_wz_(0.0),
      cmd_vx_(0.0), cmd_vy_(0.0), cmd_wz_(0.0),
      joy_vx_(0.0), joy_vy_(0.0), joy_wz_(0.0),
      sent_stop_(false)
    {
        // Parameters
        int control_rate_hz = this->declare_parameter<int>("control_rate_hz", 50);
        double stale_timeout_s = this->declare_parameter<double>("stale_timeout_s", 1.0);

        max_vx_ = this->declare_parameter<double>("max_vx", 2.0);
        max_vy_ = this->declare_parameter<double>("max_vy", 1.0);
        max_wz_ = this->declare_parameter<double>("max_wz", 1.0);
        stale_timeout_s_ = stale_timeout_s;

        // Calculate dt based on control rate
        dt_ = 1.0 / control_rate_hz;

        // Subscribe to cmd_pos
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_pos", rclcpp::QoS(10),
            std::bind(&SDK1ControlNode::onPosition, this, std::placeholders::_1));

        // Control timer
        using namespace std::chrono_literals;
        auto period = std::chrono::microseconds(static_cast<int64_t>(1'000'000 / std::max(1, control_rate_hz)));
        control_timer_ = this->create_wall_timer(
            period, std::bind(&SDK1ControlNode::controlLoop, this));
    }

private:
    static double clamp(double value, double min_val, double max_val)
    {
        return std::max(min_val, std::min(max_val, value));
    }

    void onPosition(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        cmd_vx_ = clamp(msg->linear.x, -max_vx_, max_vx_);
        cmd_vy_ = clamp(msg->linear.y, -max_vy_, max_vy_);
        cmd_wz_ = clamp(msg->angular.z, -max_wz_, max_wz_);
        pos_cmd_time_ = this->now();
        sent_stop_ = false;
    }

    void controlLoop()
    {
        robot_control_.udpRecv(); // Update joystick and robot state
        const auto key_data = robot_control_.getJoystickData();

        // Process joystick input
        joy_vx_ = key_data.ly * max_vx_;  // ly controls forward/backward velocity
        joy_vy_ = key_data.lx * max_vy_;  // lx controls left/right velocity
        joy_wz_ = key_data.rx * max_wz_;  // rx controls angular velocity (yaw)

        if (key_data.btn.components.L2) { // Reset joystick commands
            joy_vx_ = 0.0;
            joy_vy_ = 0.0;
            joy_wz_ = 0.0;
        }

        const auto now = this->now();
        const double dt_pos = (now - pos_cmd_time_).seconds();

        if (dt_pos <= stale_timeout_s_) { // Respond to position commands
            last_vx_ = cmd_vx_;
            last_vy_ = cmd_vy_;
            last_wz_ = cmd_wz_;
            robot_control_.applyVelCmdControl(last_vx_, last_vy_, last_wz_);
        } else { // Use joystick commands
            last_vx_ = joy_vx_;
            last_vy_ = joy_vy_;
            last_wz_ = joy_wz_;
            robot_control_.applyVelCmdControl(last_vx_, last_vy_, last_wz_);
        }
    }

private:
    SDK1RobotControl robot_control_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    rclcpp::Time pos_cmd_time_;
    double last_vx_; double last_vy_; double last_wz_;
    double cmd_vx_; double cmd_vy_; double cmd_wz_;
    double joy_vx_; double joy_vy_; double joy_wz_;
    bool sent_stop_;

    double max_vx_;
    double max_vy_;
    double max_wz_;
    double stale_timeout_s_;
    double dt_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SDK1ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}