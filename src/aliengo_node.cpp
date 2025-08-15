/*****************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "sdk1/sdk1_robot_control.h" // Include the robot control logic

class SDK1ControlNode : public rclcpp::Node
{
public:
    SDK1ControlNode()
    : rclcpp::Node("sdk1_position_node"),
      robot_control_(8082, "192.168.123.10", 8007),
      pos_cmd_time_(this->now()),
      joy_cmd_time_(this->now() - rclcpp::Duration::from_seconds(1000)),
      last_px_(0.0), last_py_(0.0), last_pz_(0.0),
      cmd_px_(0.0), cmd_py_(0.0), cmd_pz_(0.0),
      joy_px_(0.0), joy_py_(0.0), joy_pz_(0.0),
      sent_stop_(false)
    {
        // Parameters
        int control_rate_hz = this->declare_parameter<int>("control_rate_hz", 50);
        double stale_timeout_s = this->declare_parameter<double>("stale_timeout_s", 1.0);

        max_px_ = this->declare_parameter<double>("max_px", 2.0);
        max_py_ = this->declare_parameter<double>("max_py", 1.0);
        max_pz_ = this->declare_parameter<double>("max_pz", 1.0);
        stale_timeout_s_ = stale_timeout_s;

        // Calculate dt based on control rate
        dt_ = 1.0 / control_rate_hz;

        // Subscribe to cmd_pos
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_pos", rclcpp::QoS(10),
            std::bind(&SDK1ControlNode::onPosition, this, std::placeholders::_1));

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", rclcpp::QoS(10),
            std::bind(&SDK1ControlNode::onJoy, this, std::placeholders::_1));

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
        cmd_px_ = clamp(msg->linear.x, -max_px_, max_px_);
        cmd_py_ = clamp(msg->linear.y, -max_py_, max_py_);
        cmd_pz_ = clamp(msg->angular.z, -max_pz_, max_pz_);
        pos_cmd_time_ = this->now();
        sent_stop_ = false;
    }

    void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        joy_cmd_time_ = this->now();
        sent_stop_ = false;

        // 处理轴输入 (lx, ly -> px, py)
        joy_px_ = msg->axes[0] * max_px_;  // lx轴控制x方向速度
        joy_py_ = msg->axes[1] * max_py_;  // ly轴控制y方向速度

        // 处理ABXY按钮 (控制角速度pz)
        joy_pz_ = 0.0;
        if (msg->buttons[0]) joy_pz_ += max_pz_ * 0.5;  // A按钮
        if (msg->buttons[1]) joy_pz_ -= max_pz_ * 0.5;  // B按钮
        if (msg->buttons[2]) joy_pz_ += max_pz_;        // X按钮
        if (msg->buttons[3]) joy_pz_ -= max_pz_;        // Y按钮
        joy_pz_ = clamp(joy_pz_, -max_pz_, max_pz_);

        // L2按钮重置速度指令
        if (msg->buttons[6]) {  // 假设L2对应按钮6
            joy_px_ = 0.0;
            joy_py_ = 0.0;
            joy_pz_ = 0.0;
        }
    }

    void controlLoop()
    {
        const auto now = this->now();
        const double dt_pos = (now - pos_cmd_time_).seconds();
        const double dt_joy = (now - joy_cmd_time_).seconds();

        if (dt_joy <= stale_timeout_s_) {  // 优先响应摇杆输入
            last_px_ = joy_px_;
            last_py_ = joy_py_;
            last_pz_ = joy_pz_;
            robot_control_.applyVelCmdControl(last_px_, last_py_, last_pz_);
        } else if (dt_pos <= stale_timeout_s_) {  // 其次响应速度话题
            last_px_ = cmd_px_;
            last_py_ = cmd_py_;
            last_pz_ = cmd_pz_;
            robot_control_.applyVelCmdControl(last_px_, last_py_, last_pz_);
        } else {  // 指令超时，停止运动
            if (!sent_stop_) {
                RCLCPP_INFO(get_logger(), "Command stale for %.2fs, stopping movement", std::max(dt_pos, dt_joy));
                sent_stop_ = true;
                robot_control_.stopMotors();
            }
        }
    }

private:
    SDK1RobotControl robot_control_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    rclcpp::Time pos_cmd_time_;
    rclcpp::Time joy_cmd_time_;
    double last_px_; double last_py_; double last_pz_;
    double cmd_px_; double cmd_py_; double cmd_pz_;
    double joy_px_; double joy_py_; double joy_pz_;
    bool sent_stop_;

    double max_px_;
    double max_py_;
    double max_pz_;
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