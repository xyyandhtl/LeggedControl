/*****************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include <chrono>
#include <algorithm>
#include <memory>
#include <string>
#include <math.h>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace UNITREE_LEGGED_SDK;

namespace unitree_control
{

class SDK1VelocityNode : public rclcpp::Node
{
public:
    SDK1VelocityNode()
    : rclcpp::Node("sdk1_velocity_node"),
      last_cmd_time_(this->now()),
      last_vx_(0.0), last_vy_(0.0), last_wz_(0.0),
      sent_stop_(false),
      Tpi_(0),
      motiontime_(0)
    {
        // Parameters
        uint16_t target_port = this->declare_parameter<uint16_t>("target_port", 8007);
        uint16_t local_port = this->declare_parameter<uint16_t>("local_port", 8082);
        std::string target_ip = this->declare_parameter<std::string>("target_ip", "192.168.123.10");
        int control_rate_hz = this->declare_parameter<int>("control_rate_hz", 500); // 500Hz for low-level control
        double stale_timeout_s = this->declare_parameter<double>("stale_timeout_s", 0.5);
        
        max_vx_ = this->declare_parameter<double>("max_vx", 2.0);
        max_vy_ = this->declare_parameter<double>("max_vy", 1.0);
        max_wz_ = this->declare_parameter<double>("max_wz", 2.0);
        stale_timeout_s_ = stale_timeout_s;
        
        // Calculate dt based on control rate
        dt_ = 1.0 / control_rate_hz;
        
        RCLCPP_INFO(get_logger(), "Init Unitree SDK1 UDP on %s:%d -> %s:%d", 
                   target_ip.c_str(), target_port, target_ip.c_str(), target_port);
        
        // Initialize UDP connection
        udp_ = std::make_unique<UDP>(local_port, target_ip.c_str(), target_port, LOW_CMD_LENGTH, LOW_STATE_LENGTH);
        udp_->InitCmdData(cmd_);
        cmd_.levelFlag = LOWLEVEL;
        
        // Initialize safety
        safe_ = std::make_unique<Safety>(LeggedType::Aliengo);
        
        // Subscribe to cmd_vel
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::QoS(10),
            std::bind(&SDK1VelocityNode::onTwist, this, std::placeholders::_1));
        
        // Control timer
        using namespace std::chrono_literals;
        auto period = std::chrono::microseconds(static_cast<int64_t>(1'000'000 / std::max(1, control_rate_hz)));
        control_timer_ = this->create_wall_timer(
            period, std::bind(&SDK1VelocityNode::controlLoop, this));
        
        // UDP send/recv timers (higher frequency for UDP communication)
        auto udp_period = std::chrono::microseconds(static_cast<int64_t>(1'000'000 / 500)); // 500Hz UDP
        udp_send_timer_ = this->create_wall_timer(
            udp_period, std::bind(&SDK1VelocityNode::udpSend, this));
        udp_recv_timer_ = this->create_wall_timer(
            udp_period, std::bind(&SDK1VelocityNode::udpRecv, this));
        
        RCLCPP_INFO(get_logger(), "SDK1VelocityNode started. control_rate=%dHz, stale_timeout=%.2fs", 
                   control_rate_hz, stale_timeout_s_);
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
    
    void udpRecv()
    {
        udp_->Recv();
    }
    
    void udpSend()
    {
        udp_->Send();
    }
    
    void controlLoop()
    {
        motiontime_++;
        udp_->GetRecv(state_);
        
        // Gravity compensation (same as example)
        cmd_.motorCmd[FR_0].tau = -1.6f;
        
        const auto now = this->now();
        const double dt_cmd = (now - last_cmd_time_).seconds();
        
        if (dt_cmd <= stale_timeout_s_) {
            // Command is still valid, apply velocity control
            if (motiontime_ >= 500) {
                // Apply velocity control to front right leg (FR_1)
                float speed = 2 * sin(3 * M_PI * Tpi_ / 2000.0);
                
                cmd_.motorCmd[FR_1].q = PosStopF;
                cmd_.motorCmd[FR_1].dq = speed * last_vx_; // Scale by vx
                cmd_.motorCmd[FR_1].Kp = 0;
                cmd_.motorCmd[FR_1].Kd = 4;
                cmd_.motorCmd[FR_1].tau = 0.0f;
                
                // Apply yaw control to other legs if needed
                if (std::abs(last_wz_) > 0.1) {
                    // Simple yaw control by applying opposite torques
                    float yaw_torque = last_wz_ * 0.5f;
                    cmd_.motorCmd[FL_0].tau = yaw_torque;
                    cmd_.motorCmd[RR_0].tau = -yaw_torque;
                    cmd_.motorCmd[RL_0].tau = yaw_torque;
                }
                
                Tpi_++;
            }
        } else {
            // Command expired, stop movement
            if (!sent_stop_) {
                RCLCPP_INFO(get_logger(), "Command stale for %.2fs, stopping movement", dt_cmd);
                sent_stop_ = true;
                last_vx_ = last_vy_ = last_wz_ = 0.0;
                
                // Stop all motors
                for (int i = 0; i < 12; i++) {
                    cmd_.motorCmd[i].q = PosStopF;
                    cmd_.motorCmd[i].dq = VelStopF;
                    cmd_.motorCmd[i].Kp = 0;
                    cmd_.motorCmd[i].Kd = 0;
                    cmd_.motorCmd[i].tau = 0.0f;
                }
            }
        }
        
        // Safety check (same as example)
        // if (motiontime_ > 10) {
        //     safe_->PowerProtect(cmd_, state_, 1);
        //     safe_->PositionProtect(cmd_, state_, 0.087);
        // }
        
        udp_->SetSend(cmd_);
    }

private:
    std::unique_ptr<UDP> udp_;
    std::unique_ptr<Safety> safe_;
    LowCmd cmd_ = {0};
    LowState state_ = {0};
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr udp_send_timer_;
    rclcpp::TimerBase::SharedPtr udp_recv_timer_;
    
    rclcpp::Time last_cmd_time_;
    double last_vx_;
    double last_vy_;
    double last_wz_;
    bool sent_stop_;
    
    double max_vx_;
    double max_vy_;
    double max_wz_;
    double stale_timeout_s_;
    double dt_;
    
    int Tpi_;
    int motiontime_;
    
    // Constants from example
    static constexpr int LOW_CMD_LENGTH = 610;
    static constexpr int LOW_STATE_LENGTH = 771;
};

}  // namespace unitree_control

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<unitree_control::SDK1VelocityNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
