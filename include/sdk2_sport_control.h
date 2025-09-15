#ifndef SDK2_SPORT_CONTROL_H
#define SDK2_SPORT_CONTROL_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include "unitree/idl/go2/WirelessController_.hpp"
#include "utils.h"
#include <mutex>

class SDK2SportControl : public rclcpp::Node {
public:
    SDK2SportControl(const rclcpp::NodeOptions & options);
    ~SDK2SportControl();

    void controlLoop();

private:
    void onTwist(const geometry_msgs::msg::Twist::SharedPtr msg);
    void onJoystickMessage(const void *msg);
    void onSportStateMessage(const void* msg);

    // Unitree SDK
    std::unique_ptr<unitree::robot::go2::SportClient> sport_client_;

    // ROS
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Joystick
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> joystick_sub_;
    unitree::common::Gamepad gamepad_;

    // Sport State
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> sport_state_sub_;
    unitree_go::msg::dds_::SportModeState_ current_sport_state_;
    std::mutex sport_state_mutex_;

    // Command arbitration
    std::array<float, 3> ros_vel_cmd_{0.0f, 0.0f, 0.0f};
    rclcpp::Time last_ros_cmd_time_;
    bool sent_stop_ = false;

    // Parameters
    float max_vx_, max_vy_, max_wz_;
    float stale_timeout_s_;
};

#endif // SDK2_SPORT_CONTROL_H
