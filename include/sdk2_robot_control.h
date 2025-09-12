#ifndef SDK2_ROBOT_CONTROL_HPP
#define SDK2_ROBOT_CONTROL_HPP

#include "sdk2_sport_controller.h"
#include "sdk2_policy_controller.h"
#include <unitree/robot/channel/channel_subscriber.hpp>
#include "unitree/idl/go2/WirelessController_.hpp"
#include "utils.h"
#include <chrono>

class SDK2RobotControl {
public:
    SDK2RobotControl(const std::string &network_interface, bool auto_stand, const std::string& config_path);
    ~SDK2RobotControl();

    void setVelCmd(float vx, float vy, float wz);
    void shutdown();
    void setStandalone(bool standalone);

private:
    enum class ControlState { SPORT_MODE, POLICY_MODE };

    static constexpr const char* TOPIC_JOYSTICK = "rt/wirelesscontroller";

    void onJoystickMessage(const void* msg);
    void switchToPolicyMode();
    void switchToSportMode();

    std::unique_ptr<SDK2SportController> sport_controller_;
    std::unique_ptr<SDK2PolicyController> policy_controller_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> joystick_sub_;

    ControlState current_state_ = ControlState::SPORT_MODE;
    unitree::common::Gamepad gamepad_;

    // For command arbitration
    std::array<float, 3> ros_vel_cmd_{0.0f, 0.0f, 0.0f};
    std::chrono::steady_clock::time_point last_ros_cmd_time_;
    const std::chrono::duration<double> ros_cmd_timeout_{0.5};
};

#endif // SDK2_ROBOT_CONTROL_HPP