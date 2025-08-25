#ifndef SDK2_ROBOT_CONTROL_HPP
#define SDK2_ROBOT_CONTROL_HPP

#include "base_robot_control.h"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include "unitree/idl/go2/WirelessController_.hpp"
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

class SDK2RobotControl : public BaseRobotControl {
public:
    enum class ControlMode { HighLevel = 0, LowLevel = 1 };

    SDK2RobotControl(const std::string &network_interface, double timeout_s, bool auto_stand, const std::string& config_path);
    ~SDK2RobotControl();

    // Low-level position control interface (auto-switch to LowLevel)
    // void controlLoop() override;
    void resetJointPosition() override;
    void applyPositionControl(const std::array<double, 12> &joint_positions) override;
    void applyVelCmdControl(double vx, double vy, double wz) override;
    const RobotObsResult getRobotObs() override;

    // High-level velocity control interface (auto-switch to HighLevel)
    int move(double vx, double vy, double wz);
    int stopMove();
    // Explicitly switch control mode
    void setControlMode(ControlMode mode);
    ControlMode controlMode() const { return mode_; }

    void initJoystick(const std::string &network_interface);

private:
    // Topics
    static constexpr const char* TOPIC_LOWSTATE = "rt/lowstate";
    static constexpr const char* TOPIC_LOWCMD   = "rt/lowcmd";
    static constexpr const char* TOPIC_JOYSTICK = "rt/wirelesscontroller";

    // Low-level channels
    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_pub_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_sub_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> joystick_sub_;

    // Low-level messages
    unitree_go::msg::dds_::LowCmd_ low_cmd_{};
    unitree_go::msg::dds_::LowState_ low_state_{};
    unitree_go::msg::dds_::WirelessController_ key_data_;
    // std::mutex joystick_mtx_;

    // High-level client
    std::unique_ptr<unitree::robot::go2::SportClient> sport_client_;
    // Low-level control loop thread
    unitree::common::ThreadPtr controlLoopThreadPtr_;

    // Motion switcher (release mode when entering LowLevel)
    std::unique_ptr<unitree::robot::b2::MotionSwitcherClient> msc_;

    // std::array<float, 3> last_cmd_{0.0f, 0.0f, 0.0f};
    // std::array<float, 12> last_act_{};

    // Control mode and low-level loop
    ControlMode mode_ = ControlMode::HighLevel;
    std::thread lowcmd_loop_;
    std::atomic<bool> lowcmd_loop_running_{false};

    float joystick_smooth_ = 0.03f;
    float joystick_dead_zone_ = 0.01f;
    float joystick_lx_ = 0.0f;
    float joystick_ly_ = 0.0f;
    float joystick_rx_ = 0.0f;
    float joystick_ry_ = 0.0f;

    void onLowStateMessage(const void* msg);

    int queryMotionStatus();
    void releaseMotionModeIfNeeded();

    void initLowCmd();
    static uint32_t crc32_core(uint32_t* ptr, uint32_t len);
};

#endif // SDK2_ROBOT_CONTROL_HPP