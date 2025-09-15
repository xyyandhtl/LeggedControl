#ifndef SDK2_POLICY_CONTROL_H
#define SDK2_POLICY_CONTROL_H

#include "base_robot_control.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include "utils.h"

class SDK2PolicyControl : public BaseRobotControl, public rclcpp::Node {
public:
    SDK2PolicyControl(const rclcpp::NodeOptions & options);
    ~SDK2PolicyControl();

    void startControlLoop();
    void stopControlLoop();

    void setPolicyRunning(bool running);
    void togglePolicyRunning();
    bool isControlLoopRunning() const { return control_thread_ != nullptr; }

    void resetJointPosition() override;
    void applyVelCmdControl(float vx, float vy, float wz) override;

protected:
    void applyPositionControl(std::vector<float>& joint_positions) override;
    const RobotObsResult getRobotObs() override;

private:
    void onTwist(const geometry_msgs::msg::Twist::SharedPtr msg);
    void onJoystickMessage(const void *msg);
    void onLowStateMessage(const void* msg);
    void controlLoop();

    void activatePolicyMode();
    void initLowCmd();
    static uint32_t crc32_core(uint32_t* ptr, uint32_t len);

    // DDS Topics
    static constexpr const char* TOPIC_LOWSTATE = "rt/lowstate";
    static constexpr const char* TOPIC_LOWCMD   = "rt/lowcmd";

    // DDS Channels
    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_pub_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_sub_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> joystick_sub_;

    // DDS Messages
    unitree_go::msg::dds_::LowCmd_ low_cmd_{};
    unitree_go::msg::dds_::LowState_ low_state_{};

    // Control loop thread
    unitree::common::ThreadPtr control_thread_;

    // ROS
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

    // For releasing sport mode
    std::unique_ptr<unitree::robot::b2::MotionSwitcherClient> msc_;

    // Joystick & State
    unitree::common::Gamepad gamepad_;
    std::array<float, 3> ros_vel_cmd_{0.0f, 0.0f, 0.0f};
    rclcpp::Time last_ros_cmd_time_;

    // Parameters
    float max_vx_, max_vy_, max_wz_;
    float stale_timeout_s_;
};

#endif // SDK2_POLICY_CONTROL_H
