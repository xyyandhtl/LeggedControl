#ifndef SDK2_POLICY_CONTROLLER_H
#define SDK2_POLICY_CONTROLLER_H

#include "base_robot_control.h"
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>

class SDK2PolicyController : public BaseRobotControl {
public:
    SDK2PolicyController(const std::string& config_path);
    ~SDK2PolicyController();

    void startControlLoop();
    void stopControlLoop();

    void setPolicyRunning(bool running);
    void togglePolicyRunning();

    void resetJointPosition() override;
    void applyVelCmdControl(float vx, float vy, float wz) override;

protected:
    // Implement BaseRobotControl pure virtual functions
    void applyPositionControl(std::vector<float>& joint_positions) override;
    const RobotObsResult getRobotObs() override;

private:
    // DDS Topics
    static constexpr const char* TOPIC_LOWSTATE = "rt/lowstate";
    static constexpr const char* TOPIC_LOWCMD   = "rt/lowcmd";

    // DDS Channels
    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_pub_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_sub_;

    // DDS Messages
    unitree_go::msg::dds_::LowCmd_ low_cmd_{};
    unitree_go::msg::dds_::LowState_ low_state_{};

    // Control loop thread
    unitree::common::ThreadPtr control_thread_;

    void onLowStateMessage(const void* msg);
    void initLowCmd();
    static uint32_t crc32_core(uint32_t* ptr, uint32_t len);
};

#endif // SDK2_POLICY_CONTROLLER_H
