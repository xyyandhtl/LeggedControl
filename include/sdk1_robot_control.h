#ifndef SDK1_ROBOT_CONTROL_HPP
#define SDK1_ROBOT_CONTROL_HPP

#include "base_robot_control.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/unitree_joystick.h"

using namespace UNITREE_LEGGED_SDK;

class SDK1RobotControl : public BaseRobotControl {
public:
    SDK1RobotControl(uint16_t local_port, const std::string &target_ip, uint16_t target_port, const std::string& config_path);
    ~SDK1RobotControl();
    
    // Low-level position control interface (auto-switch to LowLevel)
    // void controlLoop() override;
    void resetJointPosition() override;
    void applyPositionControl(std::vector<float>& joint_positions) override;
    void applyVelCmdControl(float vx, float vy, float wz) override;
    const RobotObsResult getRobotObs() override;

    void udpRecv();
    void udpSend();
    void stopMotors();

    xRockerBtnDataStruct getJoystickData() const;

private:
    UDP udp_;
    Safety safe_;
    LowCmd cmd_ = {0};
    LowState state_ = {0};
    xRockerBtnDataStruct key_data_ = {0}; // Joystick data
    static constexpr int LOW_CMD_LENGTH = 610;
    static constexpr int LOW_STATE_LENGTH = 771;
    std::mutex low_state_mtx_;
    // std::mutex low_cmd_mtx_;

    // Observation-related state
    // std::vector<float> his_obs_;
    // std::array<float, 3> last_cmd_{0.0f, 0.0f, 0.0f};
    // std::array<float, 12> last_act_{};

    std::unique_ptr<LoopFunc> loop_udpSend;
    std::unique_ptr<LoopFunc> loop_udpRecv;
    std::unique_ptr<LoopFunc> loop_control;    
};

#endif // SDK1_ROBOT_CONTROL_HPP