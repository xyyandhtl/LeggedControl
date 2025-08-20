#ifndef SDK2_ROBOT_CONTROL_HPP
#define SDK2_ROBOT_CONTROL_HPP

#include <memory>
#include <array>
#include <vector>
#include <mutex>
#include <string>
#include <cstdint>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <optional>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <onnxruntime_cxx_api.h>

struct SDK2PolicyConfig {
    std::string onnx_model_path = "policy_model.onnx";
    float act_scale = 0.5f;
    float vel_scale = 2.0f;
    float gyr_scale = 0.25f;
    float dof_pos_scale = 1.0f;
    float dof_vel_scale = 0.05f;
    float obs_clip = 100.0f;
    float kp = 40.0f; // PD gain
    float kd = 2.0f;  // PD gain
    std::array<float, 12> dft_dof_pos = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    std::array<int, 12> joint_idx_sdk2policy = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    std::array<int, 12> joint_idx_policy2sdk = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    std::size_t history_steps = 6; // number of past frames to keep (1 = current only)
    int obs_size = 45;             // 3+3+3+12+12+12

    static SDK2PolicyConfig FromFile(const std::string& path, bool* ok = nullptr);
};

struct SDK2RobotObsResult {
    // Flattened history buffer, length = cfg.obs_size * cfg.history_steps
    std::vector<float> his_obs;
};

class SDK2RobotControl
{
public:
    enum class ControlMode { HighLevel = 0, LowLevel = 1 };

    SDK2RobotControl(const std::string &network_interface, double timeout_s, bool auto_stand, const std::string& config_path);
    ~SDK2RobotControl();

    // Low-level position control interface (auto-switch to LowLevel)
    void controlLoop();
    void resetJointPosition();
    void applyPositionControl(const std::array<double, 12> &joint_positions);
    void applyVelCmdControl(double vx, double vy, double wz);

    void setObsConfig(const SDK2PolicyConfig& cfg);
    SDK2RobotObsResult getRobotObs();

    // High-level velocity control interface (auto-switch to HighLevel)
    int move(double vx, double vy, double wz);
    int stopMove();
    // Explicitly switch control mode
    void setControlMode(ControlMode mode);
    ControlMode controlMode() const { return mode_; }

private:
    // Topics
    static constexpr const char* TOPIC_LOWSTATE = "rt/lowstate";
    static constexpr const char* TOPIC_LOWCMD   = "rt/lowcmd";

    // Low-level channels
    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_pub_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_sub_;

    // Low-level messages
    unitree_go::msg::dds_::LowCmd_ low_cmd_{};
    unitree_go::msg::dds_::LowState_ low_state_{};
    std::mutex low_state_mtx_;
    // std::mutex low_cmd_mtx_;

    float control_dt_ = 0.002;

    // High-level client
    std::unique_ptr<unitree::robot::go2::SportClient> sport_client_;
    // Low-level control loop thread
    unitree::common::ThreadPtr controlLoopThreadPtr_;

    // Motion switcher (release mode when entering LowLevel)
    std::unique_ptr<unitree::robot::b2::MotionSwitcherClient> msc_;

    // PD gains (align with sdk1)
    float Kp_ = 40.0f;
    float Kd_ = 2.0f;

    std::string config_path_;
    SDK2PolicyConfig obs_cfg_;
    std::vector<float> his_obs_;
    std::array<float, 3> last_cmd_{0.0f, 0.0f, 0.0f};
    std::array<float, 12> last_act_{};

    // ONNX Runtime members
    std::unique_ptr<Ort::Env> ort_env_;
    std::unique_ptr<Ort::Session> ort_session_;
    std::vector<std::string> ort_input_names_str_;
    std::vector<std::string> ort_output_names_str_;
    std::vector<const char*> ort_input_names_;
    std::vector<const char*> ort_output_names_;
    bool onnx_ready_ = false;

    // Control mode and low-level loop
    ControlMode mode_ = ControlMode::HighLevel;
    std::thread lowcmd_loop_;
    std::atomic<bool> lowcmd_loop_running_{false};

    void onLowStateMessage(const void* msg);

    int queryMotionStatus();
    void releaseMotionModeIfNeeded();

    void initLowCmd();
    void ensureObsBuffers();
    static std::array<float, 3> gravFromQuatWxyz(const std::array<float, 4>& q_wxyz);
    static inline float clip(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
    static uint32_t crc32_core(uint32_t* ptr, uint32_t len);
};

#endif // SDK2_ROBOT_CONTROL_HPP