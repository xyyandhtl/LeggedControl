#ifndef SDK1_ROBOT_CONTROL_HPP
#define SDK1_ROBOT_CONTROL_HPP

#include <string>
#include <array>
#include <vector>
#include <mutex>

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/unitree_joystick.h"
#include <onnxruntime_cxx_api.h>

using namespace UNITREE_LEGGED_SDK;

// Add a small config and result type for observations
struct SDK1PolicyConfig {
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

    static SDK1PolicyConfig FromFile(const std::string& path, bool* ok = nullptr);
};

struct SDK1RobotObsResult {
    // Flattened history buffer, length = cfg.obs_size * cfg.history_steps
    std::vector<float> his_obs;
};

class SDK1RobotControl
{
public:
    SDK1RobotControl(uint16_t local_port, const std::string &target_ip, uint16_t target_port, const std::string& config_path);
    ~SDK1RobotControl();
    
    // Low-level position control interface (auto-switch to LowLevel)
    void controlLoop();
    void resetJointPosition();
    void applyPositionControl(const std::array<double, 12> &joint_positions);
    void applyVelCmdControl(double vx, double vy, double wz);

    void setObsConfig(const SDK1PolicyConfig& cfg);
    SDK1RobotObsResult getRobotObs();

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
    
    float control_dt_ = 0.002;

    // Observation-related state
    std::string config_path_;
    SDK1PolicyConfig obs_cfg_;
    std::vector<float> his_obs_;
    std::array<float, 3> last_cmd_{0.0f, 0.0f, 0.0f};
    std::array<float, 12> last_act_{};
    // If need averaging something
    // std::array<std::array<float, 3>, 12> gyr_buffer_{}; // Buffer for last 12 gyroscope readings
    // std::array<float, 12> gyr_weights_{}; // Time intervals between consecutive frames
    // int gyr_update_count_ = 0; // Counter for gyroscope updates
    // std::chrono::steady_clock::time_point last_time_; // Timestamp of the last update

    // ONNX Runtime members
    std::unique_ptr<Ort::Env> ort_env_;
    std::unique_ptr<Ort::Session> ort_session_;
    std::vector<std::string> ort_input_names_str_;
    std::vector<std::string> ort_output_names_str_;
    std::vector<const char*> ort_input_names_;
    std::vector<const char*> ort_output_names_;
    bool onnx_ready_ = false;

    std::unique_ptr<LoopFunc> loop_udpSend;
    std::unique_ptr<LoopFunc> loop_udpRecv;
    std::unique_ptr<LoopFunc> loop_control;

    void ensureObsBuffers();
    static std::array<float, 3> gravFromQuatWxyz(const std::array<float, 4>& q_wxyz);
    static inline float clip(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
};

#endif // SDK1_ROBOT_CONTROL_HPP