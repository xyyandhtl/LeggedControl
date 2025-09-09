#ifndef BASE_ROBOT_CONTROL_H
#define BASE_ROBOT_CONTROL_H

#include <array>
#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>
#include <functional>

#include "utils.h"
#include "onnxruntime_cxx_api.h"

class BaseRobotControl {
public:
    struct PolicyConfig {
        std::string onnx_model_path = "policy_model.onnx";
        float act_scale = 1.0f;
        float vel_scale = 1.0f;
        float gyr_scale = 1.0f;
        float dof_pos_scale = 1.0f;
        float dof_vel_scale = 1.0f;
        float obs_clip = 1.0f;
        float kp = 40.0f;
        float kd = 2.0f;
        std::size_t history_steps = 1;
        int obs_size = 45;
        int act_size = 12;
        std::vector<float> dft_dof_pos{};
        std::vector<int> joint_idx_sdk2policy{};
        std::vector<int> joint_idx_policy2sdk{};

        static PolicyConfig FromFile(const std::string& path, bool* ok = nullptr);
    };

    struct RobotObsResult {
        // to be extended for multi-format obs
        std::vector<float> his_obs;
    };

    struct OnnxOptions {
        enum class Provider { CPU, CUDA, TensorRT };
        Provider provider{Provider::CPU};
        int device_id{0};     // GPU id
    };

    virtual ~BaseRobotControl() = default;

    virtual void resetJointPosition() = 0;
    virtual void applyPositionControl(std::vector<float>& joint_positions) = 0;
    virtual void applyVelCmdControl(float vx, float vy, float wz) = 0;
    virtual const RobotObsResult getRobotObs() = 0;

    void controlLoop();

    void setObsConfig(const PolicyConfig& cfg);
    
    std::array<float, 3> gravFromQuatWxyz(const std::array<float, 4>& q);
    // Onnx inference
    bool runOnnxInference(const std::vector<float>& input, std::vector<float>& output);
    // If non-ros, set standalone mode
    void setStandalone(bool standalone) {
        standalone_ = standalone;
        std::cout << "[BaseRobotControl] standalone_ set to: " << (standalone_ ? "True" : "False") << std::endl;
    }

protected:
    std::string config_path_;
    PolicyConfig obs_cfg_;
    std::vector<float> his_obs_;
    std::vector<float> last_act_; // Changed from std::array<float, 12> to std::vector<float>
    std::array<float, 3> last_cmd_{0.0f, 0.0f, 0.0f};

    unitree::common::Gamepad gamepad_;
    std::atomic<bool> policy_running_{false};

    std::unique_ptr<Ort::Env> ort_env_;
    std::unique_ptr<Ort::Session> ort_session_;
    std::vector<const char*> ort_input_names_;
    std::vector<const char*> ort_output_names_;
    std::vector<std::string> ort_input_names_str_;
    std::vector<std::string> ort_output_names_str_;
    bool onnx_ready_ = false;

    bool standalone_ = false;
    float control_dt_ = 0.002;
    std::mutex low_state_mtx_;
    std::mutex low_cmd_mtx_;

    void loadOnnxModel(const std::string& onnx_path);
    void loadOnnxModel(const std::string& onnx_path, const OnnxOptions& opt);

    void ensureObsBuffers();
    static inline float clip(float v, float lo, float hi) {
      return v < lo ? lo : (v > hi ? hi : v);
    }
};

#endif // BASE_ROBOT_CONTROL_H
