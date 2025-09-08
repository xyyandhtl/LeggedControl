#include "sdk2_policy_controller.h"
#include "utils.h"
#include <iostream>
#include <algorithm>
#include <iterator>

// 常量与示例一致
static constexpr float PosStopF = (2.146E+9f);
static constexpr float VelStopF = (16000.0f);

SDK2PolicyController::SDK2PolicyController(const std::string& config_path) {
    // Load config
    config_path_ = config_path;
    bool ok = false;
    obs_cfg_ = PolicyConfig::FromFile(config_path, &ok);
    std::cout << "[SDK2 | PolicyController] Load config: " << config_path << (ok ? " [OK]" : " [ERR]") << std::endl;

    // Ensure buffers for observations
    ensureObsBuffers();
    std::cout << "[SDK2 | PolicyController] Observation buffers prepared: obs_size=" << obs_cfg_.obs_size
              << " history_steps=" << obs_cfg_.history_steps << std::endl;

    // Load ONNX model
    size_t pos = config_path_.find_last_of('/');
    std::string parent_dir = config_path_.substr(0, pos + 1);
    std::string onnx_path = parent_dir + obs_cfg_.onnx_model_path;
    std::cout << "[SDK2 | PolicyController] Load onnx_path: " << onnx_path << std::endl;
    loadOnnxModel(onnx_path);

    // Initialize LowLevel DDS channels
    lowcmd_pub_.reset(new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_pub_->InitChannel();
    std::cout << "[SDK2 | PolicyController] LowCmd publisher initialized: " << TOPIC_LOWCMD << std::endl;

    lowstate_sub_.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_sub_->InitChannel(std::bind(&SDK2PolicyController::onLowStateMessage, this, std::placeholders::_1), 1);
    std::cout << "[SDK2 | PolicyController] LowState subscriber initialized: " << TOPIC_LOWSTATE << std::endl;

    // Initialize LowCmd message structure
    initLowCmd();
    std::cout << "[SDK2 | PolicyController] LowCmd message initialized." << std::endl;
}

SDK2PolicyController::~SDK2PolicyController() {
    stopControlLoop();
}

void SDK2PolicyController::startControlLoop() {
    if (control_thread_) {
        std::cout << "[SDK2 | PolicyController] Control loop already running." << std::endl;
        return;
    }
    std::cout << "[SDK2 | PolicyController] Starting control loop thread." << std::endl;
    control_thread_ = unitree::common::CreateRecurrentThreadEx(
        "policy_control_loop", UT_CPU_ID_NONE, 20000, &SDK2PolicyController::controlLoop, this);
}

void SDK2PolicyController::stopControlLoop() {
    if (control_thread_) {
        std::cout << "[SDK2 | PolicyController] Stopping control loop thread." << std::endl;
        control_thread_.reset(); // Resetting the unique_ptr will destroy the thread object and stop the thread.
    }
}

void SDK2PolicyController::setPolicyRunning(bool running) {
    policy_running_ = running;
    std::cout << "[SDK2 | PolicyController] Policy is now " << (running ? "RUNNING" : "STOPPED") << std::endl;
}

void SDK2PolicyController::togglePolicyRunning() {
    policy_running_ = !policy_running_;
    std::cout << "[SDK2 | PolicyController] Policy is now " << (policy_running_ ? "RUNNING" : "STOPPED") << std::endl;
}

void SDK2PolicyController::resetJointPosition() {
    const int steps = 200;  // 2 seconds at 10ms intervals
    const float interval = 0.01f;  // 10ms
    std::vector<float> current_positions(obs_cfg_.act_size);
    std::vector<float> target_positions(obs_cfg_.act_size);

    // Initialize current and target positions
    for (int i = 0; i < obs_cfg_.act_size; ++i) {
        current_positions[i] = low_state_.motor_state()[i].q();
        target_positions[i] = obs_cfg_.dft_dof_pos[i];
    }

    for (int step = 0; step <= steps; ++step) {
        std::vector<float> interpolated_positions(obs_cfg_.act_size);
        for (int i = 0; i < obs_cfg_.act_size; ++i) {
            interpolated_positions[i] = current_positions[i] + 
                (target_positions[i] - current_positions[i]) * (static_cast<float>(step) / steps);
        }
        applyPositionControl(interpolated_positions);
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(interval * 1000)));
    }
}

void SDK2PolicyController::applyPositionControl(std::vector<float>& joint_positions) {
    for (std::size_t i = 0; i < obs_cfg_.act_size; i++) {
        low_cmd_.motor_cmd()[i].mode() = 0x01;  // PMSM 伺服
        low_cmd_.motor_cmd()[i].q()    = joint_positions[i];
        low_cmd_.motor_cmd()[i].dq()   = 0.0f;
        low_cmd_.motor_cmd()[i].kp()   = obs_cfg_.kp;
        low_cmd_.motor_cmd()[i].kd()   = obs_cfg_.kd;
        low_cmd_.motor_cmd()[i].tau()  = 0.0f;  // 安全起见不叠加力矩
    }
    low_cmd_.crc() = crc32_core(reinterpret_cast<uint32_t*>(&low_cmd_), (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_pub_->Write(low_cmd_);
}

void SDK2PolicyController::applyVelCmdControl(float vx, float vy, float wz) {
    last_cmd_[0] = vx;
    last_cmd_[1] = vy;
    last_cmd_[2] = wz;
}

const BaseRobotControl::RobotObsResult SDK2PolicyController::getRobotObs() {
    if (standalone_) {
        // In standalone mode, we might want to get velocity commands from a joystick.
        // This part needs to be connected to the joystick data source if required.
    }

    unitree_go::msg::dds_::LowState_ state_copy{};
    {
        std::lock_guard<std::mutex> lk(low_state_mtx_);
        state_copy = low_state_;
    }

    // 1) cmd (3) scaled
    std::array<float, 3> cmd_scaled = last_cmd_;
    cmd_scaled[0] *= obs_cfg_.vel_scale;
    cmd_scaled[1] *= obs_cfg_.vel_scale;
    cmd_scaled[2] *= obs_cfg_.gyr_scale;

    // 2) IMU gyr (3) scaled and clipped
    std::array<float, 3> gyr{0, 0, 0};
    for (int i = 0; i < 3; ++i) {
        gyr[i] = state_copy.imu_state().gyroscope()[i] * obs_cfg_.gyr_scale;
    }

    // 3) grav (3) from quaternion (w,x,y,z)
    std::array<float, 4> q{1, 0, 0, 0};
    if (state_copy.imu_state().quaternion().size() >= 4) {
        q[0] = state_copy.imu_state().quaternion()[0];
        q[1] = state_copy.imu_state().quaternion()[1];
        q[2] = state_copy.imu_state().quaternion()[2];
        q[3] = state_copy.imu_state().quaternion()[3];
    }
    std::array<float, 3> grav = gravFromQuatWxyz(q);

    // 4) dof pos/vel (obs_cfg_.act_size, 12/16) in sdk order
    std::vector<float> dof_pos_sdk(obs_cfg_.act_size), dof_vel_sdk(obs_cfg_.act_size);
    const auto& ms = state_copy.motor_state();
    for (int i = 0; i < obs_cfg_.act_size; ++i) {
        dof_pos_sdk[i] = ms[i].q() - obs_cfg_.dft_dof_pos[i];
        dof_vel_sdk[i] = ms[i].dq();
    }
    // in policy order (by using joint_idx_sdk2policy)
    std::vector<float> dof_pos(obs_cfg_.act_size), dof_vel(obs_cfg_.act_size);
    for (int pi = 0; pi < obs_cfg_.act_size; ++pi) {
        const int ri = obs_cfg_.joint_idx_sdk2policy[pi];
        dof_pos[pi] = dof_pos_sdk[ri] * obs_cfg_.dof_pos_scale;
        dof_vel[pi] = dof_vel_sdk[ri] * obs_cfg_.dof_vel_scale;
    }

    // 5) act (obs_cfg_.act_size) in policy order
    const std::vector<float>& act = last_act_;

    // Concatenate into obs
    std::vector<float> obs;
    obs.reserve(obs_cfg_.obs_size);  // 12/16
    obs.insert(obs.end(), gyr.begin(), gyr.end());
    obs.insert(obs.end(), grav.begin(), grav.end());
    obs.insert(obs.end(), cmd_scaled.begin(), cmd_scaled.end());
    obs.insert(obs.end(), dof_pos.begin(), dof_pos.end());
    obs.insert(obs.end(), dof_vel.begin(), dof_vel.end());
    obs.insert(obs.end(), act.begin(), act.end());

    // Clip
    for (auto& v : obs) {
        v = clip(v, -obs_cfg_.obs_clip, obs_cfg_.obs_clip);
    }

    // Update history buffer: shift right by one frame, put latest at front
    const std::size_t frame = static_cast<std::size_t>(obs_cfg_.obs_size);
    const std::size_t total = frame * obs_cfg_.history_steps;
    if (his_obs_.size() != total) {
        ensureObsBuffers();
    }
    if (obs_cfg_.history_steps > 1) {
        std::memmove(his_obs_.data() + frame, his_obs_.data(), sizeof(float) * frame * (obs_cfg_.history_steps - 1));
    }
    std::memcpy(his_obs_.data(), obs.data(), sizeof(float) * frame);

    RobotObsResult res;
    res.his_obs = his_obs_;  // return a copy

    static uint64_t obs_cnt = 0;
    if ((++obs_cnt % 50) == 0) {
        std::cout << "[SDK2 | PolicyController] Current robot obs: ";
        std::copy(obs.begin(), obs.end(), std::ostream_iterator<float>(std::cout, " "));
        std::cout << std::endl;
    }
    return res;
}

void SDK2PolicyController::onLowStateMessage(const void* msg) {
    std::lock_guard<std::mutex> lk(low_state_mtx_);
    low_state_ = *reinterpret_cast<const unitree_go::msg::dds_::LowState_*>(msg);
}

void SDK2PolicyController::initLowCmd() {
    // 参考官方示例
    low_cmd_.head()[0] = 0xFE;
    low_cmd_.head()[1] = 0xEF;
    low_cmd_.level_flag() = 0xFF;
    low_cmd_.gpio() = 0;

    for (int i = 0; i < static_cast<int>(low_cmd_.motor_cmd().size()); ++i) {
        low_cmd_.motor_cmd()[i].mode() = 0x01;  // PMSM 伺服
        low_cmd_.motor_cmd()[i].q()    = PosStopF;
        low_cmd_.motor_cmd()[i].kp()   = 0;
        low_cmd_.motor_cmd()[i].dq()   = VelStopF;
        low_cmd_.motor_cmd()[i].kd()   = 0;
        low_cmd_.motor_cmd()[i].tau()  = 0;
    }
    std::cout << "[SDK2 | PolicyController] initLowCmd: motor_cmd.size = " << low_cmd_.motor_cmd().size() << std::endl;
}

uint32_t SDK2PolicyController::crc32_core(uint32_t* ptr, uint32_t len) {
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++) {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else {
                CRC32 <<= 1;
            }
            if (data & xbit) CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}