#include "sdk1_robot_control.h"
#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <cctype>
#include <iterator>
#include "utils.h"

SDK1RobotControl::SDK1RobotControl(uint16_t local_port, const std::string &target_ip, uint16_t target_port, const std::string& config_path)
    : BaseRobotControl(), 
      udp_(local_port, target_ip.c_str(), target_port, LOW_CMD_LENGTH, LOW_STATE_LENGTH, -1),
      safe_(LeggedType::Aliengo) 
{
    std::cout << "[SDK1] local_port=" << local_port
              << " target_ip=" << target_ip
              << " target_port=" << target_port << std::endl;

    udp_.InitCmdData(cmd_);
    cmd_.levelFlag = LOWLEVEL;
    udp_.Recv();
    udp_.GetRecv(state_);
    udp_.SetSend(cmd_);
    udp_.Send();

    // Load config
    config_path_ = config_path;
    bool ok = false;
    obs_cfg_ = PolicyConfig::FromFile(config_path, &ok);
    std::cout << "[SDK1] Load config: " << config_path << (ok ? " [OK]" : " [ERR]") << std::endl;

    // Ensure buffers
    ensureObsBuffers();
    std::cout << "[SDK1] Observation buffers prepared: obs_size=" << obs_cfg_.obs_size
              << " history_steps=" << obs_cfg_.history_steps << std::endl;

    // Load ONNX model
    std::string onnx_path = config_path.substr(0, config_path.find_last_of('/') + 1) + obs_cfg_.onnx_model_path;
    loadOnnxModel(onnx_path);

    // loop_udpSend and loop_udpRecv can be merged to one thread
    loop_udpSend = std::make_unique<LoopFunc>(
        "udp_send", control_dt_, 3, boost::bind(&SDK1RobotControl::udpSend, this));
    loop_udpRecv = std::make_unique<LoopFunc>(
        "udp_recv", control_dt_, 3, boost::bind(&SDK1RobotControl::udpRecv, this));
    loop_control = std::make_unique<LoopFunc>(
        "control_loop", 0.02, boost::bind(&SDK1RobotControl::controlLoop, this));
    loop_udpSend->start();
    loop_udpRecv->start();

    std::cout << "[SDK1] Waiting for joystick to reset and begin loop_control..." << std::endl;
    while (getJoystickData().btn.components.up != 1) {
        // std::cout << state_.motorState[1].q << std::endl;
        std::this_thread::sleep_for(std::chrono::duration<double>(control_dt_));
    }
    resetJointPosition();
    while (getJoystickData().btn.components.up != 1) {
        std::this_thread::sleep_for(std::chrono::duration<double>(control_dt_));
    }
    loop_control->start();
}

SDK1RobotControl::~SDK1RobotControl()
{
    if (loop_udpSend) {
        loop_udpSend->shutdown();
        loop_udpSend.reset();
    }
    if (loop_udpRecv) {
        loop_udpRecv->shutdown();
        loop_udpRecv.reset();
    }
    if (loop_control) {
        loop_control->shutdown();
        loop_control.reset();
    }
    std::cout << "[SDK1] dtor: stopping..." << std::endl;
}

void SDK1RobotControl::resetJointPosition()
{
    // Parameters analogous to the Python logic
    const double max_time = 5.0;       // seconds
    const double control_freq = 20.0;  // Hz
    const double act_clip = 0.1;       // rad per step

    // 1) Read current joint positions (thread-safe)
    std::array<double, 12> joint_pos{};
    std::array<double, 12> dft_dof_pos{};
    for (int i = 0; i < 12; ++i) {
        joint_pos[i] = static_cast<double>(state_.motorState[i].q);
        dft_dof_pos[i] = static_cast<double>(obs_cfg_.dft_dof_pos[i]);
        std::cout << "[" << joint_pos[i] << "|" << dft_dof_pos[i] << "], ";
    }
    std::cout << std::endl;

    // 2) Compute number of steps based on max delta and act_clip
    double act_max = 0.0;
    for (int i = 0; i < 12; ++i) {
        const double diff = std::abs(joint_pos[i] - dft_dof_pos[i]);
        if (diff > act_max) act_max = diff;
    }
    const int num_steps = std::max(1, static_cast<int>(std::ceil(act_max / act_clip)));
    const auto interval = std::chrono::duration<double>(1.0 / control_freq);

    // 3) Progressive reset with timeout check
    const auto start_time = std::chrono::steady_clock::now();
    for (int step = 0; step < num_steps; ++step) {
        const auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - start_time).count() > max_time) {
            std::cerr << "[SDK1] RESET FAILED: timeout after " << max_time << "s" << std::endl;
            break;
        }

        const double ratio = static_cast<double>(step + 1) / static_cast<double>(num_steps);
        std::array<double, 12> interp{};
        for (int i = 0; i < 12; ++i) {
            interp[i] = joint_pos[i] * (1.0 - ratio) + dft_dof_pos[i] * ratio;
            // std::cout << interp[i] << ", ";
        }
        // std::cout << std::endl;

        for (int i = 0; i < 12; i++) {
            cmd_.motorCmd[i].q  = interp[i];
            cmd_.motorCmd[i].dq = 0.0;
            cmd_.motorCmd[i].Kp = 80.0;
            cmd_.motorCmd[i].Kd = 2.0;
            cmd_.motorCmd[i].tau = 0.0f;
        }
        std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(interval));
    }
}

void SDK1RobotControl::applyPositionControl(const std::array<double, 12> &joint_positions)
{
    for (int i = 0; i < 12; i++) {
        cmd_.motorCmd[i].q  = joint_positions[i];
        cmd_.motorCmd[i].dq = 0.0;
        cmd_.motorCmd[i].Kp = obs_cfg_.kp;
        cmd_.motorCmd[i].Kd = obs_cfg_.kd;
        cmd_.motorCmd[i].tau = 0.0f;
    }
    // Gravity compensation
    // cmd_.motorCmd[FR_0].tau = -1.6f;
    // cmd_.motorCmd[FL_0].tau = -1.6f;
    // cmd_.motorCmd[RR_0].tau = -1.6f;
    // cmd_.motorCmd[RL_0].tau = -1.6f;
}

void SDK1RobotControl::applyVelCmdControl(double vx, double vy, double wz)
{
    last_cmd_[0] = static_cast<float>(vx);
    last_cmd_[1] = static_cast<float>(vy);
    last_cmd_[2] = static_cast<float>(wz);
}

const BaseRobotControl::RobotObsResult SDK1RobotControl::getRobotObs()
{
    // If test non-ros, uncomment this line
    // applyVelCmdControl(key_data_.ly * max_vx_, key_data_.lx * max_vy_, key_data_.rx * max_wz_);

    LowState state_copy = {0};
    {
        std::lock_guard<std::mutex> lk(low_state_mtx_);
        state_copy = state_;
    }

    // 1) cmd (3)
    std::array<float, 3> cmd_scaled = last_cmd_;
    cmd_scaled[0] *= obs_cfg_.vel_scale;
    cmd_scaled[1] *= obs_cfg_.vel_scale;
    cmd_scaled[2] *= obs_cfg_.gyr_scale;

    // 2) IMU gyr (3) scaled and clipped
    std::array<float, 3> gyr = state_copy.imu.gyroscope;
    for (int i = 0; i < 3; ++i) {
        gyr[i] *= obs_cfg_.gyr_scale;
    }

    // 3) grav (3) from quaternion (w,x,y,z)
    std::array<float, 3> grav = gravFromQuatWxyz(state_copy.imu.quaternion);

    // 4) dof_pos (12)
    std::array<float, 12> dof_pos_sdk{};
    std::array<float, 12> dof_vel_sdk{};
    for (int i = 0; i < 12; ++i) {
        dof_pos_sdk[i] = static_cast<float>(state_copy.motorState[i].q);
        dof_vel_sdk[i] = static_cast<float>(state_copy.motorState[i].dq);
    }
    // Subtract default offsets
    for (int i = 0; i < 12; ++i) {
        dof_pos_sdk[i] -= obs_cfg_.dft_dof_pos[i];
    }
    // Reorder robot->policy using joint_idx_sdk2policy
    std::array<float, 12> dof_pos{};
    std::array<float, 12> dof_vel{};
    for (int pi = 0; pi < 12; ++pi) {
        const int ri = obs_cfg_.joint_idx_sdk2policy[pi];
        dof_pos[pi] = dof_pos_sdk[ri] * obs_cfg_.dof_pos_scale;
        dof_vel[pi] = dof_vel_sdk[ri] * obs_cfg_.dof_vel_scale;
    }

    // 5) act (12) in policy order
    const std::array<float, 12>& act = last_act_;

    // Concatenate into obs
    const int obs_size = obs_cfg_.obs_size;
    std::vector<float> obs;
    obs.reserve(obs_size);

    obs.insert(obs.end(), cmd_scaled.begin(), cmd_scaled.end());
    obs.insert(obs.end(), gyr.begin(), gyr.end());
    obs.insert(obs.end(), grav.begin(), grav.end());
    obs.insert(obs.end(), dof_pos.begin(), dof_pos.end());
    obs.insert(obs.end(), dof_vel.begin(), dof_vel.end());
    obs.insert(obs.end(), act.begin(), act.end());

    // Clip
    for (auto& v : obs) {
        v = clip(v, -obs_cfg_.obs_clip, obs_cfg_.obs_clip);
    }

    // Update history buffer: shift right by one frame, put latest at front
    const std::size_t frame = static_cast<std::size_t>(obs_size);
    const std::size_t total = frame * obs_cfg_.history_steps;
    if (his_obs_.size() != total) {
        ensureObsBuffers();
    }
    if (obs_cfg_.history_steps > 1) {
        std::memmove(his_obs_.data() + frame, his_obs_.data(), sizeof(float) * frame * (obs_cfg_.history_steps - 1));
    }
    std::memcpy(his_obs_.data(), obs.data(), sizeof(float) * frame);

    RobotObsResult res;
    res.his_obs = his_obs_; // return a copy
    
    static uint64_t obs_cnt = 0;
    if ((++obs_cnt % 50) == 0) {
        std::cout << "Current robot obs: ";
        std::copy(obs.begin(), obs.end(), std::ostream_iterator<float>(std::cout, " "));
        std::cout << std::endl;
    }
    return res;
}

void SDK1RobotControl::udpRecv()
{
    udp_.Recv();
    udp_.GetRecv(state_);
    std::memcpy(&key_data_, &state_.wirelessRemote, sizeof(xRockerBtnDataStruct));
}

void SDK1RobotControl::udpSend()
{
    // For debug
    // static uint64_t rec_cnt = 0;
    // if ((++rec_cnt % 500) == 0) {
    //     std::cout << "current joint pos: ";
    //     for (int i = 0; i < 12; ++i) {
    //         std::cout << state_.motorState[i].q << ", ";
    //     }
    //     std::cout << std::endl;
    //     std::cout << "current cmd pos: ";
    //     for (int i = 0; i < 12; ++i) {
    //         std::cout << cmd_.motorCmd[i].q << ", ";
    //     }
    //     std::cout << std::endl;
    // }
    safe_.PowerProtect(cmd_, state_, 8);
    // safe_.PositionProtect(cmd, state, 0.087);
    udp_.SetSend(cmd_);
    udp_.Send();
}

void SDK1RobotControl::stopMotors()
{
    for (int i = 0; i < 12; i++) {
        cmd_.motorCmd[i].q = PosStopF;
        cmd_.motorCmd[i].dq = VelStopF;
        cmd_.motorCmd[i].Kp = 0;
        cmd_.motorCmd[i].Kd = 0;
        cmd_.motorCmd[i].tau = 0.0f;
    }
}

xRockerBtnDataStruct SDK1RobotControl::getJoystickData() const
{
    return key_data_;
}

