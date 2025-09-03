#include "sdk1_robot_control_him.h"
#include "utils.h"
#include <iostream>
#include <iterator>
#include <vector>


SDK1RobotControlHIM::SDK1RobotControlHIM(uint16_t local_port, const std::string &target_ip, uint16_t target_port, const std::string& config_path)
    : SDK1RobotControl(local_port, target_ip, target_port, config_path)
{
    std::cout << "[SDK1_HIM] Constructor: Inheriting from SDK1RobotControl." << std::endl;
}

// 重写 getRobotObs
const BaseRobotControl::RobotObsResult SDK1RobotControlHIM::getRobotObs()
{
    if (standalone_) {
        applyVelCmdControl(key_data_.ly * 1.0, key_data_.lx * 0.5, key_data_.rx * 1.0);
    }

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

    // 4) dof_pos (obs_cfg_.act_size)
    std::vector<float> dof_pos_sdk(obs_cfg_.act_size);
    std::vector<float> dof_vel_sdk(obs_cfg_.act_size);
    for (int i = 0; i < obs_cfg_.act_size; ++i) {
        dof_pos_sdk[i] = state_copy.motorState[i].q;
        dof_vel_sdk[i] = state_copy.motorState[i].dq;
    }
    // Subtract default offsets
    for (int i = 0; i < obs_cfg_.act_size; ++i) {
        dof_pos_sdk[i] -= obs_cfg_.dft_dof_pos[i];
    }
    // Reorder robot->policy using joint_idx_sdk2policy
    std::vector<float> dof_pos(obs_cfg_.act_size);
    std::vector<float> dof_vel(obs_cfg_.act_size);
    for (int pi = 0; pi < obs_cfg_.act_size; ++pi) {
        const int ri = obs_cfg_.joint_idx_sdk2policy[pi];
        dof_pos[pi] = dof_pos_sdk[ri] * obs_cfg_.dof_pos_scale;
        dof_vel[pi] = dof_vel_sdk[ri] * obs_cfg_.dof_vel_scale;
    }

    // 5) act in policy order
    const std::vector<float>& act = last_act_;

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