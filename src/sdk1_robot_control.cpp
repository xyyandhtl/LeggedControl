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

    udp_.InitCmdData(cmd_);  // 初始化UDP通信
    // 并进行第一次数据收发
    cmd_.levelFlag = LOWLEVEL;
    udp_.Recv();
    udp_.GetRecv(state_);  // 将解析后的状态存入 state_
    udp_.SetSend(cmd_);  // 将 cmd_ 中的数据设置为发送数据
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

    // loop_udpSend and loop_udpRecv can be merged to one thread，创建三个线程，分别用于：发送UDP数据、接收UDP数据 和 执行主控制循环
    loop_udpSend = std::make_unique<LoopFunc>(
        "udp_send", control_dt_, 3, boost::bind(&SDK1RobotControl::udpSend, this));
    loop_udpRecv = std::make_unique<LoopFunc>(
        "udp_recv", control_dt_, 3, boost::bind(&SDK1RobotControl::udpRecv, this));
    loop_control = std::make_unique<LoopFunc>(
        "control_loop", 0.02, boost::bind(&SDK1RobotControl::controlLoop, this));
    loop_udpSend->start();
    loop_udpRecv->start();

    // 等待手柄按下 "up" 按钮，则复位机器狗的关节位置
    std::cout << "[SDK1] Waiting for joystick to reset and begin loop_control..." << std::endl;
    while (getJoystickData().btn.components.up != 1) {
        // std::cout << state_.motorState[1].q << std::endl;
        std::this_thread::sleep_for(std::chrono::duration<double>(control_dt_));
    }
    resetJointPosition();
    // 等待手柄按下 "up" 按钮，则启动主控制循环
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
    const float max_time = 5.0;       // 重置时间阈值，seconds
    const float control_freq = 20.0;  // Hz
    const float act_clip = 0.1;       // 每步最大角度阈值，rad per step

    // 1) Read current joint positions (thread-safe)
    std::vector<float> joint_pos(obs_cfg_.act_size);
    std::vector<float> dft_dof_pos(obs_cfg_.act_size);
    for (int i = 0; i < obs_cfg_.act_size; ++i) {
        joint_pos[i] = state_.motorState[i].q;
        dft_dof_pos[i] = obs_cfg_.dft_dof_pos[i];
        std::cout << "[" << joint_pos[i] << "|" << dft_dof_pos[i] << "], ";
    }
    std::cout << std::endl;

    // 2) Compute number of steps based on max delta and act_clip
    float act_max = 0.0;  // 所有关节中的 最大角度偏差
    for (int i = 0; i < obs_cfg_.act_size; ++i) {
        const float diff = std::abs(joint_pos[i] - dft_dof_pos[i]);
        if (diff > act_max) act_max = diff;
    }
    const int num_steps = std::max(1, static_cast<int>(std::ceil(act_max / act_clip)));  // 所需调整的 步数
    const auto interval = std::chrono::duration<float>(1.0 / control_freq);

    // 3) Progressive reset with timeout check
    const auto start_time = std::chrono::steady_clock::now();
    for (int step = 0; step < num_steps; ++step) {
        // 重置时间超时检测
        const auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<float>(now - start_time).count() > max_time) {
            std::cerr << "[SDK1] RESET FAILED: timeout after " << max_time << "s" << std::endl;
            break;
        }
        // 插值计算 当前调整步 的各关节位置
        const float ratio = static_cast<float>(step + 1) / static_cast<float>(num_steps);
        std::vector<float> interp(obs_cfg_.act_size);
        for (int i = 0; i < obs_cfg_.act_size; ++i) {
            interp[i] = joint_pos[i] * (1.0 - ratio) + dft_dof_pos[i] * ratio;
            // std::cout << interp[i] << ", ";
        }
        // std::cout << std::endl;
        for (int i = 0; i < obs_cfg_.act_size; i++) {
            cmd_.motorCmd[i].q  = interp[i];
            cmd_.motorCmd[i].dq = 0.0;
            cmd_.motorCmd[i].Kp = 80.0;
            cmd_.motorCmd[i].Kd = 2.0;
            cmd_.motorCmd[i].tau = 0.0f;
        }
        std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(interval));
    }
}

/**
 * 位置控制：将 输入的关节位置 以及 配置文件中的 刚度、阻尼 赋值给 cmd_ 中的 电机控制命令
 */
void SDK1RobotControl::applyPositionControl(std::vector<float>& joint_positions)
{
    for (std::size_t i = 0; i < obs_cfg_.act_size; i++) {
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

void SDK1RobotControl::applyVelCmdControl(float vx, float vy, float wz)
{
    last_cmd_[0] = vx;
    last_cmd_[1] = vy;
    last_cmd_[2] = wz;
}

void SDK1RobotControl::udpRecv()
{
    udp_.Recv();
    udp_.GetRecv(state_);
    std::memcpy(&key_data_, &state_.wirelessRemote, sizeof(xRockerBtnDataStruct));  // 从 state_ 中获取 手柄数据 至 key_data_
}

void SDK1RobotControl::udpSend()
{
    // For debug
    // static uint64_t rec_cnt = 0;
    // if ((++rec_cnt % 500) == 0) {
    //     std::cout << "current joint pos: ";
    //     for (int i = 0; i < obs_cfg_.act_size; ++i) {
    //         std::cout << state_.motorState[i].q << ", ";
    //     }
    //     std::cout << std::endl;
    //     std::cout << "current cmd pos: ";
    //     for (int i = 0; i < obs_cfg_.act_size; ++i) {
    //         std::cout << cmd_.motorCmd[i].q << ", ";
    //     }
    //     std::cout << std::endl;
    // }
    safe_.PowerProtect(cmd_, state_, 8);  // 根据 要发送给机器狗的命令 及 机器狗的状态 判定 是否触发功率保护（关节电机限制功率的 80%）
    // safe_.PositionProtect(cmd, state, 0.087);  // 是否触发位置突变保护（5度）
    udp_.SetSend(cmd_);
    udp_.Send();
}

void SDK1RobotControl::stopMotors()
{
    for (int i = 0; i < obs_cfg_.act_size; i++) {
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
