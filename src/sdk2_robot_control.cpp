#include "sdk2_robot_control.h"
#include <iostream>
#include <algorithm>
#include <atomic>
#include <fstream>
#include <sstream>
#include <cctype>
#include <iterator>
#include "utils.h"

// 常量与示例一致
static constexpr double PosStopF = (2.146E+9f);
static constexpr double VelStopF = (16000.0f);

using unitree::robot::ChannelFactory;
using unitree::robot::ChannelPublisher;
using unitree::robot::ChannelPublisherPtr;
using unitree::robot::ChannelSubscriber;
using unitree::robot::ChannelSubscriberPtr;

SDK2RobotControl::SDK2RobotControl(const std::string &network_interface, double timeout_s, bool auto_stand, const std::string& config_path)
    : BaseRobotControl()
{
    std::cout << "[SDK2] ctor iface=" << network_interface
              << " timeout_s=" << timeout_s
              << " auto_stand=" << (auto_stand ? "true" : "false") << std::endl;

    ChannelFactory::Instance()->Init(0, network_interface.c_str());

    sport_client_ = std::make_unique<unitree::robot::go2::SportClient>();
    sport_client_->SetTimeout(static_cast<float>(timeout_s));
    sport_client_->Init();
    std::cout << "[SDK2] SportClient initialized" << std::endl;

    if (auto_stand) {
        int res = sport_client_->StandUp();
        std::cout << "[SDK2] Auto StandUp return code: " << res << std::endl;
    }

    // 低层发布/订阅初始化
    lowcmd_pub_.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_pub_->InitChannel();
    std::cout << "[SDK2] LowCmd publisher initialized: " << TOPIC_LOWCMD << std::endl;

    lowstate_sub_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_sub_->InitChannel(std::bind(&SDK2RobotControl::onLowStateMessage, this, std::placeholders::_1), 1);

    // 不在构造函数中释放高层模式，默认以 HighLevel 启动，便于调用 SportClient  // todo: fix this bug
    msc_ = std::make_unique<unitree::robot::b2::MotionSwitcherClient>();
    msc_->SetTimeout(static_cast<float>(timeout_s));
    msc_->Init();

    // // 初始化低层命令
    initLowCmd();
    std::cout << "[SDK2] initLowCmd done" << std::endl;

    // Load config
    config_path_ = config_path;
    bool ok = false;
    obs_cfg_ = PolicyConfig::FromFile(config_path, &ok);
    std::cout << "[SDK2] Load config: " << config_path << (ok ? " [OK]" : " [ERR]") << std::endl;

    // Ensure buffers
    ensureObsBuffers();
    std::cout << "[SDK2] Observation buffers prepared: obs_size=" << obs_cfg_.obs_size
              << " history_steps=" << obs_cfg_.history_steps << std::endl;
    
    size_t pos = config_path_.find_last_of('/');
    std::string parent_dir = config_path_.substr(0, pos + 1);
    std::string onnx_path = parent_dir + obs_cfg_.onnx_model_path;
    std::cout << "[SDK2] Load onnx_path: " << onnx_path << std::endl;
    loadOnnxModel(onnx_path);

    // Start control loop thread
    // todo: add joystick to reset and begin control loop
    controlLoopThreadPtr_ = unitree::common::CreateRecurrentThreadEx(
        "controlLoopThread", UT_CPU_ID_NONE, 20000, &SDK2RobotControl::controlLoop, this);
    std::cout << "[SDK2] Control loop thread started" << std::endl;
}

SDK2RobotControl::~SDK2RobotControl()
{
    std::cout << "[SDK2] dtor: stopping..." << std::endl;
    if (sport_client_) {
        sport_client_->StopMove();
    }
}

void SDK2RobotControl::resetJointPosition()
{
    const int steps = 200; // 2 seconds at 10ms intervals
    const double interval = 0.01; // 10ms
    std::array<double, 12> current_positions{};
    std::array<double, 12> target_positions{};
    
    // Initialize current and target positions
    for (int i = 0; i < 12; ++i) {
        current_positions[i] = low_state_.motor_state()[i].q();
        target_positions[i] = static_cast<double>(obs_cfg_.dft_dof_pos[i]);
    }

    for (int step = 0; step <= steps; ++step) {
        std::array<double, 12> interpolated_positions{};
        for (int i = 0; i < 12; ++i) {
            interpolated_positions[i] = current_positions[i] + 
                (target_positions[i] - current_positions[i]) * (static_cast<double>(step) / steps);
        }
        applyPositionControl(interpolated_positions);
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(interval * 1000)));
    }
}

void SDK2RobotControl::applyPositionControl(const std::array<double, 12> &joint_positions)
{
    // 切换到低层
    setControlMode(ControlMode::LowLevel);
    // 写入目标并由后台循环持续发布
    // std::lock_guard<std::mutex> lk(low_cmd_mtx_);
    for (int i = 0; i < 12; i++) {
        low_cmd_.motor_cmd()[i].mode() = 0x01; // PMSM 伺服
        low_cmd_.motor_cmd()[i].q()    = static_cast<float>(joint_positions[i]);
        low_cmd_.motor_cmd()[i].dq()   = 0.0f;
        low_cmd_.motor_cmd()[i].kp()   = obs_cfg_.kp;
        low_cmd_.motor_cmd()[i].kd()   = obs_cfg_.kd;
        low_cmd_.motor_cmd()[i].tau()  = 0.0f; // 安全起见不叠加力矩
    }
    low_cmd_.crc() = crc32_core(reinterpret_cast<uint32_t*>(&low_cmd_), (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_pub_->Write(low_cmd_);
    // todo: add go2w wheel control
}

void SDK2RobotControl::applyVelCmdControl(double vx, double vy, double wz)
{
    // Store last velocity command for observations
    last_cmd_[0] = static_cast<float>(vx);
    last_cmd_[1] = static_cast<float>(vy);
    last_cmd_[2] = static_cast<float>(wz);
}

const BaseRobotControl::RobotObsResult SDK2RobotControl::getRobotObs()
{
    unitree_go::msg::dds_::LowState_ state_copy{};
    {
        std::lock_guard<std::mutex> lk(low_state_mtx_);
        state_copy = low_state_;
    }
    
    // 1) cmd (3)
    std::array<float, 3> cmd_scaled = last_cmd_;
    cmd_scaled[0] *= obs_cfg_.vel_scale;
    cmd_scaled[1] *= obs_cfg_.vel_scale;
    cmd_scaled[2] *= obs_cfg_.gyr_scale;

    // 2) IMU gyr (3) scaled and clipped
    std::array<float, 3> gyr{0, 0, 0};
    for (int i = 0; i < 3; ++i) {
        float g = state_copy.imu_state().gyroscope()[i];
        gyr[i] = g * obs_cfg_.gyr_scale;
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

    // 4) dof pos/vel (12) todo: 12 -> 16
    std::array<float, 12> dof_pos_sdk{};
    std::array<float, 12> dof_vel_sdk{};
    const auto& ms = state_copy.motor_state();
    // const int n = std::min<int>(12, ms.size());
    for (int i = 0; i < 12; ++i) {
        dof_pos_sdk[i] = ms[i].q();
        dof_vel_sdk[i] = ms[i].dq();
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
        // const int ri_clamped = std::clamp(ri, 0, 11);
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
        std::copy(obs.begin(), obs.end(), std::ostream_iterator<float>(std::cout, " "));
        std::cout << std::endl;
    }
    return res;
}

void SDK2RobotControl::setControlMode(ControlMode mode)
{
    if (mode_ == mode) return;
    std::cout << "[SDK2] setControlMode: " << static_cast<int>(mode_) << " -> " << static_cast<int>(mode) << std::endl;

    if (mode == ControlMode::LowLevel) {
        // 切换到低层：停止上层运动，释放运动模式，并启动低层循环
        (void)sport_client_->StopMove();
        releaseMotionModeIfNeeded();
        mode_ = ControlMode::LowLevel;
        std::cout << "[SDK2] Switched to LowLevel (joint position) mode." << std::endl;
    } else {
        // 切换到高层：停止低层循环；上层命令会在 move/stopMove 中发送
        mode_ = ControlMode::HighLevel;
        std::cout << "[SDK2] Switched to HighLevel (SportClient) mode." << std::endl;
    }
}

int SDK2RobotControl::move(double vx, double vy, double wz)
{
    // 切换到高层
    setControlMode(ControlMode::HighLevel);

    // 更新上次速度命令以便观测
    last_cmd_[0] = static_cast<float>(vx);
    last_cmd_[1] = static_cast<float>(vy);
    last_cmd_[2] = static_cast<float>(wz);

    std::cout << "[SDK2] Move cmd: vx=" << vx << " vy=" << vy << " wz=" << wz << std::endl;
    int ret = sport_client_->Move(vx, vy, wz);
    std::cout << "[SDK2] Move ret=" << ret << std::endl;
    return ret;
}

int SDK2RobotControl::stopMove()
{
    // 切换到高层（确保停止上层动作）
    setControlMode(ControlMode::HighLevel);
    std::cout << "[SDK2] StopMove requested" << std::endl;
    int ret = sport_client_->StopMove();
    std::cout << "[SDK2] StopMove ret=" << ret << std::endl;
    return ret;
}

void SDK2RobotControl::initLowCmd()
{
    // 参考官方示例
    low_cmd_.head()[0] = 0xFE;
    low_cmd_.head()[1] = 0xEF;
    low_cmd_.level_flag() = 0xFF;
    low_cmd_.gpio() = 0;

    for (int i = 0; i < static_cast<int>(low_cmd_.motor_cmd().size()); ++i) {
        low_cmd_.motor_cmd()[i].mode() = 0x01; // PMSM 伺服
        low_cmd_.motor_cmd()[i].q()    = PosStopF;
        low_cmd_.motor_cmd()[i].kp()   = 0;
        low_cmd_.motor_cmd()[i].dq()   = VelStopF;
        low_cmd_.motor_cmd()[i].kd()   = 0;
        low_cmd_.motor_cmd()[i].tau()  = 0;
    }
    std::cout << "[SDK2] initLowCmd: motor_cmd.size=" << low_cmd_.motor_cmd().size() << std::endl;
}

void SDK2RobotControl::onLowStateMessage(const void* msg)
{
    std::lock_guard<std::mutex> lk(low_state_mtx_);
    low_state_ = *reinterpret_cast<const unitree_go::msg::dds_::LowState_*>(msg);
}

int SDK2RobotControl::queryMotionStatus()
{
    std::string robotForm,motionName;
    int motionStatus;
    int32_t ret = msc_->CheckMode(robotForm,motionName);
    if (ret == 0) {
        std::cout << "CheckMode succeeded." << std::endl;
    } else {
        std::cout << "CheckMode failed. Error code: " << ret << std::endl;
    }
    if(motionName.empty())
    {
        std::cout << "The motion control-related service is deactivated." << std::endl;
        motionStatus = 0;
    }
    else
    {
        std::string serviceName = "";
        if(robotForm == "0")
        {
            if(motionName == "normal" ) serviceName = "sport_mode"; 
            if(motionName == "ai" ) serviceName = "ai_sport"; 
            if(motionName == "advanced" ) serviceName = "advanced_sport"; 
        }
        else
        {
            if(motionName == "ai-w" ) serviceName = "wheeled_sport(go2W)"; 
            if(motionName == "normal-w" ) serviceName = "wheeled_sport(b2W)";
        }
        std::cout << "Service: "<< serviceName<< " is activate" << std::endl;
        motionStatus = 1;
    }
    return motionStatus;
}

void SDK2RobotControl::releaseMotionModeIfNeeded()
{   
    while(queryMotionStatus())
    {
        std::cout << "Try to deactivate the motion control-related service." << std::endl;
        int32_t ret = msc_->ReleaseMode(); 
        if (ret == 0) {
            std::cout << "ReleaseMode succeeded." << std::endl;
        } else {
            std::cout << "ReleaseMode failed. Error code: " << ret << std::endl;
        }
        sleep(5);
    }
    std::cout << "[SDK2] releaseMotionModeIfNeeded: ok" << std::endl;
}

uint32_t SDK2RobotControl::crc32_core(uint32_t* ptr, uint32_t len)
{
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