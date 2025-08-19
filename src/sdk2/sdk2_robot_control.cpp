#include "sdk2_robot_control.h"
#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>
#include <algorithm>
#include <atomic>
#include <fstream>
#include <sstream>
#include <cctype>
#include <iterator>
#include "../common/utils.h"
#include <unitree/common/thread/thread.hpp>

// 常量与示例一致
static constexpr double PosStopF = (2.146E+9f);
static constexpr double VelStopF = (16000.0f);

using unitree::robot::ChannelFactory;
using unitree::robot::ChannelPublisher;
using unitree::robot::ChannelPublisherPtr;
using unitree::robot::ChannelSubscriber;
using unitree::robot::ChannelSubscriberPtr;

SDK2PolicyConfig SDK2PolicyConfig::FromFile(const std::string& path, bool* ok) {
    SDK2PolicyConfig cfg;
    std::ifstream fin(path);
    bool good = fin.good();
    if (!fin) {
        if (ok) *ok = false;
        std::cerr << "[SDK2] Config open failed: " << path << std::endl;
        return cfg;
    }
    std::string line;
    while (std::getline(fin, line)) {
        // strip comments
        auto p = line.find_first_of("#;");
        if (p != std::string::npos) line = line.substr(0, p);
        p = line.find("//");
        if (p != std::string::npos) line = line.substr(0, p);
        line = trim(line);
        if (line.empty()) continue;

        // split by '=' or ':'
        size_t sep = line.find('=');
        if (sep == std::string::npos) sep = line.find(':');
        if (sep == std::string::npos) continue;
        std::string key = trim(line.substr(0, sep));
        std::string val = trim(line.substr(sep + 1));

        if (key == "onnx_model_path") cfg.onnx_model_path = val;
        else if (key == "act_scale") cfg.act_scale = std::stof(val);
        else if (key == "vel_scale") cfg.vel_scale = std::stof(val);
        else if (key == "gyr_scale") cfg.gyr_scale = std::stof(val);
        else if (key == "dof_pos_scale") cfg.dof_pos_scale = std::stof(val);
        else if (key == "dof_vel_scale") cfg.dof_vel_scale = std::stof(val);
        else if (key == "obs_clip") cfg.obs_clip = std::stof(val);
        else if (key == "history_steps") cfg.history_steps = static_cast<std::size_t>(std::stoul(val));
        else if (key == "obs_size") cfg.obs_size = std::stoi(val);
        else if (key == "kp") cfg.kp = std::stof(val);
        else if (key == "kd") cfg.kd = std::stof(val);
        else if (key == "dft_dof_pos") {
            std::cout << "[SDK2]Parseing dft_dof_pos: " << std::endl;
            if (!parse_list<float, 12>(val, cfg.dft_dof_pos))
                std::cerr << "[SDK2] parse dft_dof_pos failed, expect 12 floats\n";
        } else if (key == "joint_idx_sdk2policy") {
            std::cout << "[SDK2]Parseing joint_idx_sdk2policy: " << std::endl;
            if (!parse_list<int, 12>(val, cfg.joint_idx_sdk2policy))
                std::cerr << "[SDK2] parse joint_idx_sdk2policy failed, expect 12 ints\n";
        }
    }
    // Compute inverse mapping (do not load from file)
    compute_pol2rob_from_sdk2policy(cfg.joint_idx_sdk2policy, cfg.joint_idx_policy2sdk);
    std::cout << "[SDK2] Computed joint_idx_policy2sdk: ";
    for (int i = 0; i < 12; ++i) {
        if (i) std::cout << ", ";
        std::cout << cfg.joint_idx_policy2sdk[i];
    }
    std::cout << std::endl;

    if (ok) *ok = good;
    return cfg;
}

SDK2RobotControl::SDK2RobotControl(const std::string &network_interface, double timeout_s, bool auto_stand, const std::string& config_path)
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
    obs_cfg_ = SDK2PolicyConfig::FromFile(config_path, &ok);
    std::cout << "[SDK2] Load config: " << config_path << (ok ? " [OK]" : " [ERR]") << std::endl;

    // Ensure buffers
    ensureObsBuffers();
    std::cout << "[SDK2] Observation buffers prepared: obs_size=" << obs_cfg_.obs_size
              << " history_steps=" << obs_cfg_.history_steps << std::endl;
    
    size_t pos = config_path_.find_last_of('/');
    std::string parent_dir = config_path_.substr(0, pos + 1);
    std::string onnx_path = parent_dir + obs_cfg_.onnx_model_path;
    std::cout << "[SDK2] Load onnx_path: " << onnx_path << std::endl;
    // Load ONNX in ctor
    try {
        ort_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "him_policy");
        Ort::SessionOptions opt;
        opt.SetIntraOpNumThreads(1);
        opt.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_BASIC);

        ort_session_ = std::make_unique<Ort::Session>(*ort_env_, onnx_path.c_str(), opt);

        // IO names
        Ort::AllocatorWithDefaultOptions allocator;
        const size_t in_count = ort_session_->GetInputCount();
        const size_t out_count = ort_session_->GetOutputCount();
        if (in_count == 0 || out_count == 0) {
            std::cerr << "[SDK2] ONNX: invalid IO count (in=" << in_count
                      << ", out=" << out_count << ")" << std::endl;
            ort_session_.reset();
            onnx_ready_ = false;
        } else {
            {
                auto name = ort_session_->GetInputNameAllocated(0, allocator);
                ort_input_names_str_.emplace_back(name.get());
            }
            {
                auto name = ort_session_->GetOutputNameAllocated(0, allocator);
                ort_output_names_str_.emplace_back(name.get());
            }
            ort_input_names_  = { ort_input_names_str_[0].c_str() };
            ort_output_names_ = { ort_output_names_str_[0].c_str() };
            onnx_ready_ = true;
            std::cout << "[SDK2] ONNX loaded: " << obs_cfg_.onnx_model_path
                      << " input=" << ort_input_names_str_[0]
                      << " output=" << ort_output_names_str_[0] << std::endl;
        }
    } catch (const Ort::Exception& e) {
        std::cerr << "[SDK2] ONNX load error: " << e.what()
                  << " path=" << obs_cfg_.onnx_model_path << std::endl;
        ort_session_.reset();
        onnx_ready_ = false;
    }

    // Start control loop thread
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

void SDK2RobotControl::controlLoop()
{
    SDK2RobotObsResult res = getRobotObs();

    const int frame = obs_cfg_.obs_size;
    const std::size_t total = static_cast<std::size_t>(frame) * std::max<std::size_t>(obs_cfg_.history_steps, std::size_t(1));
    if (res.his_obs.size() < total) {
        std::cerr << "[SDK2] ONNX infer: his_obs size too small: " << res.his_obs.size()
                  << " < " << total << std::endl;
        return;
    }
    if (!onnx_ready_ || !ort_session_) {
        std::cerr << "[SDK2] ONNX infer: session not ready." << std::endl;
        return;
    }

    // 使用整段历史作为输入
    std::vector<float> obs(res.his_obs.begin(), res.his_obs.begin() + total);

    try {
        std::array<int64_t, 2> input_shape{1, static_cast<int64_t>(total)};
        Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(mem, obs.data(),
                                                                  obs.size(), input_shape.data(), input_shape.size());

        auto outputs = ort_session_->Run(Ort::RunOptions{nullptr},
                                         ort_input_names_.data(), &input_tensor, 1,
                                         ort_output_names_.data(), 1);

        if (outputs.empty() || !outputs[0].IsTensor()) {
            std::cerr << "[SDK2] ONNX infer: empty or non-tensor output" << std::endl;
            return;
        }

        float* out = outputs[0].GetTensorMutableData<float>();

        // 打印 1x12 输出
        static uint64_t policy_cnt = 0;
        if ((++policy_cnt % 50) == 0) {
            std::cout << "[SDK2] ONNX infer out: ";
            for (int i = 0; i < 12; ++i) {
                if (i) std::cout << ", ";
                std::cout << out[i];
            }
            std::cout << std::endl;
        }

        // 保存原始策略动作（policy 顺序）到 last_act_
        for (int i = 0; i < 12; ++i) {
            // todo: add act clip
            last_act_[i] = out[i];
        }

        // 将策略顺序的动作映射到机器人关节顺序
        std::array<double, 12> joint_positions{};
        for (int r = 0; r < 12; ++r) {
            int p = obs_cfg_.joint_idx_policy2sdk[r];
            // todo: add hip_reduction config
            joint_positions[r] = static_cast<double>(out[p]) * obs_cfg_.act_scale + obs_cfg_.dft_dof_pos[r];
        }

        applyPositionControl(joint_positions);
    } catch (const Ort::Exception& e) {
        std::cerr << "[SDK2] ONNX runtime error: " << e.what() << std::endl;
        return;
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
    for (int i = 0; i < 12; ++i) {
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

// ---- methods for observations ----
void SDK2RobotControl::setObsConfig(const SDK2PolicyConfig& cfg)
{
    obs_cfg_ = cfg;
    compute_pol2rob_from_sdk2policy(obs_cfg_.joint_idx_sdk2policy, obs_cfg_.joint_idx_policy2sdk);
    ensureObsBuffers();
}

SDK2RobotObsResult SDK2RobotControl::getRobotObs()
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

    SDK2RobotObsResult res;
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

std::array<float, 3> SDK2RobotControl::gravFromQuatWxyz(const std::array<float, 4>& q)
{
    // Quaternion (w, x, y, z), normalized
    const float w = q[0], x = -q[1], y = -q[2], z = -q[3];

    // Rotation matrix (body->world). Gravity in world is [0,0,-1].
    // g_body = R^T * [0,0,-1] = - third column of R
    const float c0 = -2.0f * (x * z + w * y);
    const float c1 = -2.0f * (y * z - w * x);
    const float c2 = -(w * w - x * x - y * y + z * z);

    // Negative of third column
    return std::array<float, 3>{c0, c1, c2};
}

void SDK2RobotControl::ensureObsBuffers()
{
    // const int obs_size = 3 + 3 + 3 + 12 + 12 + 12;
    // obs_cfg_.obs_size = obs_size;
    const std::size_t total = static_cast<std::size_t>(obs_cfg_.obs_size) * std::max<std::size_t>(obs_cfg_.history_steps, 1);
    his_obs_.assign(total, 0.0f);
    std::cout << "[SDK2] ensureObsBuffers: obs_size=" << total
              << " history_steps=" << obs_cfg_.history_steps
              << " total=" << total << std::endl;
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
    static uint64_t ls_cnt = 0;
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