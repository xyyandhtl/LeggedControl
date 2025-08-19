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
#include <onnxruntime_cxx_api.h>
#include "../common/utils.h"

SDK1PolicyConfig SDK1PolicyConfig::FromFile(const std::string& path, bool* ok) {
    SDK1PolicyConfig cfg;
    std::ifstream fin(path);
    bool good = fin.good();
    if (!fin) {
        if (ok) *ok = false;
        std::cerr << "[SDK1] Config open failed: " << path << std::endl;
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
            std::cout << "[SDK1]Parseing dft_dof_pos: " << std::endl;
            if (!parse_list<float, 12>(val, cfg.dft_dof_pos))
                std::cerr << "[SDK1] parse dft_dof_pos failed, expect 12 floats\n";
        } else if (key == "joint_idx_rob2pol") {
            std::cout << "[SDK1]Parseing joint_idx_rob2pol: " << std::endl;
            if (!parse_list<int, 12>(val, cfg.joint_idx_rob2pol))
                std::cerr << "[SDK1] parse joint_idx_rob2pol failed, expect 12 ints\n";
        }
    }
    // Compute inverse mapping (do not load from file)
    compute_pol2rob_from_rob2pol(cfg.joint_idx_rob2pol, cfg.joint_idx_pol2rgb);
    std::cout << "[SDK1] Computed joint_idx_pol2rgb: ";
    for (int i = 0; i < 12; ++i) {
        if (i) std::cout << ", ";
        std::cout << cfg.joint_idx_pol2rgb[i];
    }
    std::cout << std::endl;

    if (ok) *ok = good;
    return cfg;
}

SDK1RobotControl::SDK1RobotControl(uint16_t local_port, const std::string &target_ip, uint16_t target_port, const std::string& config_path)
    : udp_(local_port, target_ip.c_str(), target_port, LOW_CMD_LENGTH, LOW_STATE_LENGTH),
      safe_(LeggedType::Aliengo)
{
    std::cout << "[SDK2] local_port=" << local_port
              << " target_ip=" << target_ip
              << " target_port=" << target_port << std::endl;

    udp_.InitCmdData(cmd_);
    cmd_.levelFlag = LOWLEVEL;

    // Load config
    config_path_ = config_path;
    bool ok = false;
    obs_cfg_ = SDK1PolicyConfig::FromFile(config_path, &ok);
    std::cout << "[SDK1] Load config: " << config_path << (ok ? " [OK]" : " [ERR]") << std::endl;

    // Ensure buffers
    ensureObsBuffers();
    std::cout << "[SDK2] Observation buffers prepared: obs_size=" << obs_cfg_.obs_size
              << " history_steps=" << obs_cfg_.history_steps << std::endl;

    size_t pos = config_path_.find_last_of('/');
    std::string parent_dir = config_path_.substr(0, pos + 1);
    std::string onnx_path = parent_dir + obs_cfg_.onnx_model_path;
    std::cout << "[SDK1] Load onnx_path: " << onnx_path << std::endl;
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
            std::cerr << "[SDK1] ONNX: invalid IO count (in=" << in_count
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
            std::cout << "[SDK1] ONNX loaded: " << obs_cfg_.onnx_model_path
                      << " input=" << ort_input_names_str_[0]
                      << " output=" << ort_output_names_str_[0] << std::endl;
        }
    } catch (const Ort::Exception& e) {
        std::cerr << "[SDK1] ONNX load error: " << e.what()
                  << " path=" << obs_cfg_.onnx_model_path << std::endl;
        ort_session_.reset();
        onnx_ready_ = false;
    }

    loop_udpSend = std::make_unique<LoopFunc>(
        "udp_send", 0.002, 3, boost::bind(&SDK1RobotControl::udpSend, this));
    loop_udpRecv = std::make_unique<LoopFunc>(
        "udp_recv", 0.002, 3, boost::bind(&SDK1RobotControl::udpRecv, this));
    loop_udpSend->start();
    loop_udpRecv->start();
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
    std::cout << "[SDK1] dtor: stopping..." << std::endl;
}

void SDK1RobotControl::applyPositionControl(const std::array<double, 12> &joint_positions)
{
    // 下发时再乘以 act_scale
    for (int i = 0; i < 12; i++) {
        cmd_.motorCmd[i].q  = joint_positions[i];
        cmd_.motorCmd[i].dq = 0.0;
        cmd_.motorCmd[i].Kp = obs_cfg_.kp;
        cmd_.motorCmd[i].Kd = obs_cfg_.kd;
        cmd_.motorCmd[i].tau = 0.0f;
    }

    // Gravity compensation
    cmd_.motorCmd[FR_0].tau = -1.6f;
    cmd_.motorCmd[FL_0].tau = -1.6f;
    cmd_.motorCmd[RR_0].tau = -1.6f;
    cmd_.motorCmd[RL_0].tau = -1.6f;

    udp_.SetSend(cmd_);
}

void SDK1RobotControl::applyVelCmdControl(double vx, double vy, double wz)
{
    // Store last velocity command for observations
    last_cmd_[0] = static_cast<float>(vx);
    last_cmd_[1] = static_cast<float>(vy);
    last_cmd_[2] = static_cast<float>(wz);

    SDK1RobotObsResult res = getRobotObs();

    const int frame = obs_cfg_.obs_size;
    const std::size_t total = static_cast<std::size_t>(frame) * std::max<std::size_t>(obs_cfg_.history_steps, std::size_t(1));
    if (res.his_obs.size() < total) {
        std::cerr << "[SDK1] ONNX infer: his_obs size too small: " << res.his_obs.size()
                  << " < " << total << std::endl;
        return;
    }
    if (!onnx_ready_ || !ort_session_) {
        std::cerr << "[SDK1] ONNX infer: session not ready." << std::endl;
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
            std::cerr << "[SDK1] ONNX infer: empty or non-tensor output" << std::endl;
            return;
        }

        float* out = outputs[0].GetTensorMutableData<float>();

        // 打印 1x12 输出
        static uint64_t policy_cnt = 0;
        if ((++policy_cnt % 50) == 0) {
            std::cout << "[SDK1] ONNX infer out: ";
            for (int i = 0; i < 12; ++i) {
                if (i) std::cout << ", ";
                std::cout << out[i];
            }
            std::cout << std::endl;
        }

        // 保存原始策略动作（policy 顺序）到 last_act_
        for (int i = 0; i < 12; ++i) {
            last_act_[i] = out[i];
        }

        // 将策略顺序的动作映射到机器人关节顺序
        std::array<double, 12> joint_positions{};
        for (int r = 0; r < 12; ++r) {
            int p = obs_cfg_.joint_idx_pol2rgb[r];
            joint_positions[r] = static_cast<double>(out[p]);
        }

        applyPositionControl(joint_positions);
    } catch (const Ort::Exception& e) {
        std::cerr << "[SDK1] ONNX runtime error: " << e.what() << std::endl;
        return;
    }
}

// ---- methods for observations ----
void SDK1RobotControl::setObsConfig(const SDK1PolicyConfig& cfg)
{
    obs_cfg_ = cfg;
    compute_pol2rob_from_rob2pol(obs_cfg_.joint_idx_rob2pol, obs_cfg_.joint_idx_pol2rgb);
    ensureObsBuffers();
}

SDK1RobotObsResult SDK1RobotControl::getRobotObs()
{
    // 1) cmd (3)
    std::array<float, 3> cmd_scaled = last_cmd_;
    cmd_scaled[0] *= obs_cfg_.vel_scale;
    cmd_scaled[1] *= obs_cfg_.vel_scale;
    cmd_scaled[2] *= obs_cfg_.gyr_scale;

    // 2) IMU gyr (3) scaled and clipped
    std::array<float, 3> gyr = state_.imu.gyroscope;
    for (int i = 0; i < 3; ++i) {
        gyr[i] *= obs_cfg_.gyr_scale;
    }
    gyr[2] = clip(gyr[2], -0.12f, 0.12f); // temporary IMU clipping

    // 3) grav (3) from quaternion (w,x,y,z)
    std::array<float, 3> grav = gravFromQuatWxyz(state_.imu.quaternion);

    // 4) dof_pos (12)
    std::array<float, 12> dof_pos_robot{};
    std::array<float, 12> dof_vel_robot{};
    for (int i = 0; i < 12; ++i) {
        dof_pos_robot[i] = static_cast<float>(state_.motorState[i].q);
        dof_vel_robot[i] = static_cast<float>(state_.motorState[i].dq);
    }
    // Subtract default offsets
    for (int i = 0; i < 12; ++i) {
        dof_pos_robot[i] -= obs_cfg_.dft_dof_pos[i];
    }
    // Reorder robot->policy using joint_idx_rob2pol
    std::array<float, 12> dof_pos{};
    std::array<float, 12> dof_vel{};
    for (int pi = 0; pi < 12; ++pi) {
        const int ri = obs_cfg_.joint_idx_rob2pol[pi];
        // const int ri_clamped = std::clamp(ri, 0, 11);
        dof_pos[pi] = dof_pos_robot[ri] * obs_cfg_.dof_pos_scale;
        dof_vel[pi] = dof_vel_robot[ri] * obs_cfg_.dof_vel_scale;
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

    SDK1RobotObsResult res;
    res.his_obs = his_obs_; // return a copy
    
    static uint64_t obs_cnt = 0;
    if ((++obs_cnt % 50) == 0) {
        std::copy(obs.begin(), obs.end(), std::ostream_iterator<float>(std::cout, " "));
        std::cout << std::endl;
    }
    return res;
}

void SDK1RobotControl::udpRecv()
{
    udp_.Recv();
    udp_.GetRecv(state_);
    std::memcpy(&key_data_, &state_.wirelessRemote, sizeof(xRockerBtnDataStruct)); // Update joystick data
}

void SDK1RobotControl::udpSend()
{
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
    udp_.SetSend(cmd_);
}

xRockerBtnDataStruct SDK1RobotControl::getJoystickData() const
{
    return key_data_;
}

std::array<float, 3> SDK1RobotControl::gravFromQuatWxyz(const std::array<float, 4>& q)
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

void SDK1RobotControl::ensureObsBuffers()
{
    // const int obs_size = 3 + 3 + 3 + 12 + 12 + 12;
    // obs_cfg_.obs_size = obs_size;
    const std::size_t total = static_cast<std::size_t>(obs_cfg_.obs_size) * std::max<std::size_t>(obs_cfg_.history_steps, 1);
    his_obs_.assign(total, 0.0f);
    std::cout << "[SDK2] ensureObsBuffers: obs_size=" << total
              << " history_steps=" << obs_cfg_.history_steps
              << " total=" << total << std::endl;
}