#include "base_robot_control.h"
#include <onnxruntime_cxx_api.h>
#include <provider_options.h>

// 通用：把 ProviderOptions 映射到 C API 所需的 keys/values 并附加 EP
namespace {
inline void AppendEP(Ort::SessionOptions& so, const char* provider_name, const onnxruntime::ProviderOptions& opts) {
    if (std::string(provider_name) == "CUDAExecutionProvider") {
        OrtCUDAProviderOptions cuda_opts{};
        for (const auto& kv : opts) {
            if (kv.first == "device_id") {
                cuda_opts.device_id = std::stoi(kv.second);
            }
            // Add other CUDA-specific options here if needed
        }
        Ort::ThrowOnError(Ort::GetApi().SessionOptionsAppendExecutionProvider_CUDA(so, &cuda_opts));
    } else if (std::string(provider_name) == "TensorrtExecutionProvider") {
        OrtTensorRTProviderOptions trt_opts{};
        for (const auto& kv : opts) {
            if (kv.first == "device_id") {
                trt_opts.device_id = std::stoi(kv.second);
            }
            // Add other TensorRT-specific options here if needed
        }
        Ort::ThrowOnError(Ort::GetApi().SessionOptionsAppendExecutionProvider_TensorRT(so, &trt_opts));
    } else {
        std::cerr << "[AppendEP] Unsupported provider: " << provider_name << std::endl;
    }
}
} // namespace

BaseRobotControl::PolicyConfig BaseRobotControl::PolicyConfig::FromFile(const std::string& path, bool* ok) 
{
    PolicyConfig cfg;
    std::ifstream fin(path, std::ios::in);
    if (!fin) {
        if (ok) *ok = false;
        std::cerr << "[BaseRobotControl] Config open failed: " << path << std::endl;
        return cfg;
    }
    std::string line;

    auto parse_vec = [](const std::string& s, auto& vec) {
        using T = typename std::decay_t<decltype(vec)>::value_type;
        std::stringstream ss(s);
        std::string item;
        vec.clear();
        while (std::getline(ss, item, ',')) {
            std::stringstream converter(trim(item));
            T value;
            converter >> value;
            vec.push_back(value);
        }
    };

    while (std::getline(fin, line)) {
        auto p = line.find_first_of("#;");
        if (p != std::string::npos) line = line.substr(0, p);
        p = line.find("//");
        if (p != std::string::npos) line = line.substr(0, p);
        line = trim(line);
        if (line.empty()) continue;

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
        else if (key == "act_size") cfg.act_size = std::stoi(val);
        else if (key == "kp") cfg.kp = std::stof(val);
        else if (key == "kd") cfg.kd = std::stof(val);
        else if (key == "dft_dof_pos") parse_vec(val, cfg.dft_dof_pos);
        else if (key == "joint_idx_sdk2policy") parse_vec(val, cfg.joint_idx_sdk2policy);
    }

    if (cfg.dft_dof_pos.size() != static_cast<size_t>(cfg.act_size) ||
        cfg.joint_idx_sdk2policy.size() != static_cast<size_t>(cfg.act_size)) {
        std::cerr << "[BaseRobotControl] Config Error: act_size (" << cfg.act_size 
                  << ") does not match size of dft_dof_pos (" << cfg.dft_dof_pos.size() 
                  << ") or joint_idx_sdk2policy (" << cfg.joint_idx_sdk2policy.size() << ")!" << std::endl;
        if (ok) *ok = false;
        return cfg;
    }

    compute_pol2rob_from_sdk2policy(cfg.joint_idx_sdk2policy, cfg.joint_idx_policy2sdk);

    if (ok) *ok = true;
    return cfg;
}

void BaseRobotControl::controlLoop()
{
    // Check for joystick commands to reset or toggle policy
    if (gamepad_.L1.on_press) {
        resetJointPosition();
        policy_running_ = false; // Stop policy on reset
        std::cout << "[USER] L1 pressed, reset robot position." << std::endl;
        gamepad_.L1.on_press = false;
    }
    if (gamepad_.L2.on_press) {
        policy_running_ = !policy_running_;
        std::cout << "[USER] L2 pressed, policy is now " << (policy_running_ ? "RUNNING" : "STOPPED") << std::endl;
        gamepad_.L2.on_press = false;
    }

    // Exit if policy is not running
    if (!policy_running_) {
        return;
    }

    RobotObsResult res = getRobotObs();

    const int frame = obs_cfg_.obs_size;
    const std::size_t total = static_cast<std::size_t>(frame) * std::max<std::size_t>(obs_cfg_.history_steps, std::size_t(1));

    // 使用整段历史作为输入
    std::vector<float> obs(res.his_obs.begin(), res.his_obs.begin() + total);
    std::vector<float> outputs;

    const int act_size = obs_cfg_.act_size;
    if (runOnnxInference(obs, outputs)) {
        for (int i = 0; i < act_size; ++i) {
            last_act_[i] = outputs[i];
        }
        // 打印动作
        static uint64_t policy_cnt = 0;
        if ((++policy_cnt % 50) == 0) {
            std::cout << "ONNX infer out (" << act_size << " actions): ";
            for (int i = 0; i < act_size; ++i) {
                std::cout << outputs[i] * obs_cfg_.act_scale + obs_cfg_.dft_dof_pos[i] << ", ";
            }
            std::cout << std::endl;
        }
        // 将策略顺序的动作映射到机器人关节顺序
        std::vector<float> joint_positions(act_size);
        for (std::size_t r = 0; r < act_size; ++r) {
            int p = obs_cfg_.joint_idx_policy2sdk[r];
            joint_positions[r] = outputs[p] * obs_cfg_.act_scale + obs_cfg_.dft_dof_pos[r];
        }
        applyPositionControl(joint_positions);
    }
}

void BaseRobotControl::setObsConfig(const PolicyConfig& cfg) 
{
    obs_cfg_ = cfg;

    compute_pol2rob_from_sdk2policy(obs_cfg_.joint_idx_sdk2policy, obs_cfg_.joint_idx_policy2sdk);

    ensureObsBuffers();
}

void BaseRobotControl::loadOnnxModel(const std::string& onnx_path)
{
    // 兼容旧接口：默认 CPU
    loadOnnxModel(onnx_path, OnnxOptions{});
}

void BaseRobotControl::loadOnnxModel(const std::string& onnx_path, const OnnxOptions& opt) 
{
    try {
        ort_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "robot_policy");
        Ort::SessionOptions so;
        so.SetIntraOpNumThreads(1);
        so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

        bool using_gpu = false;
        std::string ep_name = "CPU";
        try {
#ifdef ORT_WITH_TENSORRT
            std::cout << "[BaseRobotControl] Registering TensorRT EP..." << std::endl;
            onnxruntime::ProviderOptions trt_opts;
            trt_opts["device_id"] = std::to_string(opt.device_id);
            // 可选：trt_opts["trt_fp16_enable"] = "1";
            // 可选：trt_opts["trt_engine_cache_enable"] = "1";
            AppendEP(so, "TensorrtExecutionProvider", trt_opts);
            ep_name = "TensorRT";
            using_gpu = true;
#elif defined(ORT_WITH_CUDA)
            std::cout << "[BaseRobotControl] Registering CUDA EP..." << std::endl;
            onnxruntime::ProviderOptions cuda_opts;
            cuda_opts["device_id"] = std::to_string(opt.device_id);
            // 可选：cuda_opts["do_copy_in_default_stream"] = "1";
            AppendEP(so, "CUDAExecutionProvider", cuda_opts);
            ep_name = "CUDA";
            using_gpu = true;
#endif
        } catch (const Ort::Exception& e) {
            std::cerr << "[BaseRobotControl] EP registration failed, fallback to CPU. err=" << e.what() << std::endl;
            using_gpu = false;
            ep_name = "CPU";
        }

        ort_session_ = std::make_unique<Ort::Session>(*ort_env_, onnx_path.c_str(), so);

        Ort::AllocatorWithDefaultOptions allocator;
        ort_input_names_str_.clear();
        ort_output_names_str_.clear();
        ort_input_names_str_.emplace_back(ort_session_->GetInputNameAllocated(0, allocator).get());
        ort_output_names_str_.emplace_back(ort_session_->GetOutputNameAllocated(0, allocator).get());
        ort_input_names_  = { ort_input_names_str_[0].c_str() };
        ort_output_names_ = { ort_output_names_str_[0].c_str() };
        onnx_ready_ = true;

        std::cout << "ONNX loaded: " << onnx_path
                  << " provider=" << ep_name
                  << " input=" << ort_input_names_str_[0]
                  << " output=" << ort_output_names_str_[0] << std::endl;
    } catch (const Ort::Exception& e) {
        std::cerr << "ONNX load error: " << e.what() << " path=" << onnx_path << std::endl;
        ort_session_.reset();
        onnx_ready_ = false;
    }
}

bool BaseRobotControl::runOnnxInference(const std::vector<float>& input, std::vector<float>& output) 
{
    if (!onnx_ready_ || !ort_session_) {
        std::cerr << "[BaseRobotControl] ONNX session not ready." << std::endl;
        return false;
    }

    try {
        const std::array<int64_t, 2> input_shape{1, static_cast<int64_t>(input.size())};
        // 即使使用 GPU EP，传 CPU 内存也是支持的，ORT 内部会拷贝到 GPU
        Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            mem, const_cast<float*>(input.data()), input.size(),
            input_shape.data(), input_shape.size());

        auto outputs = ort_session_->Run(Ort::RunOptions{nullptr},
                                         ort_input_names_.data(), &input_tensor, 1,
                                         ort_output_names_.data(), 1);

        if (outputs.empty() || !outputs[0].IsTensor()) {
            std::cerr << "[BaseRobotControl] ONNX inference failed: empty or non-tensor output." << std::endl;
            return false;
        }

        float* out = outputs[0].GetTensorMutableData<float>();
        output.assign(out, out + outputs[0].GetTensorTypeAndShapeInfo().GetElementCount());
        return true;
    } catch (const Ort::Exception& e) {
        std::cerr << "[BaseRobotControl] ONNX runtime error: " << e.what() << std::endl;
        return false;
    }
}

std::array<float, 3> BaseRobotControl::gravFromQuatWxyz(const std::array<float, 4>& q)
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

void BaseRobotControl::ensureObsBuffers()
{
    last_act_.assign(obs_cfg_.act_size, 0.0f);
    const std::size_t total = static_cast<std::size_t>(obs_cfg_.obs_size) * std::max<std::size_t>(obs_cfg_.history_steps, 1);
    his_obs_.assign(total, 0.0f);
    std::cout << "[SDK1] ensureObsBuffers: obs_size=" << total
              << " history_steps=" << obs_cfg_.history_steps
              << " total=" << total << std::endl;
}
