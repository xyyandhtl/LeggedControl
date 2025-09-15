#include "sdk2_policy_control.h"
#include "utils.h"
#include <unitree/robot/channel/channel_factory.hpp>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <thread>

// Constants from examples
static constexpr float PosStopF = (2.146E+9f);
static constexpr float VelStopF = (16000.0f);

SDK2PolicyControl::SDK2PolicyControl(const rclcpp::NodeOptions & options)
    : rclcpp::Node("go2_policy_node", options),
      last_ros_cmd_time_(this->now())
{
    RCLCPP_INFO(get_logger(), "Starting Go2/W Policy Control Node...");

    // Declare and get parameters
    std::string network_interface = this->declare_parameter<std::string>("network_interface", "eth0");
    std::string config_path = this->declare_parameter<std::string>("config_path", "");
    float timeout_s = this->declare_parameter<float>("timeout_s", 3.0);
    max_vx_ = this->declare_parameter<float>("max_vx", 2.0);
    max_vy_ = this->declare_parameter<float>("max_vy", 0.5);
    max_wz_ = this->declare_parameter<float>("max_wz", 1.5);
    stale_timeout_s_ = this->declare_parameter<float>("stale_timeout_s", 1.0);

    std::cout << "[SDK2 | SDK2SportControl] Initializing..." << std::endl;
    std::cout << "ctor iface = " << network_interface
              << ", config_path = " << config_path << std::endl;

    // Initialize DDS
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface.c_str());

    // Load ONNX model and policy config
    config_path_ = config_path;
    bool ok = false;
    obs_cfg_ = PolicyConfig::FromFile(config_path, &ok);
    RCLCPP_INFO(get_logger(), "Load config: %s %s", config_path.c_str(), (ok ? "[OK]" : "[ERR]"));
    //  ensure buffers for observations
    ensureObsBuffers();
    RCLCPP_INFO(get_logger(), "Observation buffers prepared: obs_size=%zu, history_steps=%zu", static_cast<size_t>(obs_cfg_.obs_size), static_cast<size_t>(obs_cfg_.history_steps));
    //  load ONNX model
    size_t pos = config_path_.find_last_of('/');
    std::string parent_dir = config_path_.substr(0, pos + 1);
    std::string onnx_path = parent_dir + obs_cfg_.onnx_model_path;
    RCLCPP_INFO(get_logger(), "Load onnx_path: %s", onnx_path.c_str());
    loadOnnxModel(onnx_path);

    // Initialize LowLevel DDS channels
    lowcmd_pub_.reset(new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_pub_->InitChannel();
    std::cout << "[SDK2 | PolicyControl] LowCmd publisher initialized: " << TOPIC_LOWCMD << std::endl;
    lowstate_sub_.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_sub_->InitChannel(std::bind(&SDK2PolicyControl::onLowStateMessage, this, std::placeholders::_1), 1);
    std::cout << "[SDK2 | PolicyControl] LowState subscriber initialized: " << TOPIC_LOWSTATE << std::endl;
    initLowCmd();  // Initialize LowCmd message structure
    RCLCPP_INFO(get_logger(), "LowLevel DDS channels initialized.");

    // Initialize MotionSwitcherClient for releasing sport mode
    msc_ = std::make_unique<unitree::robot::b2::MotionSwitcherClient>();
    msc_->SetTimeout(timeout_s);
    msc_->Init();
    RCLCPP_INFO(get_logger(), "MotionSwitcherClient initialized.");

    // Initialize ROS subscription
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::QoS(10),
        std::bind(&SDK2PolicyControl::onTwist, this, std::placeholders::_1));

    // Initialize Joystick subscriber
    joystick_sub_.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>("rt/wirelesscontroller"));
    joystick_sub_->InitChannel(std::bind(&SDK2PolicyControl::onJoystickMessage, this, std::placeholders::_1), 1);
    RCLCPP_INFO(get_logger(), "Joystick subscriber initialized.");

    RCLCPP_INFO(get_logger(), "Go2/W Policy Control Node started. Press L2 to activate policy.");
}

SDK2PolicyControl::~SDK2PolicyControl() {
    RCLCPP_INFO(get_logger(), "Shutting down Go2/W Policy Control Node.");
    stopControlLoop();
}

void SDK2PolicyControl::onTwist(const geometry_msgs::msg::Twist::SharedPtr msg) {
    ros_vel_cmd_[0] = std::max(-max_vx_, std::min(static_cast<float>(msg->linear.x), max_vx_));
    ros_vel_cmd_[1] = std::max(-max_vy_, std::min(static_cast<float>(msg->linear.y), max_vy_));
    ros_vel_cmd_[2] = std::max(-max_wz_, std::min(static_cast<float>(msg->angular.z), max_wz_));
    last_ros_cmd_time_ = this->now();
}

void SDK2PolicyControl::onJoystickMessage(const void *msg) {
    auto key_data = *reinterpret_cast<const unitree_go::msg::dds_::WirelessController_ *>(msg);
    gamepad_.Update(key_data);

    if (gamepad_.L1.on_press) {
        gamepad_.L1.on_press = false;
        RCLCPP_INFO(get_logger(), "[USER] L1 pressed. SAFE EXIT and RESET initiated.");
        setPolicyRunning(false);
        stopControlLoop();
        std::thread([this]() { this->resetJointPosition(); }).detach();
    }

    if (gamepad_.L2.on_press) {
        gamepad_.L2.on_press = false;
        if (!isControlLoopRunning()) {
            RCLCPP_INFO(get_logger(), "[USER] L2 pressed. Activating Policy Mode (Paused).");
            activatePolicyMode();
        } else {
            togglePolicyRunning();
        }
    }
}

void SDK2PolicyControl::activatePolicyMode() {
    if (isControlLoopRunning()) return;

    // Release sport mode first
    int motionStatus = 1;
    while(motionStatus != 0) {
        msc_->ReleaseMode();
        RCLCPP_INFO(get_logger(), "Attempting to release sport mode...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::string robotForm, motionName;
        msc_->CheckMode(robotForm, motionName);
        motionStatus = motionName.empty() ? 0 : 1;
    }
    RCLCPP_INFO(get_logger(), "Sport mode released.");

    startControlLoop();
    setPolicyRunning(false); // Start in paused state
    RCLCPP_INFO(get_logger(), "Policy Mode activated. Press L2 again to run.");
}

void SDK2PolicyControl::controlLoop() {
    if (!isControlLoopRunning()) return;

    float vx = 0.0f, vy = 0.0f, wz = 0.0f;
    bool is_joystick_active = gamepad_.ly != 0.0f || gamepad_.lx != 0.0f || gamepad_.rx != 0.0f;

    if (is_joystick_active) {
        // Priority 1: Physical joystick is being used
        vx = gamepad_.ly * max_vx_;
        vy = gamepad_.lx * max_vy_;
        wz = gamepad_.rx * max_wz_;
    } else {
        if ((this->now() - last_ros_cmd_time_).seconds() <= stale_timeout_s_) {
            // Priority 2: ROS /cmd_vel is active
            vx = ros_vel_cmd_[0];
            vy = ros_vel_cmd_[1];
            wz = ros_vel_cmd_[2];
        }
        // Priority 3: No joystick input and ROS command timed out, velocity remains 0.0f
    }
    applyVelCmdControl(vx, vy, wz);  // update velocity command, for policy observation

    // This is the core policy execution from the base class
    BaseRobotControl::controlLoop();
}

void SDK2PolicyControl::startControlLoop() {
    if (control_thread_) return;
    RCLCPP_INFO(get_logger(), "Starting control loop thread.");
    control_thread_ = unitree::common::CreateRecurrentThreadEx(
        "policy_control_loop", UT_CPU_ID_NONE, 20000, &SDK2PolicyControl::controlLoop, this);
}

void SDK2PolicyControl::stopControlLoop() {
    if (control_thread_) {
        RCLCPP_INFO(get_logger(), "Stopping control loop thread.");
        control_thread_.reset(); // Resetting the unique_ptr will destroy the thread object and stop the thread.
    }
}

void SDK2PolicyControl::setPolicyRunning(bool running) {
    policy_running_ = running;
    RCLCPP_INFO(get_logger(), "Policy is now %s", (running ? "RUNNING" : "STOPPED"));
}

void SDK2PolicyControl::togglePolicyRunning() {
    setPolicyRunning(!policy_running_);
}

void SDK2PolicyControl::resetJointPosition() {
    const int steps = 200;  // 2 seconds at 10ms intervals
    const float interval = 0.01f;  // 10ms
    std::vector<float> current_positions(obs_cfg_.act_size);
    std::vector<float> target_positions(obs_cfg_.act_size);

    // Initialize current and target positions (This needs to be thread-safe with onLowStateMessage)
    low_state_mtx_.lock();
    for (size_t i = 0; i < obs_cfg_.act_size; ++i) {
        current_positions[i] = low_state_.motor_state()[i].q();
        target_positions[i] = obs_cfg_.dft_dof_pos[i];
    }
    low_state_mtx_.unlock();

    for (int step = 0; step <= steps; ++step) {
        std::vector<float> interpolated_positions(obs_cfg_.act_size);
        for (size_t i = 0; i < obs_cfg_.act_size; ++i) {
            interpolated_positions[i] = current_positions[i] +
                (target_positions[i] - current_positions[i]) * (static_cast<float>(step) / steps);
        }
        applyPositionControl(interpolated_positions);
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(interval * 1000)));
    }
    RCLCPP_INFO(get_logger(), "Joint reset complete.");
}

void SDK2PolicyControl::applyPositionControl(std::vector<float>& joint_positions) {
    for (std::size_t i = 0; i < obs_cfg_.act_size; i++) {
        low_cmd_.motor_cmd()[i].mode() = 0x01;
        low_cmd_.motor_cmd()[i].q()    = joint_positions[i];
        low_cmd_.motor_cmd()[i].dq()   = 0.0f;
        low_cmd_.motor_cmd()[i].kp()   = obs_cfg_.kp;
        low_cmd_.motor_cmd()[i].kd()   = obs_cfg_.kd;
        low_cmd_.motor_cmd()[i].tau()  = 0.0f;
    }
    low_cmd_.crc() = crc32_core(reinterpret_cast<uint32_t*>(&low_cmd_), (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_pub_->Write(low_cmd_);
}

void SDK2PolicyControl::applyVelCmdControl(float vx, float vy, float wz) {
    last_cmd_[0] = vx;
    last_cmd_[1] = vy;
    last_cmd_[2] = wz;
}

const BaseRobotControl::RobotObsResult SDK2PolicyControl::getRobotObs() {
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
    for (size_t i = 0; i < obs_cfg_.act_size; ++i) {
        dof_pos_sdk[i] = ms[i].q() - obs_cfg_.dft_dof_pos[i];
        dof_vel_sdk[i] = ms[i].dq();
    }
    // in policy order (by using joint_idx_sdk2policy)
    std::vector<float> dof_pos(obs_cfg_.act_size), dof_vel(obs_cfg_.act_size);
    for (size_t pi = 0; pi < obs_cfg_.act_size; ++pi) {
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
    const size_t frame = obs_cfg_.obs_size;
    if (his_obs_.size() != frame * obs_cfg_.history_steps) {
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
        std::cout << "[SDK2 | PolicyControl] Current robot obs: ";
        std::copy(obs.begin(), obs.end(), std::ostream_iterator<float>(std::cout, " "));
        std::cout << std::endl;
    }
    return res;
}

void SDK2PolicyControl::onLowStateMessage(const void* msg) {
    std::lock_guard<std::mutex> lk(low_state_mtx_);
    low_state_ = *reinterpret_cast<const unitree_go::msg::dds_::LowState_*>(msg);
}

void SDK2PolicyControl::initLowCmd() {
    // 参考官方示例
    low_cmd_.head()[0] = 0xFE;
    low_cmd_.head()[1] = 0xEF;
    low_cmd_.level_flag() = 0xFF;
    low_cmd_.gpio() = 0;

    for (size_t i = 0; i < low_cmd_.motor_cmd().size(); ++i) {
        low_cmd_.motor_cmd()[i].mode() = 0x01;  // PMSM 伺服
        low_cmd_.motor_cmd()[i].q()    = PosStopF;
        low_cmd_.motor_cmd()[i].kp()   = 0;
        low_cmd_.motor_cmd()[i].dq()   = VelStopF;
        low_cmd_.motor_cmd()[i].kd()   = 0;
        low_cmd_.motor_cmd()[i].tau()  = 0;
    }
    std::cout << "[SDK2 | PolicyControl] initLowCmd: motor_cmd.size = " << low_cmd_.motor_cmd().size() << std::endl;
}

uint32_t SDK2PolicyControl::crc32_core(uint32_t* ptr, uint32_t len) {
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