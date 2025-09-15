#include "sdk2_sport_control.h"
#include <unitree/robot/channel/channel_factory.hpp>
#include <iostream>

SDK2SportControl::SDK2SportControl(const rclcpp::NodeOptions & options)
    : rclcpp::Node("go2_sport_node", options),
      last_ros_cmd_time_(this->now())  // Set initial ROS command time to avoid immediate timeout
{
    RCLCPP_INFO(get_logger(), "Starting Go2/W Sport Control Node...");

    // Declare and get parameters
    std::string network_interface = this->declare_parameter<std::string>("network_interface", "eth0");
    bool auto_stand = this->declare_parameter<bool>("auto_stand", true);
    float timeout_s = this->declare_parameter<float>("timeout_s", 10.0);
    int control_rate_hz = this->declare_parameter<int>("control_rate_hz", 50);
    max_vx_ = this->declare_parameter<float>("max_vx", 2.0);
    max_vy_ = this->declare_parameter<float>("max_vy", 0.5);
    max_wz_ = this->declare_parameter<float>("max_wz", 1.5);
    stale_timeout_s_ = this->declare_parameter<float>("stale_timeout_s", 1.0);

    std::cout << "[SDK2 | SDK2SportControl] Initializing..." << std::endl;
    std::cout << "ctor iface = " << network_interface
              << ", auto_stand = " << (auto_stand ? "true" : "false") << std::endl;

    // Initialize DDS
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface.c_str());

    // Initialize SportClient
    sport_client_ = std::make_unique<unitree::robot::go2::SportClient>();
    sport_client_->SetTimeout(timeout_s);
    sport_client_->Init();
    RCLCPP_INFO(get_logger(), "SportClient initialized.");

    // Auto stand up if requested
    if (auto_stand) {
        RCLCPP_INFO(get_logger(), "Performing auto stand-up...");
        int ret = sport_client_->StandUp();
        std::cout << "[SDK2] Auto StandUp return code: " << ret << std::endl;
    }

    // Initialize ROS subscription (velocity commands)
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::QoS(10),
        std::bind(&SDK2SportControl::onTwist, this, std::placeholders::_1));

    // Initialize Joystick subscriber
    joystick_sub_.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>("rt/wirelesscontroller"));
    joystick_sub_->InitChannel(std::bind(&SDK2SportControl::onJoystickMessage, this, std::placeholders::_1), 1);
    RCLCPP_INFO(get_logger(), "Joystick subscriber initialized.");

    // Initialize SportState subscriber
    sport_state_sub_.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>("rt/sportstate"));
    sport_state_sub_->InitChannel(std::bind(&SDK2SportControl::onSportStateMessage, this, std::placeholders::_1), 1);
    RCLCPP_INFO(get_logger(), "SportState subscriber initialized.");

    // Initialize control loop timer
    using namespace std::chrono_literals;
    auto period = std::chrono::microseconds(static_cast<int64_t>(1'000'000 / std::max(1, control_rate_hz)));
    control_timer_ = this->create_wall_timer(period, std::bind(&SDK2SportControl::controlLoop, this));

    RCLCPP_INFO(get_logger(), "Go2/W Sport Control Node started successfully.");
}

SDK2SportControl::~SDK2SportControl() {
    RCLCPP_INFO(get_logger(), "Shutting down Go2/W Sport Control Node.");
    if (sport_client_) {
        sport_client_->StopMove();
    }
}

void SDK2SportControl::onTwist(const geometry_msgs::msg::Twist::SharedPtr msg) {
    ros_vel_cmd_[0] = std::max(-max_vx_, std::min(static_cast<float>(msg->linear.x), max_vx_));
    ros_vel_cmd_[1] = std::max(-max_vy_, std::min(static_cast<float>(msg->linear.y), max_vy_));
    ros_vel_cmd_[2] = std::max(-max_wz_, std::min(static_cast<float>(msg->angular.z), max_wz_));
    last_ros_cmd_time_ = this->now();
    sent_stop_ = false;
}

void SDK2SportControl::onJoystickMessage(const void *msg) {
    auto key_data = *reinterpret_cast<const unitree_go::msg::dds_::WirelessController_ *>(msg);
    gamepad_.Update(key_data);

    if (gamepad_.L2.on_press) {
        gamepad_.L2.on_press = false; // Consume event
        RCLCPP_INFO(get_logger(), "L2 Pressed, calling BalanceStand() to enter dynamic standing mode.");
        sport_client_->BalanceStand();
    }
}

void SDK2SportControl::controlLoop() {
    float vx = 0.0f, vy = 0.0f, wz = 0.0f;
    bool is_joystick_active = gamepad_.ly != 0.0f || gamepad_.lx != 0.0f || gamepad_.rx != 0.0f;

    if (is_joystick_active) {
        // Priority 1: Physical joystick is being used
        vx = gamepad_.ly * max_vx_;
        vy = gamepad_.lx * max_vy_;
        wz = gamepad_.rx * max_wz_;
        sent_stop_ = false;
    } else {
        if ((this->now() - last_ros_cmd_time_).seconds() <= stale_timeout_s_) {
            // Priority 2: ROS /cmd_vel is active
            vx = ros_vel_cmd_[0];
            vy = ros_vel_cmd_[1];
            wz = ros_vel_cmd_[2];
        } else {
            // Priority 3: No joystick input and ROS command timed out
            if (!sent_stop_) {
                sport_client_->StopMove();
                sent_stop_ = true;
            }
        }
    }

    if (is_joystick_active || !sent_stop_) {
        RCLCPP_DEBUG(get_logger(), "Executing Move(vx: %.2f, vy: %.2f, wz: %.2f)", vx, vy, wz);
        int ret = sport_client_->Move(vx, vy, wz);
        RCLCPP_DEBUG(get_logger(), "\tMove ret: %d", ret);
    }

    // Print current state periodically
    unitree_go::msg::dds_::SportModeState_ state_copy;
    {
        std::lock_guard<std::mutex> lock(sport_state_mutex_);
        state_copy = current_sport_state_;
    }
    RCLCPP_DEBUG(get_logger(),
        "State: mode=%d, vx=%.3f, vy=%.3f, yaw_speed=%.3f",
        state_copy.mode(),
        state_copy.velocity()[0],
        state_copy.velocity()[1],
        state_copy.yaw_speed());
}

void SDK2SportControl::onSportStateMessage(const void* msg) {
    std::lock_guard<std::mutex> lock(sport_state_mutex_);
    current_sport_state_ = *reinterpret_cast<const unitree_go::msg::dds_::SportModeState_*>(msg);
}
