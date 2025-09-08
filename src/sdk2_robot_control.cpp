#include "sdk2_robot_control.h"
#include "utils.h"
#include <iostream>
#include <unitree/robot/channel/channel_factory.hpp>

SDK2RobotControl::SDK2RobotControl(const std::string &network_interface, bool auto_stand, const std::string& config_path)
{
    std::cout << "[SDK2 | RobotControl] Initializing..." << std::endl;
    std::cout << "ctor iface = " << network_interface
              << "auto_stand = " << (auto_stand ? "true" : "false") << std::endl;

    // Initialize DDS
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface.c_str());

    // Create controllers
    sport_controller_ = std::make_unique<SDK2SportController>(network_interface);
    policy_controller_ = std::make_unique<SDK2PolicyController>(config_path);

    // Auto stand up if requested
    if (auto_stand) {
        sport_controller_->standUp();
    }

    // Initialize joystick subscriber
    joystick_sub_.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>(TOPIC_JOYSTICK));
    joystick_sub_->InitChannel(std::bind(&SDK2RobotControl::onJoystickMessage, this, std::placeholders::_1), 1);
    std::cout << "[SDK2 | RobotControl] Joystick subscriber initialized." << std::endl;

    // Set initial state
    current_state_ = ControlState::SPORT_MODE;
    std::cout << "[SDK2 | RobotControl] Initial state: SPORT_MODE" << std::endl;
}

SDK2RobotControl::~SDK2RobotControl() {
    std::cout << "[SDK2 | RobotControl] Shutting down..." << std::endl;
    // Controllers are managed by unique_ptr and will be cleaned up automatically
    // TODO: 是否要切换到 SPORT_MODE，然后运行 sport_client_->StopMove()
}

void SDK2RobotControl::shutdown() {
    std::cout << "[SDK2 | RobotControl] Performing shutdown cleanup." << std::endl;
    // Ensure policy is stopped and then reset joints
    policy_controller_->setPolicyRunning(false);
    policy_controller_->stopControlLoop();
    policy_controller_->resetJointPosition();
}

void SDK2RobotControl::onJoystickMessage(const void* msg) {
    auto key_data = *reinterpret_cast<const unitree_go::msg::dds_::WirelessController_*>(msg);
    gamepad_.Update(key_data);

    // L1 Press: SAFE EXIT and RESET
    // Always stops the policy, returns to Sport Mode, and resets joint positions.
    if (gamepad_.L1.on_press) {
        gamepad_.L1.on_press = false; // Consume event
        std::cout << "[USER] L1 pressed. SAFE EXIT and RESET initiated." << std::endl;
        switchToSportMode();

        // Launch resetJointPosition in a detached thread to avoid blocking the DDS callback thread
        std::thread([this]() {
            policy_controller_->resetJointPosition();
            std::cout << "[SDK2RobotControl] Joint reset complete." << std::endl;
        }).detach();
    }

    // L2 Press: Enter Policy Mode or Toggle Policy Run/Pause
    if (gamepad_.L2.on_press) {
        gamepad_.L2.on_press = false; // Consume event
        if (current_state_ == ControlState::SPORT_MODE) {
            std::cout << "[USER] L2 pressed. Switching to POLICY_MODE (Paused)." << std::endl;
            switchToPolicyMode();
        } else {
            // Already in POLICY_MODE, so toggle the policy's running state
            policy_controller_->togglePolicyRunning();
        }
    }

    // Pass stick inputs to the policy controller for its observation vector
    // This allows the policy to know the user's desired velocity
    if (current_state_ == ControlState::POLICY_MODE) {
        policy_controller_->applyVelCmdControl(gamepad_.ly, gamepad_.lx, gamepad_.rx);
    }
}

void SDK2RobotControl::switchToPolicyMode() {
    if (current_state_ == ControlState::POLICY_MODE) return;

    // 1. Stop any high-level movement and release the sport mode lock
    sport_controller_->stopMove();
    sport_controller_->releaseMotionModeIfNeeded();

    // 2. Check if mode was released successfully
    if (sport_controller_->queryMotionStatus() != 0) {
        std::cerr << "[SDK2 | RobotControl] ERROR: Failed to release sport mode. Cannot switch to Policy Mode." << std::endl;
        return;
    }

    // 3. Start the policy control loop and set the policy to PAUSED initially
    policy_controller_->startControlLoop();
    policy_controller_->setPolicyRunning(false);
    current_state_ = ControlState::POLICY_MODE;
    std::cout << "[SDK2 | RobotControl] Switched to POLICY_MODE." << std::endl;
}

void SDK2RobotControl::switchToSportMode() {
    if (current_state_ == ControlState::SPORT_MODE) return;

    // 1. Ensure the policy is stopped and then stop its control loop
    policy_controller_->setPolicyRunning(false);
    policy_controller_->stopControlLoop();

    // 2. Switch state. The robot is now idle, waiting for sport commands.
    current_state_ = ControlState::SPORT_MODE;
    std::cout << "[SDK2 | RobotControl] Switched to SPORT_MODE." << std::endl;
}

void SDK2RobotControl::processVelCmd(float vx, float vy, float wz) {
    // This function is called periodically by the ROS2 node.
    // It sends commands to the active controller.
    if (current_state_ == ControlState::SPORT_MODE) {
        sport_controller_->move(vx, vy, wz);
    } else {
        // In policy mode, velocity commands are handled by the joystick callback
        // to be used as part of the policy's observation.
        // We could also choose to blend them, but we keep it simple for now.
    }
}