#include "sdk2_sport_controller.h"
#include <iostream>

SDK2SportController::SDK2SportController(const std::string& network_interface, float timeout_s) {
    // HighLevel(SportClient) 初始化
    sport_client_ = std::make_unique<unitree::robot::go2::SportClient>();
    sport_client_->SetTimeout(timeout_s);
    sport_client_->Init();
    std::cout << "[SDK2 | SportController] SportClient initialized" << std::endl;

    // ModeSwitcher 初始化
    msc_ = std::make_unique<unitree::robot::b2::MotionSwitcherClient>();
    msc_->SetTimeout(timeout_s);
    msc_->Init();
    std::cout << "[SDK2 | SportController] MotionSwitcherClient initialized" << std::endl;
}

SDK2SportController::~SDK2SportController() {
    if (sport_client_) {
        sport_client_->StopMove();
    }
}

int SDK2SportController::standUp() {
    std::cout << "[SDK2 | SportController] StandUp requested" << std::endl;
    int ret = sport_client_->StandUp();
    std::cout << "[SDK2 | SportController] StandUp ret=" << ret << std::endl;
    return ret;
}

int SDK2SportController::move(float vx, float vy, float wz) {
    // std::cout << "[SDK2 | SportController] Move cmd: vx=" << vx << " vy=" << vy << " wz=" << wz << std::endl;
    int ret = sport_client_->Move(vx, vy, wz);
    // std::cout << "[SDK2 | SportController] Move ret=" << ret << std::endl;
    return ret;
}

int SDK2SportController::stopMove() {
    std::cout << "[SDK2 | SportController] StopMove requested" << std::endl;
    int ret = sport_client_->StopMove();
    std::cout << "[SDK2 | SportController] StopMove ret=" << ret << std::endl;
    return ret;
}

int SDK2SportController::queryMotionStatus() {
    std::string robotForm, motionName;
    int motionStatus;
    int32_t ret = msc_->CheckMode(robotForm,motionName);
    if (ret != 0) {
//        std::cout << "[SDK2 | SportController] CheckMode failed. Error code: " << ret << std::endl;
    }

    if(motionName.empty()) {
//        std::cout << "[SDK2 | SportController] The motion control-related service is deactivated." << std::endl;
        motionStatus = 0;
    } else {
        motionStatus = 1;
    }

    return motionStatus;
}

void SDK2SportController::releaseMotionModeIfNeeded() {
    while(queryMotionStatus()) {
        std::cout << "[SDK2 | SportController] Try to deactivate the motion control-related service." << std::endl;
        int32_t ret = msc_->ReleaseMode();

        if (ret == 0) {
            std::cout << "[SDK2 | SportController] ReleaseMode succeeded." << std::endl;
        } else {
            std::cout << "[SDK2 | SportController] ReleaseMode failed. Error code: " << ret << std::endl;
        }
        sleep(1);
    }
}
