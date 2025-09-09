#ifndef SDK2_SPORT_CONTROLLER_H
#define SDK2_SPORT_CONTROLLER_H

#include <string>
#include <memory>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

class SDK2SportController {
public:
    SDK2SportController(const std::string& network_interface, float timeout_s = 3.0f);
    ~SDK2SportController();

    // High-level commands
    int standUp();
    int move(float vx, float vy, float wz);
    int stopMove();

    // Mode management
    int queryMotionStatus();
    void releaseMotionModeIfNeeded();

private:
    std::unique_ptr<unitree::robot::go2::SportClient> sport_client_;
    std::unique_ptr<unitree::robot::b2::MotionSwitcherClient> msc_;
};

#endif // SDK2_SPORT_CONTROLLER_H
