#ifndef SDK2_ROBOT_CONTROL_GO2_HPP
#define SDK2_ROBOT_CONTROL_GO2_HPP

#include "sdk2_robot_control.h"


class SDK2RobotControlGo2 : public SDK2RobotControl {
public:
    SDK2RobotControlGo2(const std::string &network_interface, float timeout_s, bool auto_stand, const std::string& config_path);

};

#endif // SDK2_ROBOT_CONTROL_GO2_HPP