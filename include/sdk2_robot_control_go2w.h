#ifndef SDK2_ROBOT_CONTROL_GO2W_HPP
#define SDK2_ROBOT_CONTROL_GO2W_HPP

#include "sdk2_robot_control.h"


class SDK2RobotControlGo2W : public SDK2RobotControl {
public:
    SDK2RobotControlGo2W(const std::string &network_interface, float timeout_s, bool auto_stand, const std::string& config_path);

};

#endif // SDK2_ROBOT_CONTROL_GO2W_HPP