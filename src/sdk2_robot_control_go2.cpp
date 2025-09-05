#include "sdk2_robot_control_go2.h"


SDK2RobotControlGo2::SDK2RobotControlGo2(const std::string& network_interface, float timeout_s, bool auto_stand, const std::string& config_path)
    : SDK2RobotControl(network_interface, timeout_s, auto_stand, config_path)
{
    std::cout << "[SDK2 Go2] Constructor: Inheriting from SDK2RobotControl" << std::endl;
}
