#include "sdk2_robot_control_go2w.h"


SDK2RobotControlGo2W::SDK2RobotControlGo2W(const std::string& network_interface, float timeout_s, bool auto_stand, const std::string& config_path)
    : SDK2RobotControl(network_interface, timeout_s, auto_stand, config_path)
{
    std::cout << "[SDK2 Go2W] Constructor: Inheriting from SDK2RobotControl" << std::endl;
}
