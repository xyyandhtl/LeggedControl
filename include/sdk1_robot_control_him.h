#ifndef SDK1_ROBOT_CONTROL_HIM_HPP
#define SDK1_ROBOT_CONTROL_HIM_HPP

#include "sdk1_robot_control.h"


class SDK1RobotControlHIM : public SDK1RobotControl {
public:
    // 需要调用父类的构造函数
    SDK1RobotControlHIM(uint16_t local_port, const std::string &target_ip, uint16_t target_port, const std::string& config_path);

    const RobotObsResult getRobotObs() override;
};

#endif // SDK1_ROBOT_CONTROL_LAB_HPP