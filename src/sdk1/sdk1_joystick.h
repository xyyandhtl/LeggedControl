#ifndef SDK1_JOYSTICK_H
#define SDK1_JOYSTICK_H

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/unitree_joystick.h"
#include <iostream>
#include <cstring>

using namespace UNITREE_LEGGED_SDK;

class JoystickInput
{
public:
    JoystickInput(uint16_t local_port, const std::string &target_ip, uint16_t target_port)
        : udp_(local_port, target_ip.c_str(), target_port, sizeof(HighCmd), sizeof(HighState))
    {
        udp_.InitCmdData(cmd_);
    }

    void udpRecv()
    {
        udp_.Recv();
        udp_.GetRecv(state_);
        std::memcpy(&key_data_, &state_.wirelessRemote, sizeof(xRockerBtnDataStruct));
    }

    xRockerBtnDataStruct getJoystickData() const
    {
        return key_data_;
    }

private:
    UDP udp_;
    HighCmd cmd_ = {0};
    HighState state_ = {0};
    xRockerBtnDataStruct key_data_ = {0};
};

#endif // SDK1_JOYSTICK_H