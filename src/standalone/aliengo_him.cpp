#include "sdk1_robot_control_him.h"

int main(void)
{
    SDK1RobotControlHIM robot_control(8082, "192.168.123.10", 8007, 
      "/home/nhy/EmbodiedROS2/src/LeggedControl/config/sdk1_config_him.ini");
    robot_control.setStandalone(true);
    while (1)
    {
        sleep(10);
    };
    return 0;
}
