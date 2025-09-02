#include "sdk1_robot_control_lab.h"

int main(void)
{
    SDK1RobotControlLab robot_control(8082, "192.168.123.10", 8007, 
      "/home/nhy/EmbodiedROS2/src/LeggedControl/config/sdk1_config_lab.ini");
    robot_control.setStandalone(true);
    while (1)
    {
        sleep(10);
    };
    return 0;
}
