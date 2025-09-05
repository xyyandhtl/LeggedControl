#include "sdk2_robot_control_go2w.h"

int main(void)
{
    SDK2RobotControlGo2W robot_control("eth0", 3.0, true,
      "/home/nhy/EmbodiedROS2/src/LeggedControl/config/sdk2_config_go2w.ini");
    robot_control.setStandalone(true);
    while (1)
    {
        sleep(10);
    };
    return 0;
}