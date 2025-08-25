#include "sdk2_robot_control.h"

int main(void)
{
    SDK2RobotControl robot_control("eth0", 3.0, true,
      "/home/lenovo/Projects/EmbodiedROS2/src/LeggedControl/config/sdk2_config.ini");
    robot_control.setStandalone(true);
    while (1)
    {
        sleep(10);
    };
    return 0;
}