#include "sdk2_robot_control.h"

int main(void) {
    SDK2RobotControl robot_control("eth0", true,
      "/home/nhy/EmbodiedROS2/src/LeggedControl/config/sdk2_config_go2.ini");
    // robot_control.setStandalone(true); // This is now handled internally by the controller
    while (1) {
        sleep(10);
    };
    return 0;
}