#include "sdk2_robot_control.h"
#include <iostream>

int main(int argc, char **argv) {
    std::string network_interface = "eno1"; // Default interface
    if (argc > 1) {
        network_interface = argv[1];
    }

    SDK2RobotControl robot_control(network_interface, true,
      "/home/nhy/EmbodiedROS2/src/LeggedControl/config/sdk2_config_go2w.ini");
    robot_control.setStandalone(true); // This is now handled internally by the controller
    while (1) {
        sleep(10);
    };
    return 0;
}