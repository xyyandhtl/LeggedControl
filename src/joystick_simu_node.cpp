#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>

#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <map>
#include <array>

// --- Key Value Definitions (from utils.h) ---
const uint16_t KEY_L1 = 1 << 1;
const uint16_t KEY_L2 = 1 << 5;

// Non-blocking character read
int getch() {
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

// --- Key Bindings ---
const std::map<char, std::array<float, 3>> dds_key_bindings = {
    {'w', {1.0, 0.0, 0.0}},  // Forward
    {'s', {-1.0, 0.0, 0.0}}, // Backward
    {'a', {0.0, 1.0, 0.0}},  // Strafe Left
    {'d', {0.0, -1.0, 0.0}}, // Strafe Right
    {'q', {0.0, 0.0, 1.0}},  // Turn Left
    {'e', {0.0, 0.0, -1.0}}  // Turn Right
};

const std::map<char, std::array<float, 3>> ros_key_bindings = {
    {'i', {1.0, 0.0, 0.0}},  // Forward
    {'k', {-1.0, 0.0, 0.0}}, // Backward
    {'j', {0.0, 1.0, 0.0}},  // Strafe Left
    {'l', {0.0, -1.0, 0.0}}, // Strafe Right
    {'u', {0.0, 0.0, 1.0}},  // Turn Left
    {'o', {0.0, 0.0, -1.0}}  // Turn Right
};

const float speed_linear = 0.5; // m/s
const float speed_angular = 1.0; // rad/s

void print_instructions() {
    std::cout << "-----------------------------------------------------" << std::endl;
    std::cout << "Keyboard Teleop Simulator (DDS Joystick & ROS /cmd_vel)" << std::endl;
    std::cout << "-----------------------------------------------------" << std::endl;
    std::cout << "DDS (Simulated Joystick):    | ROS (/cmd_vel) Commands:" << std::endl;
    std::cout << "       w/s: Fwd/Back         |        i/k: Fwd/Back" << std::endl;
    std::cout << "       a/d: Strafe L/R       |        j/l: Strafe L/R" << std::endl;
    std::cout << "       q/e: Turn L/R         |        u/o: Turn L/R" << std::endl;
    std::cout << "-----------------------------------------------------" << std::endl;
    std::cout << "Button Commands (DDS) - ONLY AFFECTS POLICY MODE:" << std::endl;
    std::cout << "       2: Press L2 (Activate Policy / Toggle Run-Pause)" << std::endl;
    std::cout << "       1: Press L1 (Safe Exit: Stop policy and reset joints)" << std::endl;
    std::cout << "-----------------------------------------------------" << std::endl;
    std::cout << "Press [Spacebar] to STOP all velocity commands." << std::endl;
    std::cout << "Press Ctrl+C to quit." << std::endl;
    std::cout << "-----------------------------------------------------" << std::endl;
}

int main(int argc, char **argv) {
    // Init ROS
    rclcpp::init(argc, argv);
    auto ros_node = rclcpp::Node::make_shared("joystick_simu_node");
    std::string network_interface = ros_node->declare_parameter<std::string>("network_interface", "lo");
    auto ros_publisher = ros_node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Init DDS
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface.c_str());
    auto dds_publisher = std::make_unique<unitree::robot::ChannelPublisher<unitree_go::msg::dds_::WirelessController_>>("rt/wirelesscontroller");
    dds_publisher->InitChannel();

    print_instructions();

    while (rclcpp::ok()) {
        int key = getch();

        if (key == '\003') { // Ctrl+C
            break;
        }

        bool cmd_sent = false;
        unitree_go::msg::dds_::WirelessController_ dds_msg;
        geometry_msgs::msg::Twist ros_msg;

        // Check for DDS commands (movement)
        if (dds_key_bindings.count(key)) {
            const auto& cmd = dds_key_bindings.at(key);
            dds_msg.ly() = cmd[0]; // Fwd/Back
            dds_msg.lx() = cmd[1]; // Strafe
            dds_msg.rx() = cmd[2]; // Turn
            dds_publisher->Write(dds_msg);
            printf("[DDS] Sent Velocity: ly=%.1f, lx=%.1f, rx=%.1f\n", dds_msg.ly(), dds_msg.lx(), dds_msg.rx());
            cmd_sent = true;
        }
        // Check for DDS commands (buttons)
        else if (key == '1') {
            dds_msg.keys() = KEY_L1;
            dds_publisher->Write(dds_msg);
            printf("[DDS] Sent Button Click: L1\n");
            // Immediately send a release message to simulate a click
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            dds_msg.keys() = 0;
            dds_publisher->Write(dds_msg);
            cmd_sent = true;
        }
        else if (key == '2') {
            dds_msg.keys() = KEY_L2;
            dds_publisher->Write(dds_msg);
            printf("[DDS] Sent Button Click: L2\n");
            // Immediately send a release message to simulate a click
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            dds_msg.keys() = 0;
            dds_publisher->Write(dds_msg);
            cmd_sent = true;
        }
        // Check for ROS commands
        else if (ros_key_bindings.count(key)) {
            const auto& cmd = ros_key_bindings.at(key);
            ros_msg.linear.x = cmd[0] * speed_linear;
            ros_msg.linear.y = cmd[1] * speed_linear;
            ros_msg.angular.z = cmd[2] * speed_angular;
            ros_publisher->publish(ros_msg);
            printf("[ROS] Sent Velocity: vx=%.2f, vy=%.2f, wz=%.2f\n", ros_msg.linear.x, ros_msg.linear.y, ros_msg.angular.z);
            cmd_sent = true;
        }

        // Handle stop command (spacebar or any other unassigned key)
        if (!cmd_sent) {
            // Send DDS stop (zero velocity and no keys)
            dds_publisher->Write(dds_msg);

            // Send ROS stop
            ros_publisher->publish(ros_msg);

            if (key == ' ') {
                 printf("[CMD] Sent STOP to both channels.\n");
            }
        }
        printf("\r"); // Return to beginning of line
        fflush(stdout);
        rclcpp::spin_some(ros_node);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    rclcpp::shutdown();
    return 0;
}