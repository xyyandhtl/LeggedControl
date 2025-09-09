#include <iostream>
#include <string>
#include <thread>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>

#define TOPIC_JOYSTICK "rt/wirelesscontroller"

// According to advanced_gamepad.hpp
// L1 is bit 1, L2 is bit 5
const uint16_t KEY_L1 = (1 << 1);
const uint16_t KEY_L2 = (1 << 5);

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <network_interface>" << std::endl;
        return -1;
    }
    std::string network_interface = argv[1];

    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

    auto pub = std::make_shared<unitree::robot::ChannelPublisher<unitree_go::msg::dds_::WirelessController_>>(TOPIC_JOYSTICK);
    pub->InitChannel();

    unitree_go::msg::dds_::WirelessController_ msg;
    msg.lx() = 0.0f;
    msg.ly() = 0.0f;
    msg.rx() = 0.0f;
    msg.ry() = 0.0f;
    msg.keys() = 0;

    std::cout << "Joystick Emulator Started." << std::endl;
    std::cout << "Commands:" << std::endl;
    std::cout << "  l1     - Simulate L1 press" << std::endl;
    std::cout << "  l2     - Simulate L2 press" << std::endl;
    std::cout << "  exit   - Quit" << std::endl;
    std::cout << "--------------------------" << std::endl;

    while (true) {
        std::cout << "> ";
        std::string input;
        std::cin >> input;

        uint16_t key_to_press = 0;
        if (input == "l1") {
            key_to_press = KEY_L1;
            std::cout << "Simulating L1 press..." << std::endl;
        } else if (input == "l2") {
            key_to_press = KEY_L2;
            std::cout << "Simulating L2 press..." << std::endl;
        } else if (input == "exit") {
            break;
        }
        else {
            std::cout << "Unknown command." << std::endl;
            continue;
        }

        // Simulate a button press and release
        // Press
        msg.keys() = key_to_press;
        pub->Write(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        // Release
        msg.keys() = 0;
        pub->Write(msg);
    }

    return 0;
}
