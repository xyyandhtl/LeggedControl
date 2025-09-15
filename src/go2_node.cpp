#include <rclcpp/rclcpp.hpp>
#include "sdk2_sport_control.h"
#include "sdk2_policy_control.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create a temporary node to get the control_mode parameter
    auto temp_node = std::make_shared<rclcpp::Node>("go2_launcher_temp");
    std::string control_mode = temp_node->declare_parameter<std::string>("control_mode", "sport");

    rclcpp::NodeOptions options;

    if (control_mode == "sport") {
        auto node = std::make_shared<SDK2SportControl>(options);
        rclcpp::spin(node);
    } else if (control_mode == "policy") {
        auto node = std::make_shared<SDK2PolicyControl>(options);
        rclcpp::spin(node);
    } else {
        RCLCPP_ERROR(temp_node->get_logger(), "Invalid control_mode: %s. Must be 'sport' or 'policy'.", control_mode.c_str());
    }

    rclcpp::shutdown();
    return 0;
}
