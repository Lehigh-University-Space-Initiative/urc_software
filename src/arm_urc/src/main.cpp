#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cross_pkg_messages/msg/rover_computer_arm_cmd.hpp" // TODO: figure these out
#include "arm_urc/ArmMotorManager.h"
#include "arm_urc/main.h"

// Global variables
std::shared_ptr<rclcpp::Node> node;


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Create ROS2 node
    node = rclcpp::Node::make_shared("ArmMotorManager");

    RCLCPP_INFO(node->get_logger(), "ArmMotorManager is running");

    // Construct the manager
    std::unique_ptr<ArmMotorManager> manager = std::make_unique<ArmMotorManager>();

    rclcpp::Rate loop_rate(10);  // Set rate to 10 Hz

    while (rclcpp::ok()) {
        // Spin and process ROS callbacks
        rclcpp::spin_some(node);
        manager->tick();
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
