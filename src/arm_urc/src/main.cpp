#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cross_pkg_messages/msg/rover_computer_arm_cmd.hpp" // TODO: figure these out
#include "arm_urc/ArmMotorManager.h"
#include "arm_urc/main.h"

// Global variables
std::shared_ptr<rclcpp::Node> node;

std::unique_ptr<ArmMotorManager> manager;

// Callback function
void callback(const cross_pkg_messages::msg::RoverComputerArmCMD::SharedPtr msg)
{
    //    RCLCPP_INFO(rclcpp::get_logger("Motor_CTR"), "Received command with CMD_R.z: %f", msg->cmd_r.z);
    // wrist_yaw.setVelocity(msg->cmd_r.z);  // Uncomment and set velocity when integrating
    manager->setArmCommand(msg);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create ROS2 node
    node = rclcpp::Node::make_shared("ArmMotorManager");

    RCLCPP_INFO(node->get_logger(), "ArmMotorManager is running");

    // Construct the manager
    manager = std::make_unique<ArmMotorManager>();

        // Subscriber for rover drive commands
    auto driveCommandsSub = node->create_subscription<cross_pkg_messages::msg::RoverComputerArmCMD>(
        "/roverArmCommands", 10, callback);


    rclcpp::Rate loop_rate(10); // Set rate to 10 Hz

    while (rclcpp::ok())
    {
        // Spin and process ROS callbacks
        rclcpp::spin_some(node);
        manager->tick();
        loop_rate.sleep();
    }

    //dealloc
    manager = nullptr;

    rclcpp::shutdown();
    return 0;
}
