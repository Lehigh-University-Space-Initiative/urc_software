#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>
#include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "CANDriver.h"
#include "DriveTrainMotorManager.h"

/*
Note 
Left stick:
pitch: pitch
roll: base rotate
yaw: M3 rist pitch

right stick:
pitch: elbow pitch
roll: rist roll
yaw: rist/ yaw

left trigger: close end effector
right trigger: open end effector
*/


std::shared_ptr<rclcpp::Node> node;
std::unique_ptr<DriveTrainMotorManager> manager;

// Callback function
void callback(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg) {
   RCLCPP_INFO(rclcpp::get_logger("Motor_CTR"), "Received command with CMD_R.z: %f", msg->cmd_r.z);
   // wrist_yaw.setVelocity(msg->cmd_r.z);  // Uncomment and set velocity when integrating
   manager->parseDriveCommands(msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("DriveTrainMotorManager");

    node->declare_parameter("kp",1.0);
    node->declare_parameter("kd",0.01);
    node->declare_parameter("ki",1.0);
    node->declare_parameter("max_i",0.01);
    node->declare_parameter("readOnly",false);

    RCLCPP_INFO(node->get_logger(), "Motor CTR startup");

    // Construct the manager
    manager = std::make_unique<DriveTrainMotorManager>(node,false);
    manager->init();

    // Set loop rate to 100 Hz
    rclcpp::Rate loop_rate(30000);

    // Subscriber for rover drive commands
    auto driveCommandsSub = node->create_subscription<cross_pkg_messages::msg::RoverComputerDriveCMD>(
        "/roverDriveCommands", 10, callback);

    // Main loop
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        manager->tick();
        loop_rate.sleep();  // Maintain the loop rate
    }

    rclcpp::shutdown();
    return 0;
}