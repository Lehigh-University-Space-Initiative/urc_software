/**
 * @file
 * MotorCtr_node: drives the rover's six wheel motors from drive commands
 *
 * Runs on: the driveline Raspberry Pi (10.0.0.20)
 * Started by: driveline_launch.py (run mode "driveline", or "hootl" for an off-rover test)
 *
 * Subscribes:
 *   /roverDriveCommands (cross_pkg_messages/RoverComputerDriveCMD) - per-wheel speeds from the main computer
 *
 * How it connects to the system:
 *   - The base station's JoyMapper (or the navigation stack) publishes /cmd_vel
 *   - DriveTrainManager on the main computer turns /cmd_vel into per-wheel speeds on /roverDriveCommands
 *   - This node turns those speeds into SparkMax power commands over CAN bus 0
 *   - Without the CAN HAT (WSL, laptop, hootl) it runs in no-hardware mode and ignores motor output
 */
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>
#include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "CANDriver.h"
#include "DriveTrainMotorManager.h"

// Syntax: std::shared_ptr is a reference-counted pointer; the object is deleted when the last copy goes away
std::shared_ptr<rclcpp::Node> node;
std::unique_ptr<DriveTrainMotorManager> manager;

/**
 * Handle one drive command from the main computer
 *
 * Parameters (inputs):
 *   msg - per-wheel speeds for the left and right sides
 */
void callback(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("Motor_CTR"), "Received command with CMD_R.z: %f", msg->cmd_r.z);
    manager->parseDriveCommands(msg);
}

/**
 * Steps:
 *   1. Start ROS and create the node, declaring the PID parameters every SparkMax reads at startup
 *   2. Create the six wheel motors (PID off, since the driveline sends open-loop power)
 *   3. Subscribe to /roverDriveCommands
 *   4. Loop: process ROS callbacks, then run one motor-manager tick
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("DriveTrainMotorManager");

    // ROS parameters can be overridden at launch time, e.g. --ros-args -p kp:=0.5
    node->declare_parameter("kp", 1.0);  // PID proportional gain
    node->declare_parameter("kd", 0.01);  // PID derivative gain
    node->declare_parameter("ki", 1.0);  // PID integral gain
    node->declare_parameter("max_i", 0.01);  // Cap on the PID integral term (anti-windup)
    node->declare_parameter("readOnly", false);  // Compute outputs but never power the motors

    RCLCPP_INFO(node->get_logger(), "Motor CTR startup");

    // Syntax: std::make_unique<T>(args) creates a T on the heap owned by a unique_ptr
    manager = std::make_unique<DriveTrainMotorManager>(node, false);
    manager->init();

    // Tight control loop; CAN reads inside tick() are the real rate limiter
    rclcpp::Rate loop_rate(30000);

    // Syntax: create_subscription<MsgType>(topic, queue depth, callback) calls callback for every message
    auto driveCommandsSub = node->create_subscription<cross_pkg_messages::msg::RoverComputerDriveCMD>(
        "/roverDriveCommands", 10, callback);

    while (rclcpp::ok()) {
        // Syntax: spin_some runs any callbacks that are ready, then returns (unlike spin, which never returns)
        rclcpp::spin_some(node);
        manager->tick();
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
