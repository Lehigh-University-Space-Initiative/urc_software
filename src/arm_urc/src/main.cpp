/**
 * @file
 * ArmMotorManager node: drives the robotic arm's joint motors and gripper
 *
 * Runs on: the arm Raspberry Pi
 * Started by: arm_launch.py (run mode "arm")
 *
 * Subscribes:
 *   /roverArmCommands (cross_pkg_messages/RoverComputerArmCMD) - joint position targets from MoveIt
 *   /armInputRaw (cross_pkg_messages/ArmInputRaw) - gripper open/close buttons from the base station
 *
 * Publishes:
 *   /roverArmPos (cross_pkg_messages/RoverComputerArmCMD) - measured joint positions (radians)
 *
 * How it connects to the system:
 *   - The operator moves the arm with the SpaceMouse (SpaceMouseMapper on the base station)
 *   - MoveIt Servo on the main computer turns that motion into joint targets
 *   - The ros2_control plugin on the main computer forwards those targets here as /roverArmCommands
 *   - (That plugin is MockArmHardware, which despite its name is the real bridge, not a simulation)
 *   - This node runs a PID per joint over CAN bus 1 and reports positions back on /roverArmPos
 */
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cross_pkg_messages/msg/rover_computer_arm_cmd.hpp"
#include "cross_pkg_messages/msg/arm_input_raw.hpp"
#include "ArmMotorManager.h"
#include "main.h"
#include <chrono>

std::shared_ptr<rclcpp::Node> node;

std::unique_ptr<ArmMotorManager> manager;

/**
 * Handle new joint targets from MoveIt
 *
 * Parameters (inputs):
 *   msg - target position for each joint (base, shoulder, elbow, three wrist axes)
 */
void callback(const cross_pkg_messages::msg::RoverComputerArmCMD::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("Motor_CTR"), "Received command with CMD_S: %f", msg->cmd_s);
    manager->setArmCommand(msg);
}

/**
 * Handle raw operator input (only the gripper buttons are used here)
 *
 * Parameters (inputs):
 *   msg - SpaceMouse axes and buttons from the base station
 */
void callback2(const cross_pkg_messages::msg::ArmInputRaw::SharedPtr msg)
{
    // Syntax: setArmCommand is overloaded; C++ picks this version because msg is an ArmInputRaw
    manager->setArmCommand(msg);
}

/**
 * Steps:
 *   1. Start ROS, create the node, and declare the PID parameters each joint motor reads at startup
 *   2. Create the joint motors (PID on) and the gripper motor
 *   3. Subscribe to joint targets and gripper input, and advertise the position topic
 *   4. Loop at 100 Hz: process callbacks, tick the motors, and publish positions every other cycle
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    node = rclcpp::Node::make_shared("ArmMotorManager");

    // ROS parameters can be overridden at launch time, e.g. --ros-args -p kp:=0.5
    node->declare_parameter("kp", 1.0);  // PID proportional gain
    node->declare_parameter("kd", 0.01);  // PID derivative gain
    node->declare_parameter("ki", 1.0);  // PID integral gain
    node->declare_parameter("max_i", 0.01);  // Cap on the PID integral term (anti-windup)
    node->declare_parameter("readOnly", false);  // Compute PID output but never power the motors

    RCLCPP_INFO(node->get_logger(), "ArmMotorManager is running");

    manager = std::make_unique<ArmMotorManager>(node, true);
    manager->init();

    auto driveCommandsSub = node->create_subscription<cross_pkg_messages::msg::RoverComputerArmCMD>(
        "/roverArmCommands", 10, callback);
    auto eefCommandSub = node->create_subscription<cross_pkg_messages::msg::ArmInputRaw>(
        "/armInputRaw", 10, callback2);

    // Syntax: create_publisher<MsgType>(topic, queue depth) returns an object whose publish() sends messages
    // System: MockArmHardware on the main computer subscribes to this to report positions to MoveIt
    auto armPosPub = node->create_publisher<cross_pkg_messages::msg::RoverComputerArmCMD>(
        "/roverArmPos", 10);

    rclcpp::Rate loop_rate(100);  // 100 Hz control loop

    bool firstTime = true;

    size_t itr = 0;

    /*
        Performance investigation notes (kept for context):

        TODO:
        - log the period of all update events and incoming topics
        - try moving velocity/position reading to another thread
        - understand the effect of subscription queue length
        - use rqt_plot to see whether delay occurs before/during/after the servo node

        Measured frequencies:
        - full loop:      ~2000-7000 Hz
        - pid tick:       ~600 Hz
        - position publish: ~131 Hz

        Observation: this node is not the bottleneck; the data received from the
        servo node is itself oscillating.
    */

    while (rclcpp::ok()) {
        // Measuring the time since the previous loop iteration started (profiling output)
        static std::chrono::system_clock::time_point last_update;
        double delta = std::chrono::duration<double>(std::chrono::system_clock::now() - last_update).count();
        last_update = std::chrono::system_clock::now();
        RCLCPP_INFO(rclcpp::get_logger("Arm"), "full cycle delta: %f", delta);

        rclcpp::spin_some(node);
        manager->tick();

        // Publishing positions on every other cycle (and skipping the very first one)
        if (!firstTime && ((itr++ % 2) == 0)) {
            // Syntax: this inner static last_update is a separate variable that hides the outer one here
            static std::chrono::system_clock::time_point last_update;
            double delta = std::chrono::duration<double>(std::chrono::system_clock::now() - last_update).count();
            last_update = std::chrono::system_clock::now();
            RCLCPP_INFO(rclcpp::get_logger("Arm"), "publish cycle delta: %f", delta);

            // Motor readings for MoveIt should be in radians from vertical
            manager->readMotors(delta);

            auto& positions = manager->getMotorPositions();
            if (positions.size() >= 6) {
                cross_pkg_messages::msg::RoverComputerArmCMD msg{};

                msg.cmd_b = positions[0];  // Base
                msg.cmd_s = positions[1];  // Shoulder
                msg.cmd_e = positions[2];  // Elbow
                msg.cmd_w.x = positions[3];  // Wrist 1
                msg.cmd_w.y = positions[4];  // Wrist 2
                msg.cmd_w.z = positions[5];  // Wrist 3

                armPosPub->publish(msg);
            }
        }
        firstTime = false;

        {
            // Uses the outer last_update again: how long this iteration took before sleeping
            double delta = std::chrono::duration<double>(std::chrono::system_clock::now() - last_update).count();
            RCLCPP_INFO(rclcpp::get_logger("Arm"), "full cycle actual duration: %f", delta);
        }
        loop_rate.sleep();
    }

    // Destroying the manager (and its motors) before ROS shuts down
    manager = nullptr;

    rclcpp::shutdown();

    return 0;
}
