/**
 * @file
 * ros2_control hardware plugin that bridges MoveIt on the main computer to the arm Pi over ROS topics
 *
 * Despite the "Mock" folder and library name, this is the real path to the arm, not a simulation:
 *   - MoveIt's arm_controller writes joint position targets into this plugin's command interfaces
 *   - The plugin publishes those targets on /roverArmCommands for arm_urc's ArmMotorManager to execute
 *   - ArmMotorManager reports measured positions on /roverArmPos, which the plugin feeds back to MoveIt
 *
 * How it gets loaded:
 *   - description/robot.ros2_control.xacro names this plugin as main_computer_urc/MockArmHardware
 *   - plugin.xml maps that name to the class arm_urc::ArmHardware in the library libmock_arm_hw.so
 *   - The controller_manager (ros2_control_node, started by main_computer_launch.py) loads it at runtime
 */
#pragma once

#include <memory>
#include <vector>
#include <rclcpp_lifecycle/state.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include "cross_pkg_messages/msg/rover_computer_arm_cmd.hpp"

extern rclcpp::Logger arm_logger;

// Syntax: a namespace groups names to avoid clashes; this class's full name is arm_urc::ArmHardware
namespace arm_urc
{

/**
 * The arm as seen by ros2_control: six position-controlled joints
 *
 * Syntax: "override" marks a method that replaces a virtual method of the base class
 * The compiler errors if the base class has no matching method, which catches typos in signatures
 */
class ArmHardware : public hardware_interface::SystemInterface
{
public:
    /// Check the URDF lists 6 joints, size the buffers, and create the topic publisher/subscriber
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    /// Lifecycle "configure" step (nothing to do here)
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    /// Expose each joint's position and velocity so controllers can read them
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    /// Expose each joint's position command so controllers can write targets
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    /// Called every control cycle: process incoming positions and publish the current targets
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    /// Called every control cycle after the controllers run (nothing to do; read() publishes)
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    // System: arm_urc's ArmMotorManager subscribes to /roverArmCommands and publishes /roverArmPos
    rclcpp::Publisher<cross_pkg_messages::msg::RoverComputerArmCMD>::SharedPtr armPublisher;
    rclcpp::Subscription<cross_pkg_messages::msg::RoverComputerArmCMD>::SharedPtr armPosSubscriber;

private:
    // A private ROS node for the topics (a hardware plugin isn't a node, so it creates its own)
    rclcpp::Node::SharedPtr node_;

    /// Store measured joint positions arriving on /roverArmPos
    void onReceivePosition(const cross_pkg_messages::msg::RoverComputerArmCMD::SharedPtr msg);

    std::vector<double> hw_positions_;  // Measured joint positions (radians), read by controllers
    std::vector<double> hw_velocities_;  // Joint velocities (never updated; always 0)
    std::vector<double> hw_commands_;  // Joint position targets (radians), written by controllers
    size_t motorCount = 0;  // Number of joints (6)
};

}  // namespace arm_urc
