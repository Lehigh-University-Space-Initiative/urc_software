/**
 * @file
 * Implementation of the MoveIt-to-arm bridge plugin (see ArmHardware.hpp for how it fits in)
 *
 * Joint order everywhere in this file: base, shoulder, elbow, wrist 1, wrist 2, wrist 3
 * The elbow and wrist 2 signs are flipped between MoveIt and the arm Pi, in both directions
 */
#include "ArmHardware.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>

rclcpp::Logger arm_logger = rclcpp::get_logger("arm logger");

namespace arm_urc
{

/**
 * Steps:
 *   1. Let the base class parse the hardware info from the URDF
 *   2. Require exactly 6 joints, and size the position/velocity/command buffers
 *   3. Create a private node with the /roverArmCommands publisher and /roverArmPos subscriber
 */
hardware_interface::CallbackReturn ArmHardware::on_init(const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // info_ is filled in by the base class from the <ros2_control> block of the URDF
    if (info_.joints.size() != 6) {
        RCLCPP_ERROR(arm_logger, "Arm: Expected 6 joints in URDF, found %zu", info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }
    motorCount = info_.joints.size();

    hw_positions_.resize(motorCount, 0.0);
    hw_velocities_.resize(motorCount, 0.0);
    hw_commands_.resize(motorCount, 0.0);

    node_ = rclcpp::Node::make_shared("mock_arm_hardware");

    armPublisher = node_->create_publisher<cross_pkg_messages::msg::RoverComputerArmCMD>("/roverArmCommands", 10);
    // Syntax: std::placeholders::_1 stands in for the message argument that the subscription passes to the callback
    armPosSubscriber = node_->create_subscription<cross_pkg_messages::msg::RoverComputerArmCMD>("/roverArmPos", 10, std::bind(&ArmHardware::onReceivePosition, this, std::placeholders::_1));

    RCLCPP_INFO(arm_logger, "Arm on_init: command clamp [%.2f, %.2f]", std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(arm_logger, "ArmHardware on_configure done");

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> interfaces;
    for (size_t i = 0; i < motorCount; i++) {
        // Each interface holds a pointer into our buffer, so controllers read our values directly (no copying)
        interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "position", &hw_positions_[i]));
        interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "velocity", &hw_velocities_[i]));
    }

    return interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (size_t i = 0; i < motorCount; i++) {
        // Controllers write position targets straight into hw_commands_[i] through this pointer
        interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "position", &hw_commands_[i]));
    }

    return interfaces;
}

/**
 * Steps:
 *   1. Process any /roverArmPos messages waiting on the private node (updates hw_positions_)
 *   2. Publish the current targets in hw_commands_ to the arm Pi, flipping the elbow and wrist 2 signs
 *
 * Publishing here instead of in write() means each cycle sends the targets from the previous cycle
 */
hardware_interface::return_type ArmHardware::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
    rclcpp::spin_some(node_);

    cross_pkg_messages::msg::RoverComputerArmCMD msg{};
    msg.cmd_b = hw_commands_[0];
    msg.cmd_s = hw_commands_[1];
    msg.cmd_e = -hw_commands_[2];
    msg.cmd_w.x = hw_commands_[3];
    msg.cmd_w.y = -hw_commands_[4];
    msg.cmd_w.z = hw_commands_[5];

    armPublisher->publish(msg);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmHardware::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // NOTE: would setCommands() be better? Could writeMotors() be improved like setCommands?

    return hardware_interface::return_type::OK;
}

void ArmHardware::onReceivePosition(const cross_pkg_messages::msg::RoverComputerArmCMD::SharedPtr msg)
{
    // Undoing the same elbow and wrist 2 sign flips applied in read()
    hw_positions_[0] = msg->cmd_b;
    hw_positions_[1] = msg->cmd_s;
    hw_positions_[2] = -msg->cmd_e;
    hw_positions_[3] = msg->cmd_w.x;
    hw_positions_[4] = -msg->cmd_w.y;
    hw_positions_[5] = msg->cmd_w.z;
}

}  // namespace arm_urc

// Registering the class with pluginlib so ros2_control can create it by name at runtime (see plugin.xml)
PLUGINLIB_EXPORT_CLASS(arm_urc::ArmHardware, hardware_interface::SystemInterface)
