#include "ArmHardware.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
// #include "/Logger.h"

rclcpp::Logger arm_logger = rclcpp::get_logger("arm logger");

namespace arm_urc
{


hardware_interface::CallbackReturn ArmHardware::on_init(const hardware_interface::HardwareInfo &info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(arm_logger, "Arm: Expected 2 joints in URDF, found %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  motorCount = info_.joints.size();
    // Resize vectors for position, velocity, command
  hw_positions_.resize(motorCount, 0.0);
  hw_velocities_.resize(motorCount, 0.0);
  hw_commands_.resize(motorCount, 0.0);

  RCLCPP_INFO(arm_logger, "Arm on_init: command clamp [%.2f, %.2f]", std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
  ArmHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Create a logger for the arm hardware
  // rclcpp::Logger arm_logger = rclcpp::get_logger("arm logger");

  RCLCPP_INFO(arm_logger, "ArmHardware on_configure done");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
  ArmHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < motorCount; i++) {
    interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, /*hardware_interface::HW_IF_POSITION*/ "position", &hw_positions_[i]));
    interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, /*hardware_interface::HW_IF_VELOCITY*/ "velocity", &hw_velocities_[i]));
  }

  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
  ArmHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < motorCount; i++) {
    interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, /*hardware_interface::HW_IF_POSITION*/ "position", &hw_commands_[i]));
  }
  return interfaces;
}

hardware_interface::return_type
  ArmHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
{
  // manager_->tick();

  // read the actual velocities from the motors
  // manager_->readMotors(period);
  // if (hw_commands_[0] != 0 || hw_commands_[1] != 0)
  RCLCPP_INFO(arm_logger, "GOT updated arm commands: %.2f %.2f",hw_commands_[0],hw_commands_[1]);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
  ArmHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // manager_->writeMotors();
  //
  // NOTE: would setCommands() be better? Could writeMotors() be improved like setCommands?
  //

  return hardware_interface::return_type::OK;
}

}
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(arm_urc::ArmHardware, hardware_interface::SystemInterface)