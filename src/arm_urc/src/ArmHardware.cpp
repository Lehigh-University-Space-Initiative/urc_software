#include "arm_urc/ArmHardware.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include "arm_urc/main.h"

rclcpp::Logger arm_logger = rclcpp::get_logger("arm logger");
#include "arm_urc/ArmMotorManager.h"

namespace arm_urc
{


hardware_interface::CallbackReturn ArmHardware::on_init(const hardware_interface::HardwareInfo &info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 6) {
    RCLCPP_ERROR(node->get_logger(), "Arm: Expected 6 joints in URDF, found %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(node->get_logger(), "Arm on_init: command clamp [%.2f, %.2f]", -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
  ArmHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Create a logger for the arm hardware
  // rclcpp::Logger dl_logger = rclcpp::get_logger("arm logger");

  // Construct the manager
  manager_ = std::make_unique<ArmMotorManager>();

  RCLCPP_INFO(node->get_logger(), "ArmHardware on_configure done");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
  ArmHardware::export_state_interfaces()
{
  // std::vector<hardware_interface::StateInterface> interfaces;
  // for (size_t i = 0; i < 6; i++) {
  //   interfaces.emplace_back(hardware_interface::StateInterface(
  //     info_.joints[i].name, /*hardware_interface::HW_IF_POSITION*/ "position", &hw_positions_[i]));
  //   interfaces.emplace_back(hardware_interface::StateInterface(
  //     info_.joints[i].name, /*hardware_interface::HW_IF_VELOCITY*/ "velocity", &hw_velocities_[i]));
  // }

  std::vector<hardware_interface::StateInterface> interfaces = manager_->getStateInterfaces(info_.joints);

  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
  ArmHardware::export_command_interfaces()
{
  // std::vector<hardware_interface::CommandInterface> interfaces;
  // for (size_t i = 0; i < 6; i++) {
  //   interfaces.emplace_back(hardware_interface::CommandInterface(
  //     info_.joints[i].name, /*hardware_interface::HW_IF_VELOCITY*/ "velocity", &hw_commands_[i]));
  // }
  std::vector<hardware_interface::CommandInterface> interfaces = manager_->getCommandInterface(info_.joints);
  return interfaces;
}

hardware_interface::return_type
  ArmHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
{
  manager_->tick();

  // read the actual velocities from the motors
  manager_->readMotors(period);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
  ArmHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  manager_->writeMotors();
  //
  // NOTE: would setCommands() be better? Could writeMotors() be improved like setCommands?
  //

  return hardware_interface::return_type::OK;
}

}
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(arm_urc::ArmHardware, hardware_interface::SystemInterface)