#include "driveline_urc/DrivelineHardware.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include "driveline_urc/CANDriver.h"
#include "driveline_urc/DriveTrainMotorManager.h"

namespace driveline_urc
{


hardware_interface::CallbackReturn DrivelineHardware::on_init(const hardware_interface::HardwareInfo &info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 6) {
    RCLCPP_ERROR(rclcpp::get_logger("DrivelineHardware"),
                 "Expected 6 joints in URDF, found %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn DrivelineHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  node_ = std::make_shared<rclcpp::Node>("driveline_hw_node");

  manager_ = std::make_unique<DriveTrainMotorManager>();

  RCLCPP_INFO(node_->get_logger(), "DrivelineHardware on_configure completed");
  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> DrivelineHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < 6; i++) {
    interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "position", &hw_positions_[i]));
    interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "velocity", &hw_velocities_[i]));
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> DrivelineHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < 6; i++) {
    interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "velocity", &hw_commands_[i]));
  }
  return interfaces;
}

hardware_interface::return_type DrivelineHardware::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  manager_->tick();


  auto & motors = manager_->getMotors();
  for (size_t i = 0; i < 6; i++) {
    double velocity = motors[i].lastVelocityAsRadPerSec();
    hw_velocities_[i] = velocity;
    hw_positions_[i] += velocity * period.seconds();
  }

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type DrivelineHardware::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  manager_->setCommands(hw_commands_);
  return hardware_interface::return_type::OK;
}

}
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(driveline_urc::DrivelineHardware, hardware_interface::SystemInterface)