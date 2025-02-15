#pragma once

#include <memory>
#include <vector>
#include <rclcpp_lifecycle/state.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include "cross_pkg_messages/msg/rover_computer_arm_cmd.hpp" // TODO: figure these out

extern rclcpp::Logger arm_logger;

namespace arm_urc
{

class ArmHardware : public hardware_interface::SystemInterface
{
public:

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  rclcpp::Publisher<cross_pkg_messages::msg::RoverComputerArmCMD>::SharedPtr armPublisher;
  rclcpp::Subscription<cross_pkg_messages::msg::RoverComputerArmCMD>::SharedPtr armPosSubscriber;

private:

  rclcpp::Node::SharedPtr node_;

    void onReceivePosition(const cross_pkg_messages::msg::RoverComputerArmCMD::SharedPtr msg);


    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_commands_;
    size_t motorCount = 0;
};

}  // namespace arm_urc
