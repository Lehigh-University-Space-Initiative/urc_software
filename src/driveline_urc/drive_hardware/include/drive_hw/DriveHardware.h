#include "CANDriver.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>
#include <vector>
#include <cs_plain_guarded.h>
#include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"


class DriveHardware : public hardware_interface::SystemInterface {

public:

    hardware_interface::CallbackReturn DriveHardware::on_init(const hardware_interface::HardwareInfo &info);

    std::vector<hardware_interface::StateInterface> DriveHardware::export_state_interfaces();
    std::vector<hardware_interface::CommandInterface> DriveHardware::export_command_interfaces();

    hardware_interface::return_type DriveHardware::read(const rclcpp::Time &, const rclcpp::Duration &);
    hardware_interface::return_type DriveHardware::write(const rclcpp::Time &, const rclcpp::Duration &);

private:
    // Joint names
    std::vector<double> joint_names;

    // States and commands
    std::vector<double> joint_pos;
    std::vector<double> joint_vel;
    std::vector<double> joint_effort;
    std::vector<double> joint_cmd_pos;
    std::vector<double> joint_cmd_vel;
    std::vector<double> joint_cmd_effort;

    std::vector<int> can_ids;

    
}