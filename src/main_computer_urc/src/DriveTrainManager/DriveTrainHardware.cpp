#include <hardware_interface/system_interface.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"


namespace drive_train_hardware
{


class DriveTrainHardware : public hardware_interface::SystemInterface
{
public:
  // Initialize the hardware interface.
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override
  {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }


    // Initialize wheel velocity variables for 6 joints.
    lf_wheel_velocity_ = 0.0;
    lm_wheel_velocity_ = 0.0;
    lb_wheel_velocity_ = 0.0;
    rf_wheel_velocity_ = 0.0;
    rm_wheel_velocity_ = 0.0;
    rb_wheel_velocity_ = 0.0;


    // Create a ROS node and publisher to publish drive commands.
    rclcpp::NodeOptions options;
    node_ = rclcpp::Node::make_shared("drive_train_hardware", options);
    command_publisher_ = node_->create_publisher<cross_pkg_messages::msg::RoverComputerDriveCMD>(
      "roverDriveCommands", 10);


    return hardware_interface::CallbackReturn::SUCCESS;
  }


  // Export state interfaces for each joint.
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back("lf_wheel_joint", "velocity", &lf_wheel_velocity_);
    state_interfaces.emplace_back("lm_wheel_joint", "velocity", &lm_wheel_velocity_);
    state_interfaces.emplace_back("lb_wheel_joint", "velocity", &lb_wheel_velocity_);
    state_interfaces.emplace_back("rf_wheel_joint", "velocity", &rf_wheel_velocity_);
    state_interfaces.emplace_back("rm_wheel_joint", "velocity", &rm_wheel_velocity_);
    state_interfaces.emplace_back("rb_wheel_joint", "velocity", &rb_wheel_velocity_);
    return state_interfaces;
  }


  // Export command interfaces for each joint.
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back("lf_wheel_joint", "velocity", &lf_wheel_velocity_);
    command_interfaces.emplace_back("lm_wheel_joint", "velocity", &lm_wheel_velocity_);
    command_interfaces.emplace_back("lb_wheel_joint", "velocity", &lb_wheel_velocity_);
    command_interfaces.emplace_back("rf_wheel_joint", "velocity", &rf_wheel_velocity_);
    command_interfaces.emplace_back("rm_wheel_joint", "velocity", &rm_wheel_velocity_);
    command_interfaces.emplace_back("rb_wheel_joint", "velocity", &rb_wheel_velocity_);
    return command_interfaces;
  }


  // Activate the hardware interface.
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }


  // Deactivate the hardware interface.
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }


  // For a fake hardware interface, read() does nothing.
  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return hardware_interface::return_type::OK;
  }


  // The write() method publishes the computed wheel velocities.
  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
   
    cross_pkg_messages::msg::RoverComputerDriveCMD driveCommand;
    driveCommand.cmd_l.x = lf_wheel_velocity_;
    driveCommand.cmd_l.y = lf_wheel_velocity_;
    driveCommand.cmd_l.z = lf_wheel_velocity_;


    driveCommand.cmd_r.x = rf_wheel_velocity_;
    driveCommand.cmd_r.y = rf_wheel_velocity_;
    driveCommand.cmd_r.z = rf_wheel_velocity_;


    command_publisher_->publish(driveCommand);
    return hardware_interface::return_type::OK;
  }


private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<cross_pkg_messages::msg::RoverComputerDriveCMD>::SharedPtr command_publisher_;
  double lf_wheel_velocity_;
  double lm_wheel_velocity_;
  double lb_wheel_velocity_;
  double rf_wheel_velocity_;
  double rm_wheel_velocity_;
  double rb_wheel_velocity_;
};


}  // namespace drive_train_hardware


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(drive_train_hardware::DriveTrainHardware, hardware_interface::SystemInterface)

