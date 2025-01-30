// #include "driveline_urc/driveline_hardware.hpp"
// #include <pluginlib/class_list_macros.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <algorithm>

// namespace driveline_urc
// {

// hardware_interface::CallbackReturn
// DrivelineHardware::on_init(const hardware_interface::HardwareInfo &info)
// {
//   // Let SystemInterface store info_ internally
//   if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
//     return hardware_interface::CallbackReturn::ERROR;
//   }

//   // We expect 6 joints for a 6-wheel rover
//   if (info_.joints.size() != 6) {
//     RCLCPP_ERROR(rclcpp::get_logger("DrivelineHardware"),
//                  "Expected 6 joints in URDF, found %zu", info_.joints.size());
//     return hardware_interface::CallbackReturn::ERROR;
//   }

//   // Resize vectors for position, velocity, command
//   hw_positions_.resize(6, 0.0);
//   hw_velocities_.resize(6, 0.0);
//   hw_commands_.resize(6, 0.0);


//   RCLCPP_INFO(rclcpp::get_logger("DrivelineHardware"),
//               "on_init: command clamp [%.2f, %.2f]", cmd_min_, cmd_max_);
//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn
// DrivelineHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
// {
//   // Create a node specifically for hardware
//   node_ = std::make_shared<rclcpp::Node>("driveline_hw_node");

//   // Construct the manager with this node
//   manager_ = std::make_unique<DriveTrainMotorManager>(node_);

//   RCLCPP_INFO(node_->get_logger(), "DrivelineHardware on_configure done");
//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// std::vector<hardware_interface::StateInterface>
// DrivelineHardware::export_state_interfaces()
// {
//   std::vector<hardware_interface::StateInterface> interfaces;
//   for (size_t i = 0; i < 6; i++) {
//     interfaces.emplace_back(hardware_interface::StateInterface(
//       info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
//     interfaces.emplace_back(hardware_interface::StateInterface(
//       info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
//   }
//   return interfaces;
// }

// std::vector<hardware_interface::CommandInterface>
// DrivelineHardware::export_command_interfaces()
// {
//   std::vector<hardware_interface::CommandInterface> interfaces;
//   for (size_t i = 0; i < 6; i++) {
//     interfaces.emplace_back(hardware_interface::CommandInterface(
//       info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
//   }
//   return interfaces;
// }

// hardware_interface::return_type
// DrivelineHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
// {
//   manager_->tick();

//   // read the actual velocities from the motors
//   auto & motors = manager_->getMotors();
//   for (size_t i = 0; i < 6; i++) {
//     double velocity = motors[i].lastVelocityAsRadPerSec();
//     hw_velocities_[i] = velocity;

//     // Integrate velocity -> position 
//     hw_positions_[i] += velocity * period.seconds();
//   }

//   return hardware_interface::return_type::OK;
// }

// hardware_interface::return_type
// DrivelineHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
// {
//   // You can interpret these hw_commands_ however you'd like. If you want
//   // the /roverDriveCommands approach to remain primary, you can ignore them.
//   // Or if you want to accept velocity commands from a standard controller, do:
//   auto & motors = manager_->getMotors();

//   // For demonstration, let's apply each hw_commands_[i] as a power or velocity setpoint:
//   for (size_t i = 0; i < 6; i++) {
//     double clamped = std::max(cmd_min_, std::min(hw_commands_[i], cmd_max_));
//     // direct power
//     motors[i].sendPowerCMD(static_cast<float>(clamped));
//   }

//   return hardware_interface::return_type::OK;
// }

// }

// PLUGINLIB_EXPORT_CLASS(driveline_urc::DrivelineHardware, hardware_interface::SystemInterface)
