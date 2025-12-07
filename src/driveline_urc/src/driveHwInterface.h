
class DriveHwInterface : public hardware_interface::SystemInterface {

public:

    hardware_interface::CallbackReturn DriveHwInterface::on_init(const hardware_interface::HardwareInfo &info);

    std::vector<hardware_interface::StateInterface> DriveHwInterface::export_state_interfaces();
    std::vector<hardware_interface::CommandInterface> DriveHwInterface::export_command_interfaces();

    hardware_interface::return_type DriveHwInterface::read(const rclcpp::Time &, const rclcpp::Duration &);
    hardware_interface::return_type DriveHwInterface::write(const rclcpp::Time &, const rclcpp::Duration &);

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