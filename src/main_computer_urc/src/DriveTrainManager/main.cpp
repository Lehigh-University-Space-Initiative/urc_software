#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"
#include "controller_interface/controller_interface.hpp"

// Global variables
// cross_pkg_messages::msg::RoverComputerDriveCMD currentDriveCommand{};
// rclcpp::Publisher<cross_pkg_messages::msg::RoverComputerDriveCMD>::SharedPtr driveTrainPublisher;

std::shared_ptr<rclcpp::Node> node;
float leftSideAlpha = 0.0;
float rightSideAlpha = 0.0;

// void sendDrivePowers() {
//     driveTrainPublisher->publish(currentDriveCommand);
// }

void manualInputCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {

    //kinimatic data (metric system)
    const float roverWidth = 0.5; //meters
    const float wheelRadius = 0.1143; //meters

    // breakdown commanded lin and ang vel into left and right side vels
    float leftSideVel = msg->linear.x + msg->angular.y * 3.14 / 180 * roverWidth;
    float rightSideVel = msg->linear.x - msg->angular.y * 3.14 / 180 * roverWidth;


    leftSideAlpha = leftSideVel / wheelRadius;
    rightSideAlpha = rightSideVel / wheelRadius;


    // currentDriveCommand.cmd_l.x = leftSideAlpha;
    // curre ntDriveCommand.cmd_l.y = leftSideAlpha;
    // currentDriveCommand.cmd_l.z = leftSideAlpha;

    // currentDriveCommand.cmd_r.x = rightSideAlpha;
    // currentDriveCommand.cmd_r.y = rightSideAlpha;
    // currentDriveCommand.cmd_r.z = rightSideAlpha;

    // sendDrivePowers();
}

class DriveTrainManager : public controller_interface::ControllerInterface {

    controller_interface::CallbackReturn on_init() {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration command_interface_configuration() const {
        return {
            controller_interface::interface_configuration_type::INDIVIDUAL,
            {"left_wheel/velocity", "right_wheel/velocity"}
        };
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const {
        return {
            controller_interface::interface_configuration_type::INDIVIDUAL,
            {"left_wheel/velocity", "right_wheel/velocity"}
        };
    }

    controller_interface::return_type update(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
        command_interfaces_[0].set_value(leftSideAlpha);
        command_interfaces_[1].set_value(rightSideAlpha);
        return controller_interface::return_type::OK;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Create ROS2 node
    node = rclcpp::Node::make_shared("DriveTrainManager");

    RCLCPP_INFO(node->get_logger(), "DriveTrainManager is running");

    // Create the publisher
    // driveTrainPublisher = node->create_publisher<cross_pkg_messages::msg::RoverComputerDriveCMD>("roverDriveCommands", 10);

    // Create the subscriber
    auto subscription = node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, manualInputCallback);

    rclcpp::Rate loop_rate(10);  // Set rate to 10 Hz

    while (rclcpp::ok()) {
        // Spin and process ROS callbacks
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(DriveTrainManager, controller_interface::ControllerInterface)
