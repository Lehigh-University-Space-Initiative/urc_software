#include "rclcpp/rclcpp.hpp"
#include "cross_pkg_messages/msg/manual_drive_cmd.hpp"
#include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"

// Global variables
cross_pkg_messages::msg::RoverComputerDriveCMD currentDriveCommand{};
rclcpp::Publisher<cross_pkg_messages::msg::RoverComputerDriveCMD>::SharedPtr driveTrainPublisher;

void sendDrivePowers() {
    driveTrainPublisher->publish(currentDriveCommand);
}

void manualInputCallback(const cross_pkg_messages::msg::ManualDriveCMD::SharedPtr msg) {
    // Process incoming manual drive command and update drive powers
    currentDriveCommand.cmd_l.x = msg->value.x;
    currentDriveCommand.cmd_l.y = msg->value.x;
    currentDriveCommand.cmd_l.z = msg->value.x;

    currentDriveCommand.cmd_r.x = msg->value.y;
    currentDriveCommand.cmd_r.y = msg->value.y;
    currentDriveCommand.cmd_r.z = msg->value.y;

    sendDrivePowers();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Create ROS2 node
    auto node = rclcpp::Node::make_shared("DriveTrainManager");

    RCLCPP_INFO(node->get_logger(), "DriveTrainManager is running");

    // Create the publisher
    driveTrainPublisher = node->create_publisher<cross_pkg_messages::msg::RoverComputerDriveCMD>("roverDriveCommands", 10);

    // Create the subscriber
    auto subscription = node->create_subscription<cross_pkg_messages::msg::ManualDriveCMD>(
        "manualCommands", 10, manualInputCallback);

    rclcpp::Rate loop_rate(10);  // Set rate to 10 Hz

    while (rclcpp::ok()) {
        // Spin and process ROS callbacks
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
