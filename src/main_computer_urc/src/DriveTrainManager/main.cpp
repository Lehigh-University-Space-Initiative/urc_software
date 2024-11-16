#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"

// Global variables
cross_pkg_messages::msg::RoverComputerDriveCMD currentDriveCommand{};
rclcpp::Publisher<cross_pkg_messages::msg::RoverComputerDriveCMD>::SharedPtr driveTrainPublisher;

std::shared_ptr<rclcpp::Node> node;

void sendDrivePowers() {
    driveTrainPublisher->publish(currentDriveCommand);
}

void manualInputCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {

    //kinimatic data (metric system)
    const float roverWidth = 0.5; //meters
    const float wheelRadius = 0.1143; //meters

    // breakdown commanded lin and ang vel into left and right side vels
    float leftSideVel = msg->linear.x + msg->angular.y * 3.14 / 180 * roverWidth;
    float rightSideVel = msg->linear.x - msg->angular.y * 3.14 / 180 * roverWidth;


    float leftSideAlpha = leftSideVel / wheelRadius;
    float rightSideAlpha = rightSideVel / wheelRadius;


    currentDriveCommand.cmd_l.x = leftSideAlpha;
    currentDriveCommand.cmd_l.y = leftSideAlpha;
    currentDriveCommand.cmd_l.z = leftSideAlpha;

    currentDriveCommand.cmd_r.x = rightSideAlpha;
    currentDriveCommand.cmd_r.y = rightSideAlpha;
    currentDriveCommand.cmd_r.z = rightSideAlpha;

    sendDrivePowers();
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Create ROS2 node
    node = rclcpp::Node::make_shared("DriveTrainManager");

    RCLCPP_INFO(node->get_logger(), "DriveTrainManager is running");

    // Create the publisher
    driveTrainPublisher = node->create_publisher<cross_pkg_messages::msg::RoverComputerDriveCMD>("roverDriveCommands", 10);

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
