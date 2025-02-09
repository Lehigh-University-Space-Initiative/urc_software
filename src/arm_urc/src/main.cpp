#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cross_pkg_messages/msg/rover_computer_arm_cmd.hpp" // TODO: figure these out
#include "arm_urc/ArmMotorManager.h"

// Global variables
cross_pkg_messages::msg::RoverComputerArmCMD currentArmCommand{};
rclcpp::Publisher<cross_pkg_messages::msg::RoverComputerArmCMD>::SharedPtr armPublisher;

std::shared_ptr<rclcpp::Node> node;

rclcpp::Subscription<cross_pkg_messages::msg::RoverComputerArmCMD>::SharedPtr armCommandsSub;
rclcpp::Publisher<cross_pkg_messages::msg::RoverComputerArmCMD>::SharedPtr armVelPub;

void sendArmPowers() {
    ArmPublisher->publish(currentArmCommand);
}

void manualInputCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {

    // //kinimatic data (metric system)
    // const float roverWidth = 0.5; //meters
    // const float wheelRadius = 0.1143; //meters

    // breakdown commanded lin and ang vel into left and right side vels
    // float leftSideVel = msg->linear.x + msg->angular.y * 3.14 / 180 * roverWidth;
    // float rightSideVel = msg->linear.x - msg->angular.y * 3.14 / 180 * roverWidth;


    // float leftSideAlpha = leftSideVel / wheelRadius;
    // float rightSideAlpha = rightSideVel / wheelRadius;


    // TODO: make these actually do something, do some inverse kinematics stuff
    float baseAlpha = 0.0;
    float shoulderAlpha = 0.0;
    float elbowAlpha = 0.0;
    float wristRollAlpha = 0.0;
    float wristPitchAlpha = 0.0;
    float wristYawAlpha = 0.0;


    currentArmCommand.cmd_b = baseAlpha;
    currentArmCommand.cmd_s = shoulderAlpha;
    currentArmCommand.cmd_e = elbowAlpha;
    currentArmCommand.cmd_w.x = wristRollAlpha;
    currentArmCommand.cmd_w.y = wristPitchAlpha;
    currentArmCommand.cmd_w.z = wristYawAlpha;

    sendArmPowers();
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Create ROS2 node
    node = rclcpp::Node::make_shared("ArmMotorManager");

    RCLCPP_INFO(node->get_logger(), "ArmMotorManager is running");

    // Construct the manager
    manager = std::make_unique<ArmMotorManager>();

    // Subscribe to arm commands
    armCommandsSub = node->create_subscription<cross_pkg_messages::msg::RoverComputerArmCMD>(
        "/roverArmCommands", 10, [this](const cross_pkg_messages::msg::RoverComputerArmCMD::SharedPtr msg) {
            auto lock = lastManualCommandTime.lock();
            *lock = std::chrono::system_clock::now();
            manager->parseArmCommands(msg);
        });

    // publish arm velocity
    armVelPub = node->create_publisher<cross_pkg_messages::msg::RoverComputerArmCMD>("armMotorVels", 10);

    rclcpp::Rate loop_rate(10);  // Set rate to 10 Hz

    while (rclcpp::ok()) {
        // Spin and process ROS callbacks
        rclcpp::spin_some(node);
        manager->tick();
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
