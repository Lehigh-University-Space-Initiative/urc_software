/**
 * @file
 * DriveTrainManager node: turns a whole-rover drive command into per-wheel speeds
 *
 * Runs on: the main computer (10.0.0.10)
 * Started by: main_computer_launch.py (run modes "main_computer" and "hootl")
 *
 * Subscribes:
 *   cmd_vel (geometry_msgs/Twist) - desired rover motion (linear.x forward speed, angular.y turn rate)
 *
 * Publishes:
 *   roverDriveCommands (cross_pkg_messages/RoverComputerDriveCMD) - wheel angular velocity per side, in rad/s
 *
 * How it connects to the system:
 *   - The base station's JoyMapper publishes cmd_vel from the two joysticks
 *   - This node converts it to left/right wheel speeds (differential, or "tank", steering)
 *   - The driveline Pi's MotorCtr_node receives roverDriveCommands and powers the wheel motors
 *
 * Steering convention (important when adding new cmd_vel publishers):
 *   - This node steers with angular.y in degrees per second, matching JoyMapper ("+Y is rover top")
 *   - Standard ROS (REP 103) instead uses angular.z in radians per second
 *   - navigation_urc's WaypointFollower follows the ROS standard, so its turn commands are ignored here
 *   - Unifying the two is a team decision; see "Known integration issues" in the README
 */
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"

cross_pkg_messages::msg::RoverComputerDriveCMD currentDriveCommand{};  // Latest per-wheel command
rclcpp::Publisher<cross_pkg_messages::msg::RoverComputerDriveCMD>::SharedPtr driveTrainPublisher;

std::shared_ptr<rclcpp::Node> node;

/// Publish currentDriveCommand to the driveline Pi
void sendDrivePowers()
{
    driveTrainPublisher->publish(currentDriveCommand);
}

/**
 * Convert one whole-rover drive command into per-wheel speeds and send it to the driveline
 *
 * Parameters (inputs):
 *   msg - desired motion: linear.x in m/s, angular.y in deg/s (see the steering convention above)
 *
 * Steps:
 *   1. Split the command into a left-side and a right-side ground speed (m/s)
 *   2. Divide by the wheel radius to get each side's wheel angular velocity (rad/s)
 *   3. Give all three wheels on a side the same speed, and publish
 */
void manualInputCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    const float roverWidth = 0.5;  // Distance between the left and right wheels, in meters
    const float wheelRadius = 0.1143;  // Wheel radius in meters (4.5 inches)

    // Converting the turn rate from deg/s to rad/s (3.14 / 180) and scaling it by the rover width
    // The textbook differential-drive formula uses half the width, so turning here is twice that formula's strength
    float leftSideVel = msg->linear.x + msg->angular.y * 3.14 / 180 * roverWidth;
    float rightSideVel = msg->linear.x - msg->angular.y * 3.14 / 180 * roverWidth;

    // Ground speed (m/s) divided by wheel radius (m) gives wheel angular velocity (rad/s)
    float leftSideAlpha = leftSideVel / wheelRadius;
    float rightSideAlpha = rightSideVel / wheelRadius;

    // x, y, z are the front, middle, and back wheel on each side
    currentDriveCommand.cmd_l.x = leftSideAlpha;
    currentDriveCommand.cmd_l.y = leftSideAlpha;
    currentDriveCommand.cmd_l.z = leftSideAlpha;

    currentDriveCommand.cmd_r.x = rightSideAlpha;
    currentDriveCommand.cmd_r.y = rightSideAlpha;
    currentDriveCommand.cmd_r.z = rightSideAlpha;

    sendDrivePowers();
}

int main(int argc, char** argv)
{
    // Syntax: rclcpp::init must run before any other ROS call; it reads ROS command-line arguments
    rclcpp::init(argc, argv);

    node = rclcpp::Node::make_shared("DriveTrainManager");

    RCLCPP_INFO(node->get_logger(), "DriveTrainManager is running");

    // Syntax: a topic name without a leading "/" is relative to the node's namespace (here the root, so "/roverDriveCommands")
    // System: the driveline Pi's MotorCtr_node subscribes to this
    driveTrainPublisher = node->create_publisher<cross_pkg_messages::msg::RoverComputerDriveCMD>("roverDriveCommands", 10);

    // System: the base station's JoyMapper publishes this (and so does navigation's WaypointFollower)
    auto subscription = node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, manualInputCallback);

    rclcpp::Rate loop_rate(10);  // 10 Hz; this node only reacts to messages, so the loop rate just bounds latency

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
