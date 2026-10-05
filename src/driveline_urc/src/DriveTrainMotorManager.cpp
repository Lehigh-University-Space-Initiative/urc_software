/**
 * @file
 * Implementation of DriveTrainMotorManager (the driveline's wheel motors)
 */
#include "DriveTrainMotorManager.h"
#include "main.h"

DriveTrainMotorManager::~DriveTrainMotorManager()
{
}

/**
 * Steps:
 *   1. Create one SparkMax per wheel on CAN bus 0, IDs 1-6 (gear ratio 1, absolute encoder flag on)
 *   2. Send each one an ident message, which blinks its status light so a person can confirm wiring
 */
void DriveTrainMotorManager::setupMotors()
{
    // Syntax: emplace_back builds the SparkMax directly inside the vector using these constructor arguments
    // Arguments: (node, CAN bus, CAN ID, gear ratio, use absolute encoder)
    motors_.emplace_back(node_, 0, 1, 1.0, true);  // Left front
    motors_.emplace_back(node_, 0, 2, 1.0, true);  // Left middle
    motors_.emplace_back(node_, 0, 3, 1.0, true);  // Left back
    motors_.emplace_back(node_, 0, 4, 1.0, true);  // Right back
    motors_.emplace_back(node_, 0, 5, 1.0, true);  // Right middle
    motors_.emplace_back(node_, 0, 6, 1.0, true);  // Right front

    RCLCPP_INFO(node_->get_logger(), "Testing Motors");
    for (auto& motor : motors_) {
        motor.ident();
    }
}

/**
 * Steps:
 *   1. Scale each wheel speed into a power level by dividing by 20 (20 rad/s of wheel speed is full power)
 *   2. Negate the left side so a positive command drives both sides forward (left motors presumably mounted mirrored)
 *
 * Notes:
 *   - DriveTrainManager sends each wheel speed as angular velocity in rad/s
 *   - motors_ is ordered LF, LM, LB, RB, RM, RF (see setupMotors)
 *   - So index 3 is the right back wheel but receives cmd_r.x, which the message defines as right front
 *   - That's harmless today (DriveTrainManager sends the same speed to every wheel on a side) but matters for per-wheel control
 */
void DriveTrainMotorManager::parseDriveCommands(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg)
{
    // Syntax: msg is a shared pointer, so "->" reaches the message's fields
    motors_[0].sendPowerCMD(-msg->cmd_l.x / 20);
    motors_[1].sendPowerCMD(-msg->cmd_l.y / 20);
    motors_[2].sendPowerCMD(-msg->cmd_l.z / 20);

    motors_[3].sendPowerCMD(msg->cmd_r.x / 20);
    motors_[4].sendPowerCMD(msg->cmd_r.y / 20);
    motors_[5].sendPowerCMD(msg->cmd_r.z / 20);
}
