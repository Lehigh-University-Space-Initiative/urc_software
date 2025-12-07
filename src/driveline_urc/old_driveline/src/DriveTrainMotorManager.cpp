#include "DriveTrainMotorManager.h"
#include "main.h"

DriveTrainMotorManager::~DriveTrainMotorManager()
{
}

void DriveTrainMotorManager::setupMotors() {
    // Add motors to the motors vector
    // CAN0 (Left side)
    //   LF: Left front wheel, CAN ID 1
    //   LR: Left back wheel, CAN ID 2
    //   LS: Left steering (front wheel), CAN ID 5
    // CAN1 (Right side)
    //   LF: Right front wheel, CAN ID 3
    //   LR: Right back wheel, CAN ID 4
    //   LS: Right steering (front wheel), CAN ID 6
    motors_.emplace_back(node_, 0, 1, 1.0, true); // LF
    motors_.emplace_back(node_, 0, 2, 1.0, true); // LB
    motors_.emplace_back(node_, 0, 3, 1.0, true); // RF
    motors_.emplace_back(node_, 0, 4, 1.0, true); // RB
    // motors_.emplace_back(node_, 0, 5, 1.0, true); // LS
    // motors_.emplace_back(node_, 0, 6, 1.0, true); // RS

    RCLCPP_INFO(node_->get_logger(), "Testing Motors... (all motors should flash quickly)");
    testMotors();
}

void DriveTrainMotorManager::parseDriveCommands(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg) {
    motors_[0].sendPowerCMD(-msg->lf / 20);
    motors_[1].sendPowerCMD(-msg->lb / 20);

    motors_[2].sendPowerCMD(-msg->rf / 20);
    motors_[3].sendPowerCMD(msg->rb / 20);

    // motors_[4].sendPowerCMD(msg->ls / 20);
    // motors_[5].sendPowerCMD(msg->rs / 20);
}

