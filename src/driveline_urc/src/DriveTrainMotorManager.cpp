#include "DriveTrainMotorManager.h"
#include "main.h"

DriveTrainMotorManager::~DriveTrainMotorManager()
{
}

void DriveTrainMotorManager::setupMotors() {
    // Add motors to the motors vector
    // Left side (BUS 0)
    motors_.emplace_back(node_, 0, 1, 1.0); // LF
    motors_.emplace_back(node_, 0, 2, 1.0); // LM
    motors_.emplace_back(node_, 0, 3, 1.0); // LB
    // Right side (BUS 1)
    motors_.emplace_back(node_, 1, 4, 1.0); // RB
    motors_.emplace_back(node_, 1, 5, 1.0); // RM
    motors_.emplace_back(node_, 1, 6, 1.0); // RF

    RCLCPP_INFO(node_->get_logger(), "Testing Motors");
    for (auto &motor : motors_) {
        motor.ident();
    }
}

void DriveTrainMotorManager::parseDriveCommands(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg) {
    motors_[0].sendPowerCMD(-msg->cmd_l.x / 20);
    motors_[1].sendPowerCMD(-msg->cmd_l.y / 20);
    motors_[2].sendPowerCMD(-msg->cmd_l.z / 20);

    motors_[3].sendPowerCMD(msg->cmd_r.x / 20);
    motors_[4].sendPowerCMD(msg->cmd_r.y / 20);
    motors_[5].sendPowerCMD(msg->cmd_r.z / 20);
}

