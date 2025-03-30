#include "DriveTrainMotorManager.h"
#include "main.h"

DriveTrainMotorManager::~DriveTrainMotorManager()
{
}

void DriveTrainMotorManager::setupMotors() {
    // Add motors to the motors vector
    // Left side (BUS 0)
    motors_.emplace_back(node_, 0, 1, 1.0, true); // LF
    motors_.emplace_back(node_, 0, 2, 1.0, true); // LM
    motors_.emplace_back(node_, 0, 3, 1.0, true); // LB
    motors_.emplace_back(node_, 0, 4, 1.0, true); // RB
    motors_.emplace_back(node_, 0, 5, 1.0, true); // RM
    motors_.emplace_back(node_, 0, 6, 1.0, true); // RF

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

