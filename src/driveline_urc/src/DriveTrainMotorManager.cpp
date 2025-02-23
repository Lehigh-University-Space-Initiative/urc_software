#include "DriveTrainMotorManager.h"
#include "main.h"

DriveTrainMotorManager::~DriveTrainMotorManager()
{
}

void DriveTrainMotorManager::setupMotors() {
    // Left side (BUS 0)
    motors_.emplace_back(SparkMax(node_, 0, 1, 1.0)); // LF
    motors_.emplace_back(SparkMax(node_, 0, 2, 1.0)); // LM
    motors_.emplace_back(SparkMax(node_, 0, 3, 1.0)); // LB
    // Right side (BUS 1)
    motors_.emplace_back(SparkMax(node_, 1, 4, 1.0)); // RB
    motors_.emplace_back(SparkMax(node_, 1, 5, 1.0)); // RM
    motors_.emplace_back(SparkMax(node_, 1, 6, 1.0)); // RF

    RCLCPP_INFO(node_->get_logger(), "Testing Motors");
    for (auto &motor : motors_) {
        motor.ident();
    }
}

void DriveTrainMotorManager::setCommands(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg) {
    if (motors_.size() < 6) {
        RCLCPP_ERROR(node_->get_logger(), "Insufficient motors configured");
        return;
    }


    motors_[0].sendPowerCMD(-msg->cmd_l.x / 20);
    motors_[1].sendPowerCMD(-msg->cmd_l.y / 20);
    motors_[2].sendPowerCMD(-msg->cmd_l.z / 20);


    motors_[3].sendPowerCMD(msg->cmd_r.x / 20);
    motors_[4].sendPowerCMD(msg->cmd_r.y / 20);
    motors_[5].sendPowerCMD(msg->cmd_r.z / 20);
}

void DriveTrainMotorManager::writeMotors() {
    cross_pkg_messages::msg::RoverComputerDriveCMD speedMsg;
    speedMsg.cmd_l.x = motors_[0].lastVelocityAsRadPerSec();
    speedMsg.cmd_l.y = motors_[1].lastVelocityAsRadPerSec();
    speedMsg.cmd_l.z = motors_[2].lastVelocityAsRadPerSec();


    speedMsg.cmd_r.x = motors_[3].lastVelocityAsRadPerSec();
    speedMsg.cmd_r.y = motors_[4].lastVelocityAsRadPerSec();
    speedMsg.cmd_r.z = motors_[5].lastVelocityAsRadPerSec();


    wheelVelPub_->publish(speedMsg);
}
