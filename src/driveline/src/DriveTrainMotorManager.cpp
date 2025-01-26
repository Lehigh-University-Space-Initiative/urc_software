#include "DriveTrainMotorManager.h"
#include "main.h"

DriveTrainMotorManager::DriveTrainMotorManager()
{

    setupMotors();

    // Subscribe to drive commands
    driveCommandsSub = node->create_subscription<cross_pkg_messages::msg::RoverComputerDriveCMD>(
        "/roverDriveCommands", 10, [this](const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg) {
            auto lock = lastManualCommandTime.lock();
            *lock = std::chrono::system_clock::now();
            parseDriveCommands(msg);
        });

    wheelVelPub = node->create_publisher<cross_pkg_messages::msg::RoverComputerDriveCMD>("motorVels", 10);
}

DriveTrainMotorManager::~DriveTrainMotorManager()
{
}

void DriveTrainMotorManager::setupMotors()
{
    // Add motors to the motors vector
    // Left side (BUS 0)
    motors.emplace_back(SparkMax(1, 1)); // LF
    motors.emplace_back(SparkMax(1, 2)); // LM
    motors.emplace_back(SparkMax(1, 3)); // LB
    // Right side (BUS 1)
    motors.emplace_back(SparkMax(1, 4)); // RB
    motors.emplace_back(SparkMax(1, 5)); // RM
    motors.emplace_back(SparkMax(1, 6)); // RF

    RCLCPP_INFO(node->get_logger(), "Testing Motors");
    for (auto &motor : motors) {
        motor.ident();
    }
}

void DriveTrainMotorManager::stopAllMotors()
{
    for (auto &motor : motors) {
        motor.motorLocked = true;
        motor.sendPowerCMD(0);
    }
}

void DriveTrainMotorManager::sendHeartbeats()
{
    for (auto &motor : motors) {
        motor.sendHeartbeat();
    }
}

void DriveTrainMotorManager::tick()
{

    static uint64_t loopItr = 0;
    loopItr++;

    //read any can message at a much higher rate than all other update tasks
    //TODO: URC-102: should move this back 
    CANDriver::doCanReadIter(1);

    if (loopItr % 30 != 0) return;

    {  // lock block for LOS safety stop
        auto lock = lastManualCommandTime.lock();
        auto now = std::chrono::system_clock::now();
        if (now - *lock > manualCommandTimeout) {
            RCLCPP_WARN(node->get_logger(), "LOS Safety Stop");
            stopAllMotors();
        }
    }

    //give pid tick
    for (auto &motor : motors) {
        motor.sendHeartbeat();
        motor.pidTick();
    }

    // publish wheel data
    cross_pkg_messages::msg::RoverComputerDriveCMD speedMsg;
    speedMsg.cmd_l.x = motors[0].lastVelocityAsRadPerSec();
    speedMsg.cmd_l.y = motors[1].lastVelocityAsRadPerSec();
    speedMsg.cmd_l.z = motors[2].lastVelocityAsRadPerSec();

    speedMsg.cmd_r.x = motors[3].lastVelocityAsRadPerSec();
    speedMsg.cmd_r.y = motors[4].lastVelocityAsRadPerSec();
    speedMsg.cmd_r.z = motors[5].lastVelocityAsRadPerSec();

    wheelVelPub->publish(speedMsg);
}

void DriveTrainMotorManager::parseDriveCommands(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg)
{
    // RCLCPP_INFO(node->get_logger(), "Drive Commands Received with L: %f, R: %f", msg->cmd_l.x, msg->cmd_r.x);

    // for (auto m : motors) {
    //     m.motorLocked = false;
    // }

    // Send power commands to motors based on drive command message
    // motors[0].setPIDSetpoint(-msg->cmd_l.x);
    // motors[1].setPIDSetpoint(-msg->cmd_l.y);
    // motors[2].setPIDSetpoint(-msg->cmd_l.z);

    // motors[3].setPIDSetpoint(msg->cmd_r.x);
    // motors[4].setPIDSetpoint(msg->cmd_r.y);
    // motors[5].setPIDSetpoint(msg->cmd_r.z);

    
    motors[0].sendPowerCMD(-msg->cmd_l.x / 20);
    motors[1].sendPowerCMD(-msg->cmd_l.y / 20);
    motors[2].sendPowerCMD(-msg->cmd_l.z / 20);

    motors[3].sendPowerCMD(msg->cmd_r.x / 20);
    motors[4].sendPowerCMD(msg->cmd_r.y / 20);
    motors[5].sendPowerCMD(msg->cmd_r.z / 20);


}
