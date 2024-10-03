#include "ArmMotorManager.h"

void ArmMotorManager::setupMotors()
{
    // Add motor setup logic if necessary
}

void ArmMotorManager::stopAllMotors()
{
    // Add logic to stop all motors if necessary
}

void ArmMotorManager::parseDriveCommands(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg)
{
    wrist_pitch.setVelocity(msg->cmd_l.z);
    shoulder_pitch.setVelocity(msg->cmd_l.x);

    elbow_pitch.setVelocity(-msg->cmd_r.x);
    wrist_roll.setVelocity(-msg->cmd_r.y);
    wrist_yaw.setVelocity(msg->cmd_r.z);
}

ArmMotorManager::ArmMotorManager()
{
    // ROS2 node creation
    nodeHandle = rclcpp::Node::make_shared("arm_motor_manager");

    armCommandsSub = nodeHandle->create_subscription<cross_pkg_messages::msg::RoverComputerDriveCMD>(
        "/manualArmControl", 10, [this](const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg) {
            {
                auto lock = lastManualCommandTime.lock();
                *lock = std::chrono::system_clock::now();
            }
            parseDriveCommands(msg);
        });

    wrist_yaw.setEnable(true);
}

ArmMotorManager::~ArmMotorManager()
{
    wrist_yaw.setEnable(false);
}

void ArmMotorManager::tick()
{
    wrist_yaw.tick();
    wrist_roll.tick();
    wrist_pitch.tick();
    elbow_pitch.tick();
    shoulder_pitch.tick();
}
