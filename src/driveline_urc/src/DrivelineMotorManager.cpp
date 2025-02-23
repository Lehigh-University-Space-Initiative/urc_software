#include "driveline_urc/DrivelineMotorManager.h"
#include "driveline_urc/Logger.h"

DrivelineMotorManager::~DrivelineMotorManager()
{
}


void MotorManager::setupMotors()
{

    double drivelineGearRatio = 12;

    // Add motors to the motors vector
    // Left side (BUS 0)
    motors_.emplace_back(SparkMax(node_, 1, 1, drivelineGearRatio)); // LF
    motors_.emplace_back(SparkMax(node_, 1, 2, drivelineGearRatio)); // LM
    motors_.emplace_back(SparkMax(node_, 1, 3, drivelineGearRatio)); // LB
    // Right side (BUS 1)
    motors_.emplace_back(SparkMax(node_, 1, 4, drivelineGearRatio)); // RB
    motors_.emplace_back(SparkMax(node_, 1, 5, drivelineGearRatio)); // RM
    motors_.emplace_back(SparkMax(node_, 1, 6, drivelineGearRatio)); // RF

    RCLCPP_INFO(dl_logger, "DrivelineMotorManager: Testing Motors");
    for (auto &motor : motors_) {
        motor.ident();
    }
}



