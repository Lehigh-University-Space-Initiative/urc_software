#include "driveline_urc/DrivelineMotorManager.h"
#include "driveline_urc/Logger.h"

DrivelineMotorManager::DrivelineMotorManager() : MotorManager()
{ 
}

DrivelineMotorManager::~DrivelineMotorManager()
{
}


void MotorManager::setupMotors()
{
    // Add motors to the motors vector
    // Left side (BUS 0)
    motors_.emplace_back(SparkMax(1, 1)); // LF
    motors_.emplace_back(SparkMax(1, 2)); // LM
    motors_.emplace_back(SparkMax(1, 3)); // LB
    // Right side (BUS 1)
    motors_.emplace_back(SparkMax(1, 4)); // RB
    motors_.emplace_back(SparkMax(1, 5)); // RM
    motors_.emplace_back(SparkMax(1, 6)); // RF

    RCLCPP_INFO(dl_logger, "DrivelineMotorManager: Testing Motors");
    for (auto &motor : motors_) {
        motor.ident();
    }
}



