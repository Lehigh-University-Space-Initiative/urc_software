#include "arm_urc/ArmMotorManager.h"
#include "arm_urc/Logger.h"

ArmMotorManager::ArmMotorManager() : MotorManager()
{ 
}

ArmMotorManager::~ArmMotorManager()
{
}


void ArmMotorManager::setupMotors()
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

    RCLCPP_INFO(dl_logger, "ArmMotorManager: Testing Motors");
    for (auto &motor : motors_) {
        motor.ident();
    }
}



