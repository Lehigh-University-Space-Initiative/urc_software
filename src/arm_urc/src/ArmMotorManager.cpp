#include "arm_urc/ArmMotorManager.h"
#include "Logger.h"

ArmMotorManager::ArmMotorManager() : MotorManager()
{ 
}

ArmMotorManager::~ArmMotorManager()
{
}

void ArmMotorManager::writeMotors() {
    for (size_t i = 0; i < motor_count_; i++) {
      printf("Going to send power: %.2f to motor %ld",hw_commands_[i],i);
    }
}

void ArmMotorManager::setArmCommand(const cross_pkg_messages::msg::RoverComputerArmCMD::SharedPtr msg)
{
    //TODO add the rest
    hw_commands_[1] = msg->cmd_s;
    hw_commands_[2] = msg->cmd_e;
}

void ArmMotorManager::setupMotors()
{
    // Add motors to the motors vector
    // Right side (BUS 1)
    motors_.emplace_back(SparkMax(1, 51)); // Base
    motors_.emplace_back(SparkMax(1, 52)); // Shoulder
    motors_.emplace_back(SparkMax(1, 53)); // Elbow
    // motors_.emplace_back(SparkMax(1, 54)); // Wrist 1
    // motors_.emplace_back(SparkMax(1, 55)); // Wrist 2
    // motors_.emplace_back(SparkMax(1, 56)); // Wrist 3

    RCLCPP_INFO(dl_logger, "ArmMotorManager: Testing Motors");
    for (auto &motor : motors_) {
        motor.ident();
    }
}



