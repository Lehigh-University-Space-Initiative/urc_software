#include "ArmMotorManager.h"
#include "Logger.h"

ArmMotorManager::~ArmMotorManager()
{
}

void ArmMotorManager::writeMotors() {
    for (size_t i = 0; i < motor_count_; i++) {
    //   printf("Going to send power: %.2f to motor %ld\n",hw_commands_[i],i);
        motors_[i].setPIDSetpoint(hw_commands_[i]);
    }
}

void ArmMotorManager::setArmCommand(const cross_pkg_messages::msg::RoverComputerArmCMD::SharedPtr msg)
{
    //TODO add the rest
    hw_commands_[0] = msg->cmd_b;
    hw_commands_[1] = msg->cmd_s;
    hw_commands_[2] = msg->cmd_e;
    hw_commands_[3] = msg->cmd_w.x;
    hw_commands_[4] = msg->cmd_w.y;
    hw_commands_[5] = msg->cmd_w.z;
    // TODO test
    // hw_commands_[3] = msg->cmd_s;

    writeMotors();
}

void ArmMotorManager::setArmCommand(const cross_pkg_messages::msg::ArmInputRaw::SharedPtr msg)
{

    const double power = 0.07;

    if (msg->left_btn) {
        eef->sendPowerCMD(power);
        RCLCPP_INFO(dl_logger, "ArmMotorManager: EEF Open");
    } else if (msg->right_btn) {
        eef->sendPowerCMD(-power);
        RCLCPP_INFO(dl_logger, "ArmMotorManager: EEF Close");
    } else {
        eef->sendPowerCMD(0);
    }
}

void ArmMotorManager::setupMotors()
{
    RCLCPP_INFO(rclcpp::get_logger("VVVVVVV"), "node: %p",node_.get());
    // Add motors to the mo_tors vector
    motors_.emplace_back(node_, 1, 51, 125, true); // Base
    motors_.emplace_back(node_, 1, 52, 125, true); // Shoulder
    motors_.emplace_back(node_, 1, 53, 125, true); // Elbow
    // motors_.emplace_back(1, 55, 12); // Test
    motors_.emplace_back(node_, 1, 54, 169.836, false); // Wrist 1
    motors_.emplace_back(node_, 1, 55, 169.836, false); // Wrist 2
    motors_.emplace_back(node_, 1, 56, 169.836, false); // Wrist 3

    eef = std::make_unique<SparkMax>(node_, 1, 57, 169.836, false);

    RCLCPP_INFO(dl_logger, "ArmMotorManager: Testing Motors");
    for (auto &motor : motors_) {
        motor.ident();
    }
    // motors_[3].sendPowerCMD(0.1);
}



