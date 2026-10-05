/**
 * @file
 * Implementation of ArmMotorManager (the arm's joint and gripper motors)
 */
#include "ArmMotorManager.h"
#include "Logger.h"

ArmMotorManager::~ArmMotorManager()
{
}

void ArmMotorManager::writeMotors()
{
    for (size_t i = 0; i < motor_count_; i++) {
        motors_[i].setPIDSetpoint(hw_commands_[i]);
    }
}

void ArmMotorManager::setArmCommand(const cross_pkg_messages::msg::RoverComputerArmCMD::SharedPtr msg)
{
    // TODO: add the rest
    hw_commands_[0] = msg->cmd_b;  // Base
    hw_commands_[1] = msg->cmd_s;  // Shoulder
    hw_commands_[2] = msg->cmd_e;  // Elbow
    hw_commands_[3] = msg->cmd_w.x;  // Wrist 1
    hw_commands_[4] = msg->cmd_w.y;  // Wrist 2
    hw_commands_[5] = msg->cmd_w.z;  // Wrist 3

    writeMotors();
}

void ArmMotorManager::setArmCommand(const cross_pkg_messages::msg::ArmInputRaw::SharedPtr msg)
{
    const double power = 0.07;  // Gripper power while a button is held (7% of full power)

    // Syntax: a float64 field used as a condition is true for any nonzero value
    if (msg->left_btn) {
        eef->sendPowerCMD(power);
        RCLCPP_INFO(dl_logger, "ArmMotorManager: EEF Open");
    }
    else if (msg->right_btn) {
        eef->sendPowerCMD(-power);
        RCLCPP_INFO(dl_logger, "ArmMotorManager: EEF Close");
    }
    else {
        eef->sendPowerCMD(0);
    }
}

/**
 * Steps:
 *   1. Create the six joint motors on CAN bus 1; the base/shoulder/elbow use absolute encoders
 *   2. Create the gripper motor (stored in eef rather than motors_, so it isn't PID-controlled)
 *   3. Send each joint an ident message so a person can confirm wiring by the blinking lights
 */
void ArmMotorManager::setupMotors()
{
    RCLCPP_INFO(rclcpp::get_logger("VVVVVVV"), "node: %p", node_.get());

    // Arguments: (node, CAN bus, CAN ID, gear ratio, use absolute encoder)
    motors_.emplace_back(node_, 1, 51, 125, true);  // Base
    motors_.emplace_back(node_, 1, 52, 125, true);  // Shoulder
    motors_.emplace_back(node_, 1, 53, 125, true);  // Elbow
    motors_.emplace_back(node_, 1, 54, 169.836, false);  // Wrist 1
    motors_.emplace_back(node_, 1, 55, 169.836, false);  // Wrist 2
    motors_.emplace_back(node_, 1, 56, 169.836, false);  // Wrist 3

    eef = std::make_unique<SparkMax>(node_, 1, 57, 169.836, false);

    RCLCPP_INFO(dl_logger, "ArmMotorManager: Testing Motors");
    for (auto& motor : motors_) {
        motor.ident();
    }
}
