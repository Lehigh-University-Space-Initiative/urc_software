#include "DriveTrainMotorManager.h"
#include "main.h"

DriveTrainMotorManager::~DriveTrainMotorManager()
{
}

void DriveTrainMotorManager::setupMotors() {
    // Add motors to the motors vector
    // Left side (BUS 0)
    motors_.emplace_back(node_, 0, 1, 1.0, true); // LF (idx=0)
    motors_.emplace_back(node_, 0, 2, 1.0, true); // LB (idx=1)
    motors_.emplace_back(node_, 0, 3, 1.0, true); // RF (idx=2)
    motors_.emplace_back(node_, 0, 4, 1.0, true); // RB (idx=3)
    motors_.emplace_back(node_, 0, 5, 1.0, true); // LSteer (idx=4)
    motors_.emplace_back(node_, 0, 6, 1.0, true); // RSteer (idx=5)

    motors_.emplace_back(node_, 0, 55, 1.0, true); // Base (linear actuator) (idx=6)
    motors_.emplace_back(node_, 0, 51, 1.0, true); // Shoulder (idx=7)
    motors_.emplace_back(node_, 0, 53, 1.0, true); // Elbow (idx=8)
    motors_.emplace_back(node_, 0, 52, 1.0, true); // Wrist Roll (idx=9)
    motors_.emplace_back(node_, 0, 57, 1.0, true); // Wrist Pitch (idx=10)
    motors_.emplace_back(node_, 0, 54, 1.0, true); // End Effector (idx=11)

    RCLCPP_INFO(node_->get_logger(), "Testing Motors");
    for (auto &motor : motors_) {
        motor.ident();
    }
}

void DriveTrainMotorManager::parseDriveCommands(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg) {
    motors_[0].sendPowerCMD((-msg->cmd_r.x) / 20); // Send left drive cmd
    motors_[1].sendPowerCMD((-msg->cmd_r.z) / 20); // Send left drive cmd
    motors_[2].sendPowerCMD((msg->cmd_r.x + msg->cmd_l.x) / 20); // Send right drive cmd
    motors_[3].sendPowerCMD((msg->cmd_r.z + msg->cmd_l.z) / 20); // Send right drive cmd

    motors_[4].sendPowerCMD(-msg->cmd_l.y); // Send left steer cmd
    motors_[5].sendPowerCMD(-(msg->cmd_l.y + msg->cmd_r.y)); // Send right steer cmd
    
}


    // Button mapping (Logitech Extreme 3D Pro):
    // Axis 4 (mini joystick left/right): Base (CAN 55) - CW/CCW
    // Buttons 6-7: Shoulder (CAN 51) - CW/CCW
    // Buttons 8-9: Elbow (CAN 53) - CW/CCW
    // Buttons 4-5: Wrist Roll (CAN 52) - CW/CCW
    // Buttons 2-3: Wrist Pitch (CAN 57) - CW/CCW
    // Buttons 0-1: End Effector (CAN 54) - Close/Open
void DriveTrainMotorManager::parseArmCommands(const cross_pkg_messages::msg::RoverComputerArmCMD::SharedPtr msg) {
    motors_[6].sendPowerCMD(msg->cmd_b); 
    motors_[7].sendPowerCMD(msg->cmd_s + msg->cmd_s_extra); 
    motors_[8].sendPowerCMD(msg->cmd_e + msg->cmd_e_extra); 
    motors_[9].sendPowerCMD(msg->cmd_wr); 
    motors_[10].sendPowerCMD(msg->cmd_wp); 
    motors_[11].sendPowerCMD(msg->cmd_endeff); 
}