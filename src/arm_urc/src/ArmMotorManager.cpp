#include "arm_urc/ArmMotorManager.h"
#include "arm_urc/main.h"

ArmMotorManager::ArmMotorManager()
{ 

    setupMotors();
    motor_count_ = motors_.size();

    // Resize vectors for position, velocity, command
    hw_positions_.resize(motor_count_, 0.0);
    hw_velocities_.resize(motor_count_, 0.0);
    hw_commands_.resize(motor_count_, 0.0);
  
  {
    auto lock = lastManualCommandTime.lock();
    *lock = std::chrono::system_clock::now();
  }
    // publish arm velocity
    armVelPub = node->create_publisher<cross_pkg_messages::msg::RoverComputerArmCMD>("/armMotorVels", 10);

    // Subscribe to arm commands
    armCommandsSub = node->create_subscription<cross_pkg_messages::msg::RoverComputerArmCMD>(
        "/roverArmCommands", 10, [this](const cross_pkg_messages::msg::RoverComputerArmCMD::SharedPtr msg) {
            auto lock = lastManualCommandTime.lock();
            *lock = std::chrono::system_clock::now();
            parseCommands(msg);
        });
}

ArmMotorManager::~ArmMotorManager()
{
}


void ArmMotorManager::setupMotors()
{
    // Add motors to the motors vector
    // Left side (BUS 0)
    motors_.emplace_back(SparkMax(1, 51)); // LF
    motors_.emplace_back(SparkMax(1, 52)); // LM
    motors_.emplace_back(SparkMax(1, 53)); // LB
    // Right side (BUS 1)
    motors_.emplace_back(SparkMax(1, 54)); // RB
    motors_.emplace_back(SparkMax(1, 55)); // RM
    motors_.emplace_back(SparkMax(1, 56)); // RF

    RCLCPP_INFO(node->get_logger(), "ArmMotorManager: Testing Motors");
    for (auto &motor : motors_) {
        motor.ident();
    }
}

void ArmMotorManager::tick()
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
            RCLCPP_WARN(node->get_logger(), "MotorManager: LOS Safety Stop");
            stopAllMotors();
        }
    }

    //give pid tick
    for (auto &motor : motors_) {
        motor.sendHeartbeat();
        motor.pidTick();
    }

    // publish arm data
    cross_pkg_messages::msg::RoverComputerArmCMD speedMsg;
    speedMsg.cmd_b = motors_[0].lastVelocityAsRadPerSec();
    speedMsg.cmd_s = motors_[1].lastVelocityAsRadPerSec();
    speedMsg.cmd_e = motors_[2].lastVelocityAsRadPerSec();

    speedMsg.cmd_w.x = motors_[3].lastVelocityAsRadPerSec();
    speedMsg.cmd_w.y = motors_[4].lastVelocityAsRadPerSec();
    speedMsg.cmd_w.z = motors_[5].lastVelocityAsRadPerSec();

    armVelPub->publish(speedMsg);
}

// void sendArmPowers() {
//     armVelPub->publish(currentArmCommand);
// }

// void manualInputCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {

//     // //kinimatic data (metric system)
//     // const float roverWidth = 0.5; //meters
//     // const float wheelRadius = 0.1143; //meters

//     // breakdown commanded lin and ang vel into left and right side vels
//     // float leftSideVel = msg->linear.x + msg->angular.y * 3.14 / 180 * roverWidth;
//     // float rightSideVel = msg->linear.x - msg->angular.y * 3.14 / 180 * roverWidth;


//     // float leftSideAlpha = leftSideVel / wheelRadius;
//     // float rightSideAlpha = rightSideVel / wheelRadius;


//     // TODO: make these actually do something, do some inverse kinematics stuff
//     float baseAlpha = 0.0;
//     float shoulderAlpha = 0.0;
//     float elbowAlpha = 0.0;
//     float wristRollAlpha = 0.0;
//     float wristPitchAlpha = 0.0;
//     float wristYawAlpha = 0.0;


//     currentArmCommand.cmd_b = baseAlpha;
//     currentArmCommand.cmd_s = shoulderAlpha;
//     currentArmCommand.cmd_e = elbowAlpha;
//     currentArmCommand.cmd_w.x = wristRollAlpha;
//     currentArmCommand.cmd_w.y = wristPitchAlpha;
//     currentArmCommand.cmd_w.z = wristYawAlpha;

//     sendArmPowers();
// }

void ArmMotorManager::parseCommands(const cross_pkg_messages::msg::RoverComputerArmCMD::SharedPtr msg)
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

    
    motors_[0].sendPowerCMD(-msg->cmd_b / 20);
    motors_[1].sendPowerCMD(-msg->cmd_s / 20);
    motors_[2].sendPowerCMD(-msg->cmd_e / 20);

    motors_[3].sendPowerCMD(msg->cmd_w.x / 20);
    motors_[4].sendPowerCMD(msg->cmd_w.y / 20);
    motors_[5].sendPowerCMD(msg->cmd_w.z / 20);


}





