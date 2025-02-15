#include "arm_urc/MotorManager.h"
#include "arm_urc/main.h"

MotorManager::MotorManager()
{
//     setupMotors();
//     motor_count_ = motors_.size();

//     // Resize vectors for position, velocity, command
//     hw_positions_.resize(motorCount, 0.0);
//     hw_velocities_.resize(motorCount, 0.0);
//     hw_commands_.resize(motorCount, 0.0);
  
//   {
//     auto lock = lastManualCommandTime.lock();
//     *lock = std::chrono::system_clock::now();
//   }
}

MotorManager::~MotorManager()
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

    RCLCPP_INFO(node->get_logger(), "MotorManager: Testing Motors");
    for (auto &motor : motors_) {
        motor.ident();
    }
}

void MotorManager::stopAllMotors()
{
    for (auto &motor : motors_) {
        motor.motorLocked = true;
        motor.sendPowerCMD(0);
    }
}

void MotorManager::sendHeartbeats()
{
    for (auto &motor : motors_) {
        motor.sendHeartbeat();
    }
}

void MotorManager::readMotors(const rclcpp::Duration period) {
    for (size_t i = 0; i < motor_count_; i++) {
        double velocity = motors_[i].lastVelocityAsRadPerSec();
        hw_velocities_[i] = velocity;
        hw_positions_[i] += velocity * period.seconds();
    }
}

void MotorManager::writeMotors() {
    for (size_t i = 0; i < motor_count_; i++) {
      // direct power
      motors_[i].sendPowerCMD(hw_commands_[i]);
    }
}

size_t MotorManager::getMotorCount() { return motor_count_; }

void MotorManager::tick()
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
}


void MotorManager::setCommands(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), "MotorManager: Drive Commands Received with L: %f, R: %f", msg->cmd_l.x, msg->cmd_r.x);

    // for (auto m : motors_) {
    //     m.motorLocked = false;
    // }

    // Send power commands to motors based on drive command message
    // motors[0].setPIDSetpoint(-msg->cmd_l.x);
    // motors[1].setPIDSetpoint(-msg->cmd_l.y);
    // motors[2].setPIDSetpoint(-msg->cmd_l.z);

    // motors[3].setPIDSetpoint(msg->cmd_r.x);
    // motors[4].setPIDSetpoint(msg->cmd_r.y);
    // motors[5].setPIDSetpoint(msg->cmd_r.z);

    
    motors_[0].sendPowerCMD(-msg->cmd_l.x / 20);
    motors_[1].sendPowerCMD(-msg->cmd_l.y / 20);
    motors_[2].sendPowerCMD(-msg->cmd_l.z / 20);

    motors_[3].sendPowerCMD(msg->cmd_r.x / 20);
    motors_[4].sendPowerCMD(msg->cmd_r.y / 20);
    motors_[5].sendPowerCMD(msg->cmd_r.z / 20);


    resetLOSTimeout();
}


// Public API to reset the LOS timeout (e.g. call this from another context if needed).
void MotorManager::resetLOSTimeout()
{
  auto lock = lastManualCommandTime.lock();
  *lock = std::chrono::system_clock::now();
}

