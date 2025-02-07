#include "driveline_urc/DriveTrainMotorManager.h"
#include "driveline_urc/Logger.h"

DriveTrainMotorManager::DriveTrainMotorManager()
{
    setupMotors();

    // Resize vectors for position, velocity, command
    hw_positions_.resize(6, 0.0);
    hw_velocities_.resize(6, 0.0);
    hw_commands_.resize(6, 0.0);
}

DriveTrainMotorManager::~DriveTrainMotorManager()
{
}

void DriveTrainMotorManager::setupMotors()
{
    // Add motors to the motors vector
    // Left side (BUS 0)
    motors.emplace_back(SparkMax(1, 1)); // LF
    motors.emplace_back(SparkMax(1, 2)); // LM
    motors.emplace_back(SparkMax(1, 3)); // LB
    // Right side (BUS 1)
    motors.emplace_back(SparkMax(1, 4)); // RB
    motors.emplace_back(SparkMax(1, 5)); // RM
    motors.emplace_back(SparkMax(1, 6)); // RF

    RCLCPP_INFO(dl_logger, "DrivelineMotorManager: Testing Motors");
    for (auto &motor : motors) {
        motor.ident();
    }
}

void DriveTrainMotorManager::stopAllMotors()
{
    for (auto &motor : motors) {
        motor.motorLocked = true;
        motor.sendPowerCMD(0);
    }
}

void DriveTrainMotorManager::sendHeartbeats()
{
    for (auto &motor : motors) {
        motor.sendHeartbeat();
    }
}

void DriveTrainMotorManager::readMotors(const rclcpp::Duration period) {
    for (size_t i = 0; i < motors.size(); i++) {
        double velocity = motors[i].lastVelocityAsRadPerSec();
        hw_velocities_[i] = velocity;
        hw_positions_[i] += velocity * period.seconds();
    }
}

void DriveTrainMotorManager::readMotors() {
    for (size_t i = 0; i < motors.size(); i++) {
      // direct power
      motors[i].sendPowerCMD(hw_commands_[i]);
    }
}

size_t DriveTrainMotorManager::getMotorCount() { return motors.size(); }

std::vector<hardware_interface::StateInterface> DriveTrainMotorManager::getStateInterfaces(std::vector<hardware_interface::ComponentInfo>& joints) {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (size_t i = 0; i < motors.size(); i++) {
        interfaces.emplace_back(hardware_interface::StateInterface(
            joints[i].name, /*hardware_interface::HW_IF_POSITION*/ "position", &hw_positions_[i]));
        interfaces.emplace_back(hardware_interface::StateInterface(
            joints[i].name, /*hardware_interface::HW_IF_VELOCITY*/ "velocity", &hw_velocities_[i]));
    }
    return interfaces;
}

std::vector<hardware_interface::CommandInterface> DriveTrainMotorManager::getCommandInterface(std::vector<hardware_interface::ComponentInfo>& joints) {
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (size_t i = 0; i < motors.size(); i++) {
        interfaces.emplace_back(hardware_interface::CommandInterface(
        joints[i].name, /*hardware_interface::HW_IF_VELOCITY*/ "velocity", &hw_commands_[i]));
    }
    return interfaces;
}



void DriveTrainMotorManager::tick()
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
            RCLCPP_WARN(dl_logger, "DriveTrainMotorManager: LOS Safety Stop");
            stopAllMotors();
        }
    }

    //give pid tick
    for (auto &motor : motors) {
        motor.sendHeartbeat();
        motor.pidTick();
    }

    // publish wheel data
    cross_pkg_messages::msg::RoverComputerDriveCMD speedMsg;
    speedMsg.cmd_l.x = motors[0].lastVelocityAsRadPerSec();
    speedMsg.cmd_l.y = motors[1].lastVelocityAsRadPerSec();
    speedMsg.cmd_l.z = motors[2].lastVelocityAsRadPerSec();

    speedMsg.cmd_r.x = motors[3].lastVelocityAsRadPerSec();
    speedMsg.cmd_r.y = motors[4].lastVelocityAsRadPerSec();
    speedMsg.cmd_r.z = motors[5].lastVelocityAsRadPerSec();

    //wheelVelPub->publish(speedMsg);
}

void DriveTrainMotorManager::parseDriveCommands(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg)
{
    RCLCPP_INFO(dl_logger, "DriveTrainMotorManager: Drive Commands Received with L: %f, R: %f", msg->cmd_l.x, msg->cmd_r.x);

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

    
    motors[0].sendPowerCMD(-msg->cmd_l.x / 20);
    motors[1].sendPowerCMD(-msg->cmd_l.y / 20);
    motors[2].sendPowerCMD(-msg->cmd_l.z / 20);

    motors[3].sendPowerCMD(msg->cmd_r.x / 20);
    motors[4].sendPowerCMD(msg->cmd_r.y / 20);
    motors[5].sendPowerCMD(msg->cmd_r.z / 20);


}
