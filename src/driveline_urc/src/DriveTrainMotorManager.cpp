#include "DriveTrainMotorManager.h"
#include "main.h"

DriveTrainMotorManager::DriveTrainMotorManager()
{
  setupMotors();
  {
    auto lock = lastManualCommandTime.lock();
    *lock = std::chrono::system_clock::now();
  }
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

    RCLCPP_INFO(dl_logger, "Testing Motors");
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

std::vector<SparkMax>& DriveTrainMotorManager::getMotors() {
    return motors;
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
            RCLCPP_WARN(dl_logger, "LOS Safety Stop");
            stopAllMotors();
        }
    }

    //give pid tick
    for (auto &motor : motors) {
        motor.sendHeartbeat();
        motor.pidTick();
    }
}


void DriveTrainMotorManager::setCommands(const std::vector<double> & commands)
{
  if (commands.size() != motors.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("DriveTrainMotorManager"),
                 "Invalid number of commands: expected %zu, got %zu",
                 motors.size(), commands.size());
    return;
  }


  resetLOSTimeout();


  motors[0].sendPowerCMD(-commands[0] / 20.0);
  motors[1].sendPowerCMD(-commands[1] / 20.0);
  motors[2].sendPowerCMD(-commands[2] / 20.0);
  motors[3].sendPowerCMD( commands[3] / 20.0);
  motors[4].sendPowerCMD( commands[4] / 20.0);
  motors[5].sendPowerCMD( commands[5] / 20.0);
}


// Public API to reset the LOS timeout (e.g. call this from another context if needed).
void DriveTrainMotorManager::resetLOSTimeout()
{
  auto lock = lastManualCommandTime.lock();
  *lock = std::chrono::system_clock::now();
}

