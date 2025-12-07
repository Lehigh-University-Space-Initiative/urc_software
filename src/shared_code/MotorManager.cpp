#include "MotorManager.h"
#include "Logger.h"

// rclcpp::Logger dl_logger = rclcpp::get_logger("driveline logger");

MotorManager::MotorManager(rclcpp::Node::SharedPtr node, bool usePid)
{
    this->node_ = node;
    this->usePid_ = usePid;
}

MotorManager::~MotorManager()
{
}


void MotorManager::testMotors() {
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

void MotorManager::init()
{
    setupMotors();
    motor_count_ = motors_.size();

    for(auto& m : motors_) {
        m.pidControlled = usePid_;
    }

    // Resize vectors for position, velocity, command
    hw_positions_.resize(motor_count_, 0.0);
    hw_velocities_.resize(motor_count_, 0.0);
    hw_commands_.resize(motor_count_, 0.0);
  
  {
    auto lock = lastManualCommandTime.lock();
    *lock = std::chrono::system_clock::now();
  }

  for(auto& motor: motors_) {
    motor.sendPowerCMD(0);
  }
}

void MotorManager::sendHeartbeats()
{
    for (auto &motor : motors_) {
        motor.sendHeartbeat();
    }
}

void MotorManager::readMotors(double period) {
    for (size_t i = 0; i < motor_count_; i++) {
        double velocity = motors_[i].lastVelocityAsRadPerSec();
        double position = motors_[i].lastCorrectPos();
        hw_velocities_[i] = velocity;
        hw_positions_[i] = position;
        //RCLCPP_INFO(dl_logger, "MotorManager: motor %ld vel: %.10f, pos: %.10f",i,hw_velocities_[i],hw_positions_[i]);
    }
}

std::vector<double>& MotorManager::getMotorPositions()
{
    return hw_positions_;
}

void MotorManager::writeMotors() {
    for (size_t i = 0; i < motor_count_; i++) {
      // direct power

      //motors_[i].sendPowerCMD(hw_commands_[i]);
    //   printf("Going to send power: %d to motor %d",hw_commands_[i],i);
    }
}

size_t MotorManager::getMotorCount() { return motor_count_; }

// std::vector<hardware_interface::StateInterface> MotorManager::getStateInterfaces(std::vector<hardware_interface::ComponentInfo>& joints) {
//     std::vector<hardware_interface::StateInterface> interfaces;
//     for (size_t i = 0; i < motor_count_; i++) {
//         interfaces.emplace_back(hardware_interface::StateInterface(
//             joints[i].name, /*hardware_interface::HW_IF_POSITION*/ "position", &hw_positions_[i]));
//         interfaces.emplace_back(hardware_interface::StateInterface(
//             joints[i].name, /*hardware_interface::HW_IF_VELOCITY*/ "velocity", &hw_velocities_[i]));
//     }
//     return interfaces;
// }

// std::vector<hardware_interface::CommandInterface> MotorManager::getCommandInterface(std::vector<hardware_interface::ComponentInfo>& joints) {
//     std::vector<hardware_interface::CommandInterface> interfaces;
//     for (size_t i = 0; i < motor_count_; i++) {
//         interfaces.emplace_back(hardware_interface::CommandInterface(
//         joints[i].name, /*hardware_interface::HW_IF_VELOCITY*/ "velocity", &hw_commands_[i]));
//     }
//     return interfaces;
// }



void MotorManager::tick()
{

    static uint64_t loopItr = 0;
    loopItr++;

    //read any can message at a much higher rate than all other update tasks
    //TODO: URC-102: should move this back 
    size_t canItr = 0;
    while(CANDriver::doCanReadIter(1)) {
        canItr++;
    };
    // RCLCPP_INFO(rclcpp::get_logger("Arm"), "can ITR count: %ld", canItr);
    

    //if (loopItr % 0 != 0) return;

            {
            static std::chrono::system_clock::time_point last_update;
            double delta = std::chrono::duration<double>(std::chrono::system_clock::now() - last_update).count();
            last_update = std::chrono::system_clock::now();
            // RCLCPP_INFO(rclcpp::get_logger("Arm"), "pid tick cycle delta: %f", delta);
            }
    
    //give pid tick
    for (auto i = 0; i < motors_.size(); i++) {
        auto& motor = motors_[i];
        motor.sendHeartbeat();
        motor.pidTick(hw_positions_[i]);
    }
    // if (eef) {
    //     eef->sendHeartbeat();
    // }

    {  // lock block for LOS safety stop
        auto lock = lastManualCommandTime.lock();
        auto now = std::chrono::system_clock::now();
        if (now - *lock > manualCommandTimeout) {
            RCLCPP_WARN(dl_logger, "MotorManager: LOS Safety Stop WARNING: DISABLED");
            stopAllMotors();
            // if (eef) {
            //     eef->sendPowerCMD(0);
            // }
        }
    }


}

// Public API to reset the LOS timeout (e.g. call this from another context if needed).
void MotorManager::resetLOSTimeout()
{
  auto lock = lastManualCommandTime.lock();
  *lock = std::chrono::system_clock::now();
  
    for (auto i = 0; i < motors_.size(); i++) {
        auto& motor = motors_[i];
        motor.motorLocked = false;
    }
}

