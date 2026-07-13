#include "MotorManager.h"
#include "Logger.h"

rclcpp::Logger dl_logger = rclcpp::get_logger("driveline logger");

MotorManager::MotorManager(rclcpp::Node::SharedPtr node, bool usePid)
{
    this->node_ = node;
    this->usePid_ = usePid;
}

MotorManager::~MotorManager()
{
}

// Base implementation; subclasses (DriveTrainMotorManager, ArmMotorManager)
// override this to populate motors_ with their specific motor layout.
void MotorManager::setupMotors()
{
    RCLCPP_INFO(dl_logger, "MotorManager: Testing Motors");
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

    for (auto &m : motors_) {
        m.pidControlled = usePid_;
    }

    // Size the position/velocity/command buffers to the motor count.
    hw_positions_.resize(motor_count_, 0.0);
    hw_velocities_.resize(motor_count_, 0.0);
    hw_commands_.resize(motor_count_, 0.0);

    {
        auto lock = lastManualCommandTime.lock();
        *lock = std::chrono::system_clock::now();
    }

    for (auto &motor : motors_) {
        motor.sendPowerCMD(0);
    }
}

void MotorManager::sendHeartbeats()
{
    for (auto &motor : motors_) {
        motor.sendHeartbeat();
    }
}

void MotorManager::readMotors(double period)
{
    for (size_t i = 0; i < motor_count_; i++) {
        hw_velocities_[i] = motors_[i].lastVelocityAsRadPerSec();
        hw_positions_[i] = motors_[i].lastCorrectPos();
    }
}

std::vector<double> &MotorManager::getMotorPositions()
{
    return hw_positions_;
}

// Write path is currently disabled: power is commanded directly from the
// per-manager command parsing (see DriveTrainMotorManager::parseDriveCommands),
// not through hw_commands_.
void MotorManager::writeMotors()
{
}

size_t MotorManager::getMotorCount() { return motor_count_; }

void MotorManager::tick()
{
    static uint64_t loopItr = 0;
    loopItr++;

    // Drain any pending CAN messages. Reads run at a much higher rate than the
    // other periodic tasks below.
    // TODO: URC-102: should move this back
    size_t canItr = 0;
    while (CANDriver::doCanReadIter(1)) {
        canItr++;
    }
    RCLCPP_INFO(rclcpp::get_logger("Arm"), "can ITR count: %ld", canItr);

    {
        static std::chrono::system_clock::time_point last_update;
        double delta = std::chrono::duration<double>(std::chrono::system_clock::now() - last_update).count();
        last_update = std::chrono::system_clock::now();
        RCLCPP_INFO(rclcpp::get_logger("Arm"), "pid tick cycle delta: %f", delta);
    }

    // Heartbeat + PID tick every motor.
    for (auto i = 0; i < motors_.size(); i++) {
        auto &motor = motors_[i];
        motor.sendHeartbeat();
        motor.pidTick(hw_positions_[i]);
    }
    if (eef) {
        eef->sendHeartbeat();
    }

    {  // Loss-of-signal safety stop (currently only stops the end effector).
        auto lock = lastManualCommandTime.lock();
        auto now = std::chrono::system_clock::now();
        if (now - *lock > manualCommandTimeout) {
            RCLCPP_WARN(dl_logger, "MotorManager: LOS Safety Stop WARNING: DISABLED");
            // stopAllMotors();
            if (eef) {
                eef->sendPowerCMD(0);
            }
        }
    }
}

// Currently a no-op: commands are applied directly by the subclass command
// callbacks rather than buffered here.
void MotorManager::setCommands(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg)
{
}

// Reset the loss-of-signal timeout and unlock all motors. Call this whenever a
// fresh manual command arrives.
void MotorManager::resetLOSTimeout()
{
    auto lock = lastManualCommandTime.lock();
    *lock = std::chrono::system_clock::now();

    for (auto i = 0; i < motors_.size(); i++) {
        motors_[i].motorLocked = false;
    }
}
