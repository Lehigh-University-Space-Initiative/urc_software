/**
 * @file
 * Implementation of MotorManager, the shared motor-group logic used by the driveline and the arm
 *
 * Logging note:
 *   - The per-tick timing logs here use the "MotorManager" logger at DEBUG level
 *   - That keeps the driveline readable, since its loop runs at up to 30 kHz
 *   - arm_launch.py turns them back on with --log-level MotorManager:=debug for arm tuning
 */
#include "MotorManager.h"
#include "Logger.h"

// Syntax: this defines the logger that Logger.h declares with "extern", so every file shares one object
rclcpp::Logger dl_logger = rclcpp::get_logger("driveline logger");

MotorManager::MotorManager(rclcpp::Node::SharedPtr node, bool usePid)
{
    this->node_ = node;
    this->usePid_ = usePid;
}

MotorManager::~MotorManager()
{
}

// Base implementation; subclasses (DriveTrainMotorManager, ArmMotorManager) override this to create their own motors
void MotorManager::setupMotors()
{
    RCLCPP_INFO(dl_logger, "MotorManager: Testing Motors");
    for (auto& motor : motors_) {
        motor.ident();
    }
}

void MotorManager::stopAllMotors()
{
    for (auto& motor : motors_) {
        motor.motorLocked = true;
        motor.sendPowerCMD(0);
    }
}

/**
 * Steps:
 *   1. Ask the subclass to create its motors
 *   2. Turn PID on or off for every motor, and size the position/velocity/command buffers
 *   3. Start the loss-of-signal timer from now and command zero power to every motor
 */
void MotorManager::init()
{
    setupMotors();
    motor_count_ = motors_.size();

    // Syntax: "for (auto& m : motors_)" loops over every element by reference, so changes stick
    for (auto& m : motors_) {
        m.pidControlled = usePid_;
    }

    hw_positions_.resize(motor_count_, 0.0);
    hw_velocities_.resize(motor_count_, 0.0);
    hw_commands_.resize(motor_count_, 0.0);

    {
        // Syntax: "*lock" reaches the guarded value; the braces end the lock's scope, releasing the mutex
        auto lock = lastManualCommandTime.lock();
        *lock = std::chrono::system_clock::now();
    }

    for (auto& motor : motors_) {
        motor.sendPowerCMD(0);
    }
}

void MotorManager::sendHeartbeats()
{
    for (auto& motor : motors_) {
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

std::vector<double>& MotorManager::getMotorPositions()
{
    return hw_positions_;
}

// Write path is currently disabled (subclasses command power directly, e.g. DriveTrainMotorManager::parseDriveCommands)
void MotorManager::writeMotors()
{
}

size_t MotorManager::getMotorCount() { return motor_count_; }

/**
 * Steps:
 *   1. Drain every CAN frame waiting on bus 1, updating each motor's latest encoder readings
 *   2. Send a heartbeat to every motor (and the end effector) and run each motor's PID step
 *   3. If no manual command has arrived recently, warn about loss of signal (LOS)
 *
 * Note on the hardcoded bus:
 *   - The CAN read only checks bus 1, which is the arm's bus
 *   - On the driveline (bus 0) it reads nothing, since the driveline never opens bus 1
 *   - Driveline encoder feedback isn't used yet, so nothing depends on it
 */
void MotorManager::tick()
{
    static uint64_t loopItr = 0;  // Syntax: a static local keeps its value between calls
    loopItr++;

    // TODO: URC-102: should move this back
    size_t canItr = 0;
    while (CANDriver::doCanReadIter(1)) {
        canItr++;
    }
    RCLCPP_DEBUG(rclcpp::get_logger("MotorManager"), "can ITR count: %ld", canItr);

    {
        static std::chrono::system_clock::time_point last_update;
        double delta = std::chrono::duration<double>(std::chrono::system_clock::now() - last_update).count();
        last_update = std::chrono::system_clock::now();
        RCLCPP_DEBUG(rclcpp::get_logger("MotorManager"), "pid tick cycle delta: %f", delta);
    }

    for (auto i = 0; i < motors_.size(); i++) {
        auto& motor = motors_[i];
        motor.sendHeartbeat();
        motor.pidTick(hw_positions_[i]);
    }
    if (eef) {
        eef->sendHeartbeat();
    }

    {  // Loss-of-signal safety stop (stopping all motors is disabled; only the end effector stops)
        auto lock = lastManualCommandTime.lock();
        auto now = std::chrono::system_clock::now();
        if ((now - *lock) > manualCommandTimeout) {
            // Throttled to once every 5 s (this runs every tick, up to 30 kHz on the driveline)
            RCLCPP_WARN_THROTTLE(dl_logger, *node_->get_clock(), 5000, "MotorManager: LOS Safety Stop WARNING: DISABLED");
            if (eef) {
                eef->sendPowerCMD(0);
            }
        }
    }
}

// Currently a no-op (subclass command callbacks apply commands directly instead of buffering them here)
void MotorManager::setCommands(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg)
{
}

// Reset the loss-of-signal timeout and unlock all motors; call this whenever a fresh manual command arrives
void MotorManager::resetLOSTimeout()
{
    auto lock = lastManualCommandTime.lock();
    *lock = std::chrono::system_clock::now();

    for (auto i = 0; i < motors_.size(); i++) {
        motors_[i].motorLocked = false;
    }
}
