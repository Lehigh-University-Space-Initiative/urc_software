/**
 * @file
 * Base class that owns and runs a group of SparkMax motors (shared by the driveline and the arm)
 *
 * How it connects to the system:
 *   - driveline_urc subclasses it as DriveTrainMotorManager (six wheel motors on CAN bus 0)
 *   - arm_urc subclasses it as ArmMotorManager (six arm joints plus the end effector on CAN bus 1)
 *   - Each node's main loop calls init() once, then tick() as fast as its loop runs
 *   - Subclasses decide which motors exist by overriding setupMotors()
 */
#pragma once

#include "CANDriver.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <cs_plain_guarded.h>
#include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"

/**
 * A group of SparkMax motors plus the shared bookkeeping to drive them
 *
 * Syntax: a class with a "= 0" (pure virtual) function is abstract, so it can't be created directly
 * Only subclasses that fill in setupMotors() can be created
 */
class MotorManager {
protected:
    // Syntax: protected members are visible to subclasses but not to outside code

    std::vector<SparkMax> motors_;  // Every motor this manager drives, in the subclass's chosen order
    // Syntax: std::unique_ptr owns one heap object and deletes it automatically; it stays null if unused
    std::unique_ptr<SparkMax> eef;  // End effector (gripper) motor, only used by the arm

    size_t motor_count_;  // Number of entries in motors_, set by init()
    std::vector<double> hw_positions_;  // Latest measured position of each motor (radians)
    std::vector<double> hw_velocities_;  // Latest measured velocity of each motor (rad/s)
    std::vector<double> hw_commands_;  // Latest commanded set point of each motor

    rclcpp::Node::SharedPtr node_ = nullptr;  // The ROS node that owns this manager

    bool usePid_ = false;  // Whether motors run closed-loop PID (true for the arm, false for the driveline)

private:
    /// Fill motors_ (and eef, if used) with this subclass's motors
    virtual void setupMotors() = 0;

    // Time the last manual command arrived, guarded so callbacks and the main loop can share it safely
    libguarded::plain_guarded<std::chrono::time_point<std::chrono::system_clock>> lastManualCommandTime{std::chrono::system_clock::now()};
    // How long without a command before a loss of signal (LOS) is declared
    std::chrono::milliseconds manualCommandTimeout{1500};

    /// Declared but never defined or called (subclasses parse their own commands)
    void parseDriveCommands(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg);

public:
    /**
     * Create a manager with no motors yet (call init() next)
     *
     * Parameters (inputs):
     *   node - the ROS node that owns the motors (it must declare kp, ki, kd, max_i, and readOnly)
     *   usePid - run every motor through its PID controller on each tick
     */
    MotorManager(rclcpp::Node::SharedPtr node, bool usePid);
    virtual ~MotorManager();

    /**
     * Create the motors and zero them
     *
     * This can't happen in the constructor, because the subclass's setupMotors() override isn't callable yet
     * (While the base constructor runs, the subclass part of the object doesn't exist)
     */
    void init();

    /// Send a heartbeat to every motor so the SparkMaxes keep their outputs enabled
    void sendHeartbeats();

    /// Run one control-loop step: read CAN, send heartbeats, run PIDs, check for loss of signal
    void tick();

    /**
     * Copy the latest velocity and position of each motor into hw_velocities_ and hw_positions_
     *
     * Parameters (inputs):
     *   period - unused
     */
    void readMotors(double period);

    /// Latest measured position of each motor, in radians
    std::vector<double>& getMotorPositions();

    /// Apply hw_commands_ to the motors (a no-op here; ArmMotorManager overrides it)
    virtual void writeMotors();

    /// Number of motors created by init()
    size_t getMotorCount();

    /// Currently a no-op (subclasses apply their own commands directly)
    void setCommands(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg);

    /// Mark that a fresh manual command arrived and unlock every motor
    void resetLOSTimeout();

    /// Lock every motor and command zero power
    void stopAllMotors();
};
