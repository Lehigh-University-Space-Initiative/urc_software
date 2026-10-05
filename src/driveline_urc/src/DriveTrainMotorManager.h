/**
 * @file
 * The driveline's motor group: three wheel motors per side on CAN bus 0
 */
#pragma once

#include "CANDriver.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>
#include <vector>
#include <cs_plain_guarded.h>
#include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"
#include "MotorManager.h"

/**
 * Drives the six wheel motors from RoverComputerDriveCMD messages
 *
 * Syntax: ": public MotorManager" means DriveTrainMotorManager is a MotorManager and inherits its methods
 */
class DriveTrainMotorManager : public MotorManager {
private:
    /// Create the six wheel motors (CAN IDs 1-6 on bus 0)
    void setupMotors() override;

    // Unused copies of the base-class LOS timer (the base class's own copies are the ones tick() checks)
    libguarded::plain_guarded<std::chrono::time_point<std::chrono::system_clock>> lastManualCommandTime{std::chrono::system_clock::now()};
    std::chrono::milliseconds manualCommandTimeout{1500};

    // Declared but never created; main.cpp owns the real subscription
    rclcpp::Subscription<cross_pkg_messages::msg::RoverComputerDriveCMD>::SharedPtr driveCommandsSub;

    // Declared but never created; the GUI's TelemetryPanel listens for wheel speeds on "motorVels"
    rclcpp::Publisher<cross_pkg_messages::msg::RoverComputerDriveCMD>::SharedPtr wheelVelPub;

public:
    // Syntax: "using MotorManager::MotorManager" reuses the base class's constructor as-is
    using MotorManager::MotorManager;
    virtual ~DriveTrainMotorManager();

    /**
     * Send one drive command to the six wheel motors
     *
     * Parameters (inputs):
     *   msg - per-wheel speeds; cmd_l/cmd_r components are the front/middle/back wheel on each side
     */
    void parseDriveCommands(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg);
};
