#pragma once

#include "StepperDriver.h"
#include "CANDriver.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>
#include <vector>
#include <cs_plain_guarded.h>
#include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"

class ArmMotorManager {
private:
    std::vector<StepperDriver> motors;

    void setupMotors();
    void stopAllMotors();

    libguarded::plain_guarded<std::chrono::time_point<std::chrono::system_clock>> lastManualCommandTime{std::chrono::system_clock::now()};
    std::chrono::milliseconds manualCommandTimeout{1500};

    rclcpp::Subscription<cross_pkg_messages::msg::RoverComputerDriveCMD>::SharedPtr armCommandsSub;
    rclcpp::Node::SharedPtr nodeHandle;

    void parseDriveCommands(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg);

    // dir, step, enable
    StepperDriver elbow_pitch = StepperDriver(5, 6, 22, 5.0, 200);
    StepperDriver shoulder_pitch = StepperDriver(21, 20, 22, 1.0, 200);
    StepperDriver wrist_yaw = StepperDriver(11, 4, 22, 1/100.0, 200);
    StepperDriver wrist_roll = StepperDriver(17, 27, 22, 1/30.0, 200);
    StepperDriver wrist_pitch = StepperDriver(10, 9, 22, 1/5.0, 200);

public:
    ArmMotorManager();
    virtual ~ArmMotorManager();

    void tick();
};
