#pragma once

#include "CANDriver.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>
#include <vector>
#include <cs_plain_guarded.h>
#include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"
#include "MotorManager.h"

class DriveTrainMotorManager : public MotorManager {
private:
    std::vector<SparkMax> motors;

    void setupMotors() override;

    libguarded::plain_guarded<std::chrono::time_point<std::chrono::system_clock>> lastManualCommandTime{std::chrono::system_clock::now()};
    std::chrono::milliseconds manualCommandTimeout{1500};


    rclcpp::Subscription<cross_pkg_messages::msg::RoverComputerDriveCMD>::SharedPtr driveCommandsSub;

    rclcpp::Publisher<cross_pkg_messages::msg::RoverComputerDriveCMD>::SharedPtr wheelVelPub;


public:
    using MotorManager::MotorManager;
    virtual ~DriveTrainMotorManager();

    void init();

    void parseDriveCommands(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg);
};
