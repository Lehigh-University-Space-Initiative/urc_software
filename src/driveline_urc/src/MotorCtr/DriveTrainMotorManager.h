#pragma once

#include "CANDriver.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>
#include <vector>
#include <cs_plain_guarded.h>
#include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"


class DriveTrainMotorManager {
private:
    std::vector<SparkMax> motors;

    void setupMotors();
    void stopAllMotors();

    libguarded::plain_guarded<std::chrono::time_point<std::chrono::system_clock>> lastManualCommandTime{std::chrono::system_clock::now()};
    std::chrono::milliseconds manualCommandTimeout{1500};

    libguarded::plain_guarded<bool> sendHeartbeatsFlag{true};
    libguarded::plain_guarded<bool> heartbeatThreadShutDown{false};
    void heartbeatThread();
    std::thread heartbeatThreadObj;


    rclcpp::Subscription<cross_pkg_messages::msg::RoverComputerDriveCMD>::SharedPtr driveCommandsSub;

    void parseDriveCommands(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg);

public:
    DriveTrainMotorManager();
    virtual ~DriveTrainMotorManager();
    void sendHeartbeats();
    void tick();
};
