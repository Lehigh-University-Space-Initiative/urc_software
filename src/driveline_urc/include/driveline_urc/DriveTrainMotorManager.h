#pragma once

#include "driveline_urc/CANDriver.h"
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
    
    libguarded::plain_guarded<std::chrono::time_point<std::chrono::system_clock>> lastManualCommandTime{std::chrono::system_clock::now()};
    std::chrono::milliseconds manualCommandTimeout{1500};

public:
    DriveTrainMotorManager();
    virtual ~DriveTrainMotorManager();
    void sendHeartbeats();
    void tick();
    std::vector<SparkMax>& getMotors();
    void setCommands(const std::vector<double>& commands);
    void resetLOSTimeout();
    void stopAllMotors();
};
