#pragma once

#include "arm_urc/CANDriver.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>
#include <vector>
#include <cs_plain_guarded.h>
#include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"


class MotorManager {
protected:
    std::vector<SparkMax> motors_;
    size_t motor_count_;

    libguarded::plain_guarded<std::chrono::time_point<std::chrono::system_clock>> lastManualCommandTime{std::chrono::system_clock::now()};
    std::chrono::milliseconds manualCommandTimeout{1500};

    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_commands_;
    
private:

    virtual void setupMotors() = 0;

public:
    MotorManager();
    virtual ~MotorManager();
    void sendHeartbeats();
    virtual void tick();
    void readMotors(const rclcpp::Duration period);
    void readMotors();
    void writeMotors();
    size_t getMotorCount();
    void setCommands(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg);
    void resetLOSTimeout();
    void stopAllMotors();
};
