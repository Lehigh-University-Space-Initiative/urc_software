#pragma once

#include "driveline_urc/CANDriver.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>
#include <vector>
#include <cs_plain_guarded.h>
#include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"
#include <hardware_interface/system_interface.hpp>


class DriveTrainMotorManager {
private:
    std::vector<SparkMax> motors;

    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_commands_;

    void setupMotors();
    
    libguarded::plain_guarded<std::chrono::time_point<std::chrono::system_clock>> lastManualCommandTime{std::chrono::system_clock::now()};
    std::chrono::milliseconds manualCommandTimeout{1500};


    //rclcpp::Subscription<cross_pkg_messages::msg::RoverComputerDriveCMD>::SharedPtr driveCommandsSub;

    // rclcpp::Publisher<cross_pkg_messages::msg::RoverComputerDriveCMD>::SharedPtr wheelVelPub;

    void parseDriveCommands(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg);

public:
    DriveTrainMotorManager();
    virtual ~DriveTrainMotorManager();
    void sendHeartbeats();
    void tick();
    void readMotors(const rclcpp::Duration period);
    void readMotors();
    void writeMotors();
    size_t getMotorCount();
    std::vector<hardware_interface::StateInterface> getStateInterfaces(std::vector<hardware_interface::ComponentInfo>& joints);
    std::vector<hardware_interface::CommandInterface> getCommandInterface(std::vector<hardware_interface::ComponentInfo>& joints);
    void setCommands(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg);
    void resetLOSTimeout();
    void stopAllMotors();
};
