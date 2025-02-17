#pragma once

#include "CANDriver.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>
#include <vector>
#include <cs_plain_guarded.h>
#include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"
// #include <hardware_interface/system_interface.hpp>


class MotorManager {
protected:
    std::vector<SparkMax> motors_;
    size_t motor_count_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_commands_;

    rclcpp::Node::SharedPtr node_ = nullptr;
private:

    virtual void setupMotors() = 0;
    
    libguarded::plain_guarded<std::chrono::time_point<std::chrono::system_clock>> lastManualCommandTime{std::chrono::system_clock::now()};
    std::chrono::milliseconds manualCommandTimeout{1500};

    void parseDriveCommands(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg);

public:
    MotorManager(rclcpp::Node::SharedPtr node);
    virtual ~MotorManager();

    //can't call virtual fucntion of setupMotors from constructor so setup needs to happen here
    void init();

    void sendHeartbeats();
    void tick();
    void readMotors(double period);
    std::vector<double>& getMotorPositions();
    virtual void writeMotors();
    size_t getMotorCount();
    // std::vector<hardware_interface::StateInterface> getStateInterfaces(std::vector<hardware_interface::ComponentInfo>& joints);
    // std::vector<hardware_interface::CommandInterface> getCommandInterface(std::vector<hardware_interface::ComponentInfo>& joints);
    void setCommands(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg);
    void resetLOSTimeout();
    void stopAllMotors();
};
