#pragma once


#include "MotorManager.h"
#include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"
#include "rclcpp/rclcpp.hpp"


class DriveTrainMotorManager : public MotorManager {
private:
    rclcpp::Subscription<cross_pkg_messages::msg::RoverComputerDriveCMD>::SharedPtr driveCommandsSub_;
    rclcpp::Publisher<cross_pkg_messages::msg::RoverComputerDriveCMD>::SharedPtr wheelVelPub_;
    void setupMotors() override;

public:
    using MotorManager::MotorManager;
    virtual ~DriveTrainMotorManager();
    virtual void writeMotors() override;
    void setCommands(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg);
};
