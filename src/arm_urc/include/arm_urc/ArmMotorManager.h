#pragma once

#include "arm_urc/CANDriver.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>
#include <vector>
#include <cs_plain_guarded.h>
#include "cross_pkg_messages/msg/rover_computer_arm_cmd.hpp"
#include "arm_urc/MotorManager.h"


class ArmMotorManager : public MotorManager {
private:
    rclcpp::Publisher<cross_pkg_messages::msg::RoverComputerArmCMD>::SharedPtr armVelPub;
    rclcpp::Subscription<cross_pkg_messages::msg::RoverComputerArmCMD>::SharedPtr armCommandsSub;
    
    void setupMotors() override;
    void parseCommands(const cross_pkg_messages::msg::RoverComputerArmCMD::SharedPtr msg);
    // void sendArmPowers();

public:
    ArmMotorManager();
    virtual ~ArmMotorManager();
    virtual void tick() override;
    // void manualInputCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
};
