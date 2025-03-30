#pragma once

#include "CANDriver.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>
#include <vector>
#include <memory>
#include <cs_plain_guarded.h>
#include "cross_pkg_messages/msg/rover_computer_arm_cmd.hpp"
#include "cross_pkg_messages/msg/arm_input_raw.hpp"
// #include <hardware_interface/system_interface.hpp>
#include "MotorManager.h"


class ArmMotorManager : public MotorManager {
private:
 
    void setupMotors() override;

public:
    using MotorManager::MotorManager;
    virtual ~ArmMotorManager();

    virtual void writeMotors() override;

    void setArmCommand(const cross_pkg_messages::msg::RoverComputerArmCMD::SharedPtr msg);
    void setArmCommand(const cross_pkg_messages::msg::ArmInputRaw::SharedPtr msg);
};
