/**
 * @file
 * The arm's motor group: six joint motors plus the gripper (end effector) on CAN bus 1
 */
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
#include "MotorManager.h"

/**
 * Drives the arm joints with position PID and the gripper with open-loop power
 */
class ArmMotorManager : public MotorManager {
private:
    /// Create the six joint motors (CAN IDs 51-56) and the gripper motor (CAN ID 57) on bus 1
    void setupMotors() override;

public:
    using MotorManager::MotorManager;
    virtual ~ArmMotorManager();

    /// Copy hw_commands_ into each joint's PID set point
    virtual void writeMotors() override;

    /**
     * Set new joint position targets
     *
     * Parameters (inputs):
     *   msg - targets in radians for base, shoulder, elbow, and the three wrist axes
     */
    void setArmCommand(const cross_pkg_messages::msg::RoverComputerArmCMD::SharedPtr msg);

    /**
     * Drive the gripper from the operator's buttons
     *
     * Parameters (inputs):
     *   msg - raw operator input; left button opens, right button closes, neither stops
     */
    void setArmCommand(const cross_pkg_messages::msg::ArmInputRaw::SharedPtr msg);
};
