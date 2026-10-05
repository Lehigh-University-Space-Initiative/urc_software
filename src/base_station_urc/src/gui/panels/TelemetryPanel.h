/**
 * @file
 * Telemetry panel: visualizes drive and arm commands, and hosts the Swap Joysticks toggle
 */
#pragma once
#include "../Panel.h"
#include "../GUI.h"
#include <chrono>
#include <thread>
#include <mutex>
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"
#include "cross_pkg_messages/msg/arm_input_raw.hpp"

class TelemetryPanel : public Panel {
protected:
    cross_pkg_messages::msg::RoverComputerDriveCMD lastDriveCMD;  // Latest wheel speeds (right side negated for display)
    geometry_msgs::msg::Twist lastCmdVel;  // Latest whole-rover drive command
    cross_pkg_messages::msg::ArmInputRaw lastArmCMD;  // Latest SpaceMouse input

    rclcpp::Subscription<cross_pkg_messages::msg::RoverComputerDriveCMD>::SharedPtr sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
    rclcpp::Subscription<cross_pkg_messages::msg::ArmInputRaw>::SharedPtr sub2;

    // Client for setting parameters on another node (here, JoyMapper's swap_joysticks)
    std::shared_ptr<rclcpp::AsyncParametersClient> joyMapParamClient_;

    virtual void drawBody() override;

public:
    TelemetryPanel(const std::string& name, const rclcpp::Node::SharedPtr& node)
        : Panel(name, node)
    {
    }

    /// Create the subscriptions and the parameter client for joy_mapper
    void setup() override;
    void update() override;
    ~TelemetryPanel();
};
