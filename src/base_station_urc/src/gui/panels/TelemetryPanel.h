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

class TelemetryPanel: public Panel {
protected:
    cross_pkg_messages::msg::RoverComputerDriveCMD lastDriveCMD;
    geometry_msgs::msg::Twist lastCmdVel;
    cross_pkg_messages::msg::ArmInputRaw lastArmCMD;

    rclcpp::Subscription<cross_pkg_messages::msg::RoverComputerDriveCMD>::SharedPtr sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
    rclcpp::Subscription<cross_pkg_messages::msg::ArmInputRaw>::SharedPtr sub2;

    std::shared_ptr<rclcpp::AsyncParametersClient> joyMapParamClient_;

    virtual void drawBody() override;
public:
    TelemetryPanel(const std::string &name, const rclcpp::Node::SharedPtr &node)
        : Panel(name, node) {}

    void setup() override;
    void update() override;
    ~TelemetryPanel();
};
