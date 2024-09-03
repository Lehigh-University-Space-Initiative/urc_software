#pragma once
#include "../Panel.h"
#include "../GUI.h"
#include <chrono>
#include <thread>
#include <mutex>
#include "std_msgs/msg/string.hpp"
#include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"

class TelemetryPanel: public Panel {
protected:
    cross_pkg_messages::msg::RoverComputerDriveCMD lastDriveCMD;
    cross_pkg_messages::msg::RoverComputerDriveCMD lastArmCMD;

    rclcpp::Subscription<cross_pkg_messages::msg::RoverComputerDriveCMD>::SharedPtr sub;
    rclcpp::Subscription<cross_pkg_messages::msg::RoverComputerDriveCMD>::SharedPtr sub2;

    virtual void drawBody() override;
public:
    TelemetryPanel(const std::string &name, const rclcpp::Node::SharedPtr &node)
        : Panel(name, node) {}

    void setup() override;
    void update() override;
    ~TelemetryPanel();
};
