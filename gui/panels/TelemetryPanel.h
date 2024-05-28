#pragma once
#include "../Panel.h"
#include "../GUI.h"
#include <chrono>
#include <thread>
#include <mutex>
#include "std_msgs/msg/string.hpp"


class TelemetryPanel: public Panel, public rclcpp::Node {
protected:

    //models:
    // cross_pkg_messages::RoverComputerDriveCMD lastDriveCMD;
    // cross_pkg_messages::RoverComputerDriveCMD lastArmCMD;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub2;

    virtual void drawBody();
public:
    TelemetryPanel(const std::string & name, const std::string & node_name, const rclcpp::NodeOptions & options)
    : Panel(name),  // Initialize Panel with the name
      rclcpp::Node(node_name, options) {}

    void setup() override;
    void update() override;
    ~TelemetryPanel();
};