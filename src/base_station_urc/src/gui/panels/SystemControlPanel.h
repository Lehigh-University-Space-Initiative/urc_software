#pragma once
#include "../Panel.h"
#include "../GUI.h"
#include <chrono>
#include <thread>
#include <mutex>
#include "std_msgs/msg/string.hpp" 

class SystemControlPanel: public Panel, public rclcpp::Node{
protected:

    //models:
    // cross_pkg_messages::RoverComputerDriveCMD lastDriveCMD;

    // ros::Subscriber sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;

    // ros::NodeHandle n;

    virtual void drawBody();
public:
    using Panel::Panel;

    SystemControlPanel(const std::string & name, const std::string & node_name, const rclcpp::NodeOptions & options)
    : Panel(name),  // Initialize Panel with the name
      rclcpp::Node(node_name, options) {}

    virtual void setup() override;
    virtual void update() override;
    ~SystemControlPanel();
};