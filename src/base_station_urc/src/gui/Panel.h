#pragma once
#include "GUI.h"
#include <string>
#include <rclcpp/rclcpp.hpp>

class Panel {
protected:
    std::string name;
    rclcpp::Node::SharedPtr node_;

    // The begin and end statements are already handled so continue without them
    virtual void drawBody() = 0;

public:
    Panel(const std::string &name, const rclcpp::Node::SharedPtr &node);
    virtual ~Panel();

    void renderToScreen();

    // Called once at setup
    virtual void setup();
    // Called in between frames during ROS update time
    virtual void update();
};
