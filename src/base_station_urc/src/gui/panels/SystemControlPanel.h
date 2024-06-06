#pragma once
#include "../Panel.h"
#include "../GUI.h"
#include <chrono>
#include <thread>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp" 

class SystemControlPanel : public Panel {
protected:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
    virtual void drawBody();

public:
    SystemControlPanel(const std::string &name, const std::string &node_name, const rclcpp::NodeOptions &options);

    virtual void setup() override;
    virtual void update() override;
    ~SystemControlPanel();
};
