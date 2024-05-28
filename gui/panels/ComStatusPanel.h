#pragma once
#include "../Panel.h"
#include "../GUI.h"
#include <chrono>
#include <thread>
#include <mutex>
#include "std_msgs/msg/string.hpp"
#include <rclcpp/rclcpp.hpp>

class ComStatusChecker {

    std::chrono::system_clock::time_point lastContact{};
    std::chrono::system_clock::time_point lastCheckTime{};
    std::mutex mutex{};
    const std::chrono::milliseconds connectionTestFreq = std::chrono::milliseconds(500);

public:
    ComStatusChecker(std::string host, std::string nickname);

    std::string host;
    std::string nickname;

    std::chrono::system_clock::time_point getLastContact();

    void check();
};

class ComStatusPanel: public Panel, public rclcpp::Node {
public:
    // using Panel::Panel;
    ComStatusPanel(const std::string& name)
    : Panel(name), 
      rclcpp::Node(name + "_node", rclcpp::NodeOptions())
    {}

    virtual void setup() override;
    virtual void update() override;
    ~ComStatusPanel();
protected:
    virtual void drawBody();

    std::vector<ComStatusChecker*> hosts; 
    const std::chrono::milliseconds connectionDur = std::chrono::milliseconds(500);
    const std::chrono::milliseconds maxConnectionDur = std::chrono::milliseconds(3600'000);

    std::string conString(size_t hostIndex);
    ImU32 conColor(size_t hostIndex);

    bool hootl = false;

};