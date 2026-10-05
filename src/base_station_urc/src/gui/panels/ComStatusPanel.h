/**
 * @file
 * COMS Status panel: shows whether each rover computer is reachable and which ROS nodes are running
 */
#pragma once
#include "../Panel.h"
#include "../GUI.h"
#include <chrono>
#include <thread>
#include <mutex>
#include "std_msgs/msg/string.hpp"
#include <rclcpp/rclcpp.hpp>

/**
 * Pings one host in the background, at most once per second, and remembers the last successful reply
 */
class ComStatusChecker {
    std::chrono::system_clock::time_point lastContact{};  // Last time a ping succeeded (written by the ping thread)
    std::chrono::system_clock::time_point lastCheckTime{};  // Last time a ping was started (main thread only)
    std::mutex mutex{};  // Protects lastContact, which the ping thread and the GUI thread both touch
    const std::chrono::milliseconds connectionTestFreq = std::chrono::milliseconds(1000);  // Minimum time between pings

public:
    /**
     * Parameters (inputs):
     *   host - IP address to ping
     *   nickname - name shown in the panel
     */
    ComStatusChecker(std::string host, std::string nickname);

    std::string host;
    std::string nickname;

    /// Last time this host answered a ping (thread-safe)
    std::chrono::system_clock::time_point getLastContact();

    /// Start a background ping if at least connectionTestFreq has passed since the last one
    void check();
};

class ComStatusPanel : public Panel {
public:
    ComStatusPanel(const std::string& name, const rclcpp::Node::SharedPtr& node)
        : Panel(name, node)
    {
    }

    /// Create the host list and read the hootl parameter
    virtual void setup() override;
    /// Start any due pings
    virtual void update() override;
    ~ComStatusPanel();

protected:
    virtual void drawBody() override;

    std::vector<ComStatusChecker*> hosts;  // One checker per rover computer (owned; deleted in the destructor)
    const std::chrono::milliseconds connectionDur = std::chrono::milliseconds(2100);  // Silence longer than this shows LOS
    // Syntax: the ' in 3600'000 is a digit separator (C++14) that only makes the number easier to read
    const std::chrono::milliseconds maxConnectionDur = std::chrono::milliseconds(3600'000);  // After an hour, show "Not Connected"

    /// Status text for a host: "Connected", "LOS: N seconds", or "Not Connected"
    std::string conString(size_t hostIndex);
    /// Status color for a host: green when connected, orange-red when not
    ImU32 conColor(size_t hostIndex);

    bool hootl = false;  // Hardware-out-of-the-loop test mode (every host is shown as connected)
};
