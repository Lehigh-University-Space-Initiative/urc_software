/**
 * @file
 * System Control panel: buttons for control mode, LUSI Vision mode, and closing the GUI
 */
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
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;  // Unused
    virtual void drawBody();

public:
    /**
     * Unlike the other panels, this one creates its own ROS node
     *
     * Parameters (inputs):
     *   name - window title
     *   node_name - name of the ROS node to create (e.g. "system_control_node")
     *   options - options for that node
     */
    SystemControlPanel(const std::string& name, const std::string& node_name, const rclcpp::NodeOptions& options);

    /// Declare the /op_mode and /lusi_vision_mode parameters on this panel's node
    virtual void setup() override;
    virtual void update() override;
    ~SystemControlPanel();
};
