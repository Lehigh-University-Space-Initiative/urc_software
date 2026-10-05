/**
 * @file
 * Base class for one window ("panel") inside the Ground Station GUI
 *
 * To add a panel:
 *   1. Subclass Panel in panels/ and implement drawBody() with ImGui calls
 *   2. Optionally override setup() (create subscriptions) and update() (per-frame non-drawing work)
 *   3. Add it to loadPanels() in guiMain.cpp
 */
#pragma once
#include "GUI.h"
#include <string>
#include <rclcpp/rclcpp.hpp>

class Panel {
protected:
    std::string name;  // Window title (also ImGui's ID for the window, so it must be unique)
    rclcpp::Node::SharedPtr node_;  // ROS node used for this panel's subscriptions and parameters

    /// Draw the panel's contents (the window's ImGui::Begin/End are already handled by renderToScreen)
    virtual void drawBody() = 0;

public:
    /**
     * Parameters (inputs):
     *   name - window title
     *   node - ROS node the panel uses (usually the shared GroundStationGUI node)
     */
    Panel(const std::string& name, const rclcpp::Node::SharedPtr& node);
    virtual ~Panel();

    /// Open the panel's window, draw its body, and close the window (called once per frame)
    void renderToScreen();

    /// Called once at startup, before the window opens
    virtual void setup();
    /// Called once per frame, before drawing, alongside ROS message processing
    virtual void update();
};
