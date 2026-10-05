/**
 * @file
 * GroundStationGUI node: the operator's window for monitoring and controlling the rover
 *
 * Runs on: the base station laptop
 * Started by: base_station_launch.py (run modes "base_station" and "hootl")
 *
 * Parameters:
 *   hootl (bool, default false) - hardware-out-of-the-loop test mode; shows a banner and treats every rover link as connected
 *   stream_cam (string) - declared but unused here (the Video Stream panel sets VideoStreamer's own stream_cam instead)
 *
 * Panels (each is one window, see panels/):
 *   - COMS Status: pings each rover computer and lists running ROS nodes
 *   - Telemetry: drive and arm command visualizations, plus the Swap Joysticks toggle
 *   - System Control: control-mode and LUSI Vision toggles, and the Close button
 *   - Video Stream: rover camera feed with camera selection
 *
 * How it connects to the system:
 *   - Everything the panels show arrives as ROS topics (see each panel's setup())
 *   - The window needs an X11 display; in Docker that means DISPLAY plus the /tmp/.X11-unix mount (see launchScript.sh)
 */
#include "stdio.h"
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "GUI.h"
#include "Lifecycle.h"
#include "Panel.h"
#include "panels/ComStatusPanel.h"
#include "panels/TelemetryPanel.h"
#include "panels/SystemControlPanel.h"
#include "panels/VideoViewPanel.h"
#include "panels/SoftwareDebugPanel.h"

// Syntax: std::vector<std::shared_ptr<Panel>> holds pointers to the base class, so it can hold any Panel subclass
std::vector<std::shared_ptr<Panel>> uiPanels{};

/**
 * Create every panel and run its one-time setup
 *
 * Parameters (inputs):
 *   node - the shared GUI node most panels use for their subscriptions
 *
 * SoftwareDebugPanel exists but isn't added here (see its header for why)
 */
void loadPanels(const rclcpp::Node::SharedPtr& node)
{
    uiPanels.push_back(std::make_shared<ComStatusPanel>("COMS Status", node));
    uiPanels.push_back(std::make_shared<TelemetryPanel>("Telemetry", node));
    // System Control creates its own ROS node rather than sharing the GUI node
    // use_global_arguments(false) keeps the launch file's node-name remap (name="ground_station_gui") off this node
    // Without it both nodes were renamed ground_station_gui, and ROS warned about duplicate node names
    uiPanels.push_back(std::make_shared<SystemControlPanel>("System Control", "system_control_node", rclcpp::NodeOptions().use_global_arguments(false)));
    uiPanels.push_back(std::make_shared<VideoViewPanel>("Video Stream", node));

    for (auto& p : uiPanels) {
        p->setup();
    }
}

bool close_ui = false;

/**
 * Steps:
 *   1. Start ROS, create the GUI node, and declare its parameters
 *   2. Create the panels, then open the window
 *   3. Each frame (~60 Hz): process ROS messages, update panels, draw panels, show the frame
 *   4. On close, shut down ImGui, the window, and ROS
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("GroundStationGUI");

    node->declare_parameter<std::string>("stream_cam", "");
    // Read by ComStatusPanel::setup(); run_nodes.sh's hootl mode passes hootl:=true through base_station_launch.py
    node->declare_parameter<bool>("hootl", false);

    loadPanels(node);

    RCLCPP_INFO(node->get_logger(), "GroundStationGUI is running");

    auto window = setupIMGUI();

    // Failing loud with the fix when there's no display, then shutting ROS down cleanly (no exit() mid-run)
    if (window == nullptr) {
        const char* display = std::getenv("DISPLAY");
        RCLCPP_FATAL(node->get_logger(), "Cannot open the GUI window: no X11 display reachable (DISPLAY=%s)", display ? display : "<not set>");
        RCLCPP_FATAL(node->get_logger(), "Linux: run 'xhost +local:' first; Docker: pass -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix (see README, X11 / GUI)");
        uiPanels.clear();
        rclcpp::shutdown();

        return 1;
    }

    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);  // Background color (red, green, blue, alpha)

    std::chrono::milliseconds sleep_duration(1000 / 60);  // ~60 frames per second

    std::chrono::system_clock::time_point last_frame;

    while (rclcpp::ok() && !glfwWindowShouldClose(window) && !close_ui) {
        glfwPollEvents();  // Handling mouse, keyboard, and window events

        rclcpp::spin_some(node);
        for (auto& pan : uiPanels) {
            pan->update();
        }

        // Starting a new ImGui frame; panels then queue up their widgets for it
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        for (auto& pan : uiPanels) {
            pan->renderToScreen();
        }

        renderFrame(window, clear_color);

        // Frame timing is measured but not currently used
        auto now = std::chrono::system_clock::now();
        auto delta = now - last_frame;
        auto delta_s = std::chrono::duration<double>(delta).count();
        last_frame = now;

        std::this_thread::sleep_for(sleep_duration);
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down GUI");

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    rclcpp::shutdown();

    return 0;
}
