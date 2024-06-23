#include "stdio.h"
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

std::vector<std::shared_ptr<Panel>> uiPanels{};

void loadPanels(const rclcpp::Node::SharedPtr &node) {
    uiPanels.push_back(std::make_shared<ComStatusPanel>("COMS Status", node));
    uiPanels.push_back(std::make_shared<TelemetryPanel>("Telemetry", node));
    uiPanels.push_back(std::make_shared<SystemControlPanel>("System Control", "system_control_node", rclcpp::NodeOptions()));
    uiPanels.push_back(std::make_shared<VideoViewPanel>("Video Stream", node));

    for (auto &p : uiPanels) {
        p->setup();
    }
}

bool close_ui = false;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("GroundStationGUI");

    loadPanels(node);

    RCLCPP_INFO(node->get_logger(), "GroundStationGUI is running");

    auto window = setupIMGUI();

    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    std::chrono::milliseconds sleep_duration(1000 / 60); // 60 Hz

    std::chrono::system_clock::time_point last_frame;

    while (rclcpp::ok() && !glfwWindowShouldClose(window) && !close_ui) {
        glfwPollEvents();

        // ROS updates
        rclcpp::spin_some(node);
        for (auto &pan : uiPanels) {
            pan->update();
        }

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Render frame
        for (auto &pan : uiPanels) {
            pan->renderToScreen();
        }

        renderFrame(window, clear_color);

        auto now = std::chrono::system_clock::now();
        auto delta = now - last_frame;
        auto delta_s = std::chrono::duration<double>(delta).count();
        last_frame = now;

        std::this_thread::sleep_for(sleep_duration);
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down GUI");

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    rclcpp::shutdown();

    return 0;
}
