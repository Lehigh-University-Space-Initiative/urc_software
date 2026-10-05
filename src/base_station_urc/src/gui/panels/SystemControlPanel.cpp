/**
 * @file
 * Implementation of the System Control panel
 *
 * Known gaps (left as-is, worth fixing):
 *   - "Motor Freeze" and "Deploy Code" buttons do nothing yet
 *   - Both "Reboot" buttons run ~/URC_2022/URC_DeployTools/reboot.sh, a script from an older repo that isn't in this one
 *   - The two "Reboot" buttons share a label; ImGui uses labels as IDs, so they collide (use "Reboot##2" to give one a unique ID)
 *   - /op_mode and /lusi_vision_mode are set on this panel's own node, so other nodes don't see them
 *   - For example, VideoStreamer reads its own lusi_vision_mode parameter, not this one
 */
#include "SystemControlPanel.h"

// Creating a dedicated node for this panel (the other panels share the GUI node)
SystemControlPanel::SystemControlPanel(const std::string& name, const std::string& node_name, const rclcpp::NodeOptions& options)
    : Panel(name, std::make_shared<rclcpp::Node>(node_name, options))
{
}

void SystemControlPanel::drawBody()
{
    // Software locks all motors to stop
    // Syntax: ImGui::Button returns true on the frame it is clicked, so the if-body runs once per click
    if (ImGui::Button("Motor Freeze")) {
        // Add motor freeze logic here
    }

    ImGui::SameLine();

    // op_mode: 0 = drive control, 1 = arm control (the button offers switching to the other one)
    int op_mode;
    node_->get_parameter("/op_mode", op_mode);

    if (op_mode) {
        if (ImGui::Button("Switch to Drive Control")) {
            node_->set_parameter(rclcpp::Parameter("/op_mode", 0));
        }
    }
    else {
        if (ImGui::Button("Switch to Arm Control")) {
            node_->set_parameter(rclcpp::Parameter("/op_mode", 1));
        }
    }

    if (ImGui::Button("Reboot")) {
        system("~/URC_2022/URC_DeployTools/reboot.sh");
    }

    ImGui::SameLine();

    if (ImGui::Button("Deploy Code")) {
        // Add deploy code logic here
    }

    ImGui::SameLine();

    if (ImGui::Button("Close Ground Station")) {
        close_ui = true;  // guiMain.cpp's frame loop checks this and exits
    }

    int lusi_vision_mode;
    node_->get_parameter("/lusi_vision_mode", lusi_vision_mode);

    ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(10, 90, 230, 255));  // Blue buttons for LUSI Vision

    // Note: these labels name the opposite of what clicking does (in mode 1 the button says "Enable" but sets mode 0)
    if (lusi_vision_mode) {
        if (ImGui::Button("Enable 3D LUSI Vision")) {
            node_->set_parameter(rclcpp::Parameter("/lusi_vision_mode", 0));
        }
    }
    else {
        if (ImGui::Button("Disable 3D LUSI Vision")) {
            node_->set_parameter(rclcpp::Parameter("/lusi_vision_mode", 1));
        }
    }

    ImGui::PopStyleColor();

    if (ImGui::Button("Reboot")) {
        system("~/URC_2022/URC_DeployTools/reboot.sh");
    }
}

void SystemControlPanel::setup()
{
    // Declaring the parameters with a default of 0 unless they already exist
    if (!node_->has_parameter("/op_mode")) {
        node_->declare_parameter("/op_mode", 0);
    }
    if (!node_->has_parameter("/lusi_vision_mode")) {
        node_->declare_parameter("/lusi_vision_mode", 0);
    }
}

void SystemControlPanel::update()
{
}

SystemControlPanel::~SystemControlPanel()
{
}
