#include "SystemControlPanel.h"

SystemControlPanel::SystemControlPanel(const std::string &name, const std::string &node_name, const rclcpp::NodeOptions &options)
    : Panel(name, std::make_shared<rclcpp::Node>(node_name, options)) {}

void SystemControlPanel::drawBody() {
    // Software locks all motors to stop
    if (ImGui::Button("Motor Freeze")) {
        // Add motor freeze logic here
    }

    ImGui::SameLine();

    int op_mode;
    node_->get_parameter("/op_mode", op_mode);

    if (op_mode) {
        if (ImGui::Button("Switch to Drive Control")) {
            node_->set_parameter(rclcpp::Parameter("/op_mode", 0));
        }
    } else {
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
        close_ui = true;
    }

    int lusi_vision_mode;
    node_->get_parameter("/lusi_vision_mode", lusi_vision_mode);

    ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(10, 90, 230, 255));

    if (lusi_vision_mode) {
        if (ImGui::Button("Enable 3D LUSI Vision")) {
            node_->set_parameter(rclcpp::Parameter("/lusi_vision_mode", 0));
        }
    } else {
        if (ImGui::Button("Disable 3D LUSI Vision")) {
            node_->set_parameter(rclcpp::Parameter("/lusi_vision_mode", 1));
        }
    }

    ImGui::PopStyleColor();

    if (ImGui::Button("Reboot")) {
        system("~/URC_2022/URC_DeployTools/reboot.sh");
    }
}

void SystemControlPanel::setup() {
    if (!node_->has_parameter("/op_mode")) {
        node_->declare_parameter("/op_mode", 0);
    }
    if (!node_->has_parameter("/lusi_vision_mode")) {
        node_->declare_parameter("/lusi_vision_mode", 0);
    }
}

void SystemControlPanel::update() {
    // Update logic here if needed
}

SystemControlPanel::~SystemControlPanel() {}
