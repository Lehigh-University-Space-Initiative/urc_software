#include "TelemetryPanel.h"
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>

void TelemetryPanel::drawBody() {
    ImGui::Text("Motor Inputs");

    ImGui::Separator();

    ImGui::Text("Left Motors");
    
    auto l1_n = lastDriveCMD.cmd_l.x < 0;
    auto l2_n = lastDriveCMD.cmd_l.y < 0;
    auto l3_n = lastDriveCMD.cmd_l.z < 0;
    auto r1_n = lastDriveCMD.cmd_r.x < 0;
    auto r2_n = lastDriveCMD.cmd_r.y < 0;
    auto r3_n = lastDriveCMD.cmd_r.z < 0;

    if (l1_n)
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, IM_COL32(255, 0, 0, 255));
    ImGui::ProgressBar(abs(lastDriveCMD.cmd_l.x)); 
    if (l1_n)
        ImGui::PopStyleColor();

    if (l2_n)
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, IM_COL32(255, 0, 0, 255));
    ImGui::ProgressBar(abs(lastDriveCMD.cmd_l.y)); 
    if (l2_n)
        ImGui::PopStyleColor();

    if (l3_n)
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, IM_COL32(255, 0, 0, 255));
    ImGui::ProgressBar(abs(lastDriveCMD.cmd_l.z)); 
    if (l3_n)
        ImGui::PopStyleColor();

    ImGui::Text("Right Motors");

    if (r1_n)
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, IM_COL32(255, 0, 0, 255));
    ImGui::ProgressBar(abs(lastDriveCMD.cmd_r.x)); 
    if (r1_n)
        ImGui::PopStyleColor();

    if (r2_n)
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, IM_COL32(255, 0, 0, 255));
    ImGui::ProgressBar(abs(lastDriveCMD.cmd_r.y));
    if (r2_n)
        ImGui::PopStyleColor();

    if (r3_n)
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, IM_COL32(255, 0, 0, 255));
    ImGui::ProgressBar(abs(lastDriveCMD.cmd_r.z));
    if (r3_n)
        ImGui::PopStyleColor();

    ImGui::Separator();

    ImGui::Text("Arm Inputs");

    ImGui::Text("Left Stick");

    ImGui::Text("Pitch");
    ImGui::SameLine();
    ImGui::ProgressBar(lastArmCMD.cmd_l.x * 0.5 + 0.5);

    ImGui::Text("Base");
    ImGui::SameLine();
    ImGui::ProgressBar(lastArmCMD.cmd_l.y * 0.5 + 0.5);

    ImGui::Text("Wrist Pitch");
    ImGui::SameLine();
    ImGui::ProgressBar(lastArmCMD.cmd_l.z * 0.5 + 0.5);

    ImGui::Separator();

    ImGui::Text("Right Stick");

    ImGui::Text("Elbow Pitch");
    ImGui::SameLine();
    ImGui::ProgressBar(lastArmCMD.cmd_r.x * 0.5 + 0.5);

    ImGui::Text("Wrist Rotate");
    ImGui::SameLine();
    ImGui::ProgressBar(lastArmCMD.cmd_r.y * 0.5 + 0.5);

    ImGui::Text("Wrist Rotate");
    ImGui::SameLine();
    ImGui::ProgressBar(lastArmCMD.cmd_r.z * 0.5 + 0.5);
}

void TelemetryPanel::setup() {
    auto f = [this](const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg) {
        this->lastDriveCMD = *msg;

        // Invert right side
        this->lastDriveCMD.cmd_r.x *= -1;
        this->lastDriveCMD.cmd_r.y *= -1;
        this->lastDriveCMD.cmd_r.z *= -1;
    };

    auto f2 = [this](const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg) {
        this->lastArmCMD = *msg;
    };

    sub = node_->create_subscription<cross_pkg_messages::msg::RoverComputerDriveCMD>(
        "roverDriveCommands", 10, f);
    sub2 = node_->create_subscription<cross_pkg_messages::msg::RoverComputerDriveCMD>(
        "manualArmControl", 10, f2);
}

void TelemetryPanel::update() {
    // Add any update logic if needed
}

TelemetryPanel::~TelemetryPanel() {
}
