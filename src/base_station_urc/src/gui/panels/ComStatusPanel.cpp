#include "ComStatusPanel.h"
#include <rclcpp/rclcpp.hpp>

void ComStatusPanel::drawBody() {
    if (hootl) {
        // Show test saying "Hardware out of the loop"
        // the text should fade between red and white
        float t = sin(ImGui::GetTime() * 3.141592f * 4);

        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 255 * t, 255 * t, 255));
        ImGui::Text("Hardware out of the loop test");
        ImGui::PopStyleColor();
    }

    for (size_t i = 0; i < hosts.size(); i++) {
        ImGui::Separator();
        ImGui::Text((hosts[i]->nickname + " (" + hosts[i]->host + ")").c_str());

        ImGui::SameLine();

        ImGui::PushStyleColor(ImGuiCol_Text, conColor(i));
        ImGui::Text(conString(i).c_str());
        ImGui::PopStyleColor();
    }

    ImGui::Separator();
    ImGui::Text("\n\n\n\nROS Nodes");

    // Wrap to be scrollable
    ImGui::BeginChild("scrolling", ImVec2(0, 100), false, ImGuiWindowFlags_HorizontalScrollbar);

    auto activeNodes = node_->get_node_graph_interface()->get_node_names();

    if (activeNodes.empty()) {
        ImGui::Separator();
        ImGui::Text("No active ROS nodes / Unable to communicate with ROS master");
    }

    for (const auto& name : activeNodes) {
        ImGui::Separator();
        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 255, 0, 255));
        ImGui::Text(name.c_str());
        ImGui::PopStyleColor();
    }

    ImGui::EndChild();
}

std::string ComStatusPanel::conString(size_t hostIndex) {
    std::string str = "";

    auto lastHeard = std::chrono::system_clock::now() - hosts[hostIndex]->getLastContact();

    if (lastHeard >= maxConnectionDur && !hootl) {
        str = "Not Connected";
    } else if (lastHeard >= connectionDur && !hootl) {
        auto num_text = std::to_string(((double)lastHeard.count() / 1'000'000'000));
        std::string rounded = num_text.substr(0, num_text.find("."));
        str = "LOS: " + rounded + " seconds";
    } else {
        str = "Connected";
    }

    return str;
}

ImU32 ComStatusPanel::conColor(size_t hostIndex) {
    auto lastHeard = std::chrono::system_clock::now() - hosts[hostIndex]->getLastContact();
    if (lastHeard >= connectionDur && !hootl) {
        return IM_COL32(245, 61, 5, 255);
    } else {
        return IM_COL32(0, 255, 0, 255);
    }
}

void ComStatusPanel::setup() {
    hosts.push_back(new ComStatusChecker("192.168.1.3", "Jetson"));
    hosts.push_back(new ComStatusChecker("192.168.1.5", "Pi"));
    hosts.push_back(new ComStatusChecker("192.168.1.160", "Rover Antenna"));
    hosts.push_back(new ComStatusChecker("192.168.1.159", "Base Station"));

    if (!node_->get_parameter("hootl", hootl)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get hootl param");
    }
}

void ComStatusPanel::update() {
    for (size_t i = 0; i < hosts.size(); i++) {
        hosts[i]->check();
    }
}

ComStatusPanel::~ComStatusPanel() {
    for (auto host : hosts)
        delete host;
}

ComStatusChecker::ComStatusChecker(std::string host, std::string nickname)
    : host(host), nickname(nickname) {
}

std::chrono::system_clock::time_point ComStatusChecker::getLastContact() {
    std::lock_guard<std::mutex> lock(mutex);
    return lastContact;
}

void ComStatusChecker::check() {
    auto now = std::chrono::system_clock::now();

    if (now - lastCheckTime <= connectionTestFreq) { return; }

    auto thread = new std::thread([this]() {
        int notFound = system(("ping " + host + " -c1 -w1 > nul").c_str());

        if (notFound == 0) {
            std::lock_guard<std::mutex> lock(mutex);
            lastContact = std::chrono::system_clock::now();
        }
    });

    thread->detach();
}
