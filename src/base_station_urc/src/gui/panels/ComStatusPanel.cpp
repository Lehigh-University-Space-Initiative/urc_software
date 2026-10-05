/**
 * @file
 * Implementation of the COMS Status panel
 *
 * How it connects to the system:
 *   - Pings fixed IP addresses with the system ping command: main computer 10.0.0.10, driveline Pi 10.0.0.20, rover antenna 10.0.0.1, base station 10.1.0.1
 *   - Lists every ROS node the GUI can discover on the network, which is a quick check that DDS discovery is working
 */
#include "ComStatusPanel.h"
#include <rclcpp/rclcpp.hpp>

void ComStatusPanel::drawBody()
{
    if (hootl) {
        // Fading the banner text between red and white so test mode is hard to miss
        float t = sin(ImGui::GetTime() * 3.141592f * 4);

        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 255 * t, 255 * t, 255));
        ImGui::Text("Hardware out of the loop test");
        ImGui::PopStyleColor();
    }

    for (size_t i = 0; i < hosts.size(); i++) {
        ImGui::Separator();
        ImGui::Text("%s", (hosts[i]->nickname + " (" + hosts[i]->host + ")").c_str());

        ImGui::SameLine();  // Putting the status on the same line as the host name

        // Syntax: PushStyleColor/PopStyleColor must come in pairs; everything drawn in between uses the color
        ImGui::PushStyleColor(ImGuiCol_Text, conColor(i));
        ImGui::Text("%s", conString(i).c_str());
        ImGui::PopStyleColor();
    }

    ImGui::Separator();
    ImGui::Text("\n\n\n\nROS Nodes");

    // A fixed-height scrollable child area for the node list
    ImGui::BeginChild("scrolling", ImVec2(0, 100), false, ImGuiWindowFlags_HorizontalScrollbar);

    auto activeNodes = node_->get_node_graph_interface()->get_node_names();

    if (activeNodes.empty()) {
        ImGui::Separator();
        ImGui::Text("No active ROS nodes / Unable to communicate with ROS master");
    }

    for (const auto& name : activeNodes) {
        ImGui::Separator();
        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 255, 0, 255));
        ImGui::Text("%s", name.c_str());
        ImGui::PopStyleColor();
    }

    ImGui::EndChild();
}

std::string ComStatusPanel::conString(size_t hostIndex)
{
    std::string str = "";

    auto lastHeard = std::chrono::system_clock::now() - hosts[hostIndex]->getLastContact();

    if ((lastHeard >= maxConnectionDur) && !hootl) {
        str = "Not Connected";
    }
    else if ((lastHeard >= connectionDur) && !hootl) {
        // lastHeard.count() is in nanoseconds; dividing by 1e9 gives seconds, then the decimals are cut off
        auto num_text = std::to_string(((double)lastHeard.count() / 1'000'000'000));
        std::string rounded = num_text.substr(0, num_text.find("."));
        str = "LOS: " + rounded + " seconds";
    }
    else {
        str = "Connected";
    }

    return str;
}

ImU32 ComStatusPanel::conColor(size_t hostIndex)
{
    auto lastHeard = std::chrono::system_clock::now() - hosts[hostIndex]->getLastContact();
    if ((lastHeard >= connectionDur) && !hootl) {
        return IM_COL32(245, 61, 5, 255);  // Orange-red
    }

    return IM_COL32(0, 255, 0, 255);  // Green
}

void ComStatusPanel::setup()
{
    hosts.push_back(new ComStatusChecker("10.0.0.10", "Main Computer"));
    hosts.push_back(new ComStatusChecker("10.0.0.20", "Driveline Pi"));
    hosts.push_back(new ComStatusChecker("10.0.0.1", "Rover Antenna"));
    hosts.push_back(new ComStatusChecker("10.1.0.1", "Base Station"));

    // The hootl parameter is declared in guiMain.cpp and set by the launch file
    if (!node_->get_parameter("hootl", hootl)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get hootl param");
    }
}

void ComStatusPanel::update()
{
    for (size_t i = 0; i < hosts.size(); i++) {
        hosts[i]->check();
    }
}

ComStatusPanel::~ComStatusPanel()
{
    for (auto host : hosts) {
        delete host;
    }
}

ComStatusChecker::ComStatusChecker(std::string host, std::string nickname)
    : host(host), nickname(nickname)
{
}

std::chrono::system_clock::time_point ComStatusChecker::getLastContact()
{
    // Syntax: std::lock_guard locks the mutex now and unlocks it automatically when the function returns
    std::lock_guard<std::mutex> lock(mutex);

    return lastContact;
}

/**
 * Steps:
 *   1. Skip if a ping was started less than connectionTestFreq ago (this runs every frame, ~60 times a second)
 *   2. Otherwise record the start time and run one ping on a background thread, so the GUI never waits on the network
 *   3. If the ping succeeds, record the time as the last contact
 */
void ComStatusChecker::check()
{
    auto now = std::chrono::system_clock::now();

    if ((now - lastCheckTime) <= connectionTestFreq) {
        return;
    }
    lastCheckTime = now;

    // Syntax: [this]() { ... } is a lambda (an inline function) that captures this object so it can use host and mutex
    // Syntax: detach() lets the thread finish on its own; nothing waits for it
    std::thread([this]() {
        // -c1: one ping; -w1: give up after 1 second; output discarded (system() returns 0 on a reply)
        int notFound = system(("ping " + host + " -c1 -w1 > /dev/null 2>&1").c_str());

        if (notFound == 0) {
            std::lock_guard<std::mutex> lock(mutex);
            lastContact = std::chrono::system_clock::now();
        }
    }).detach();
}
