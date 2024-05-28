#include "SoftwareDebugPanel.h"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <thread>

std::string timePointAsString(const std::chrono::system_clock::time_point& tp) {
    std::time_t t = std::chrono::system_clock::to_time_t(tp);
    std::string ts = std::ctime(&t);
    ts.resize(ts.size()-1);
    return ts;
}

// Command: sshpass -p lusi ssh pi@192.168.1.5 cat /home/pi/urc_deploy/debugInfo.csv
// Command: sshpass -p {pass} ssh {user}@{ip} cat {path}
std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

void SoftwareDebugPanel::drawBody()
{
    
    ImGui::Columns(4, "debugInfoColumns");
    ImGui::Text("Host");
    ImGui::NextColumn();
    ImGui::Text("Date and Time");
    ImGui::NextColumn();
    ImGui::Text("Git Branch");
    ImGui::NextColumn();
    ImGui::Text("Git User Name");
    ImGui::NextColumn();

    // Loop through hosts
    auto lockHost = hosts.lock();
    auto hosts = *lockHost;
    for (const auto& host : hosts) {
        // Calculate time elapsed since last file read
        std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();
        std::chrono::duration<double> timeElapsed = currentTime - host.time_point;

        // Display host info
        ImGui::Text("Host: %s", host.hostName.c_str());
        ImGui::NextColumn();

        // Display datetime, with elapsed time since first file read
        std::string line = host.debugInfoLines;
        
        if (line.empty()) {
            ImGui::TextColored(ImVec4(1,0,0,1), "Not connected");
        } else {
            std::string timeString = timePointAsString(host.time_point);
            // ImGui::Text("%s ((%f))", timeString.c_str(), timeElapsed);
            ImGui::Text("%.0f s", timeElapsed);
        }
            ImGui::NextColumn();
            ImGui::Text("%s", host.branch.c_str());
            ImGui::NextColumn();
            ImGui::Text("%s", host.userName.c_str());
            ImGui::NextColumn();

            // Separator between hosts
            ImGui::Separator();
        
        }
        ImGui::Columns(1);
}

void SoftwareDebugPanel::setup()
{
    // Populate hosts array
    auto lockHost = hosts.lock();
    *lockHost = {
        {"192.168.1.3", "Host 1", "/home/jimmy/urc_deploy/debugInfo.csv", "jimmy", "lusi"},
        {"192.168.1.5", "Host 2", "/home/pi/urc_deploy/debugInfo.csv", "pi", "lusi"}
    };
}

void SoftwareDebugPanel::update()
{
    std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();
    if (currentTime - lastUpdateTime > refreshInterval) {
        auto thread = new std::thread([this]() {
            auto lockHost = hosts.lock();
            for (auto& host : *lockHost) {
                readDebugInfoFile(host);
                //ROS_INFO("Read debug info file for host %s (%s)", host.hostName.c_str(), host.hostIpAddress.c_str());
            }   
        });
        lastUpdateTime = currentTime;
        thread->detach();
    }
}

// Command: sshpass -p lusi ssh -o ConnectTimeout=2 pi@192.168.0.1 cat /home/pi/urc_deploy/debugInfo.csv
// Command: sshpass -p {pass} ssh -o ConnectTimeout=2 {user}@{ip} cat {path}
void SoftwareDebugPanel::readDebugInfoFile(Host& host)
{
    std::string cmd = "sshpass -p " + host.password + " ssh -o ConnectTimeout=2 " + host.user + "@" + host.hostIpAddress + " cat " + host.debugFilePath;
    //ROS_INFO("Executing command: %s", cmd.c_str());
    std::string result = exec(cmd.c_str());
    //ROS_INFO("Result: %s", result.c_str());

    std::size_t found = result.find_first_of('Email');
    std::string line = result.substr(found+1);
    host.debugInfoLines = line;
    //ROS_INFO("Read the last line from debug info file for host %s (%s)", host.hostName.c_str(), host.hostIpAddress.c_str());
    //ROS_INFO("Last line: %s", line.c_str());

    std::string dateTime, branch, userName;
    std::string gitCommitHash, gitCommitMessage; // don't need these

    // std::istringstream iss(line);
    std::stringstream ss(line);

    std::getline(ss, dateTime, ',');
    std::getline(ss, gitCommitHash, ',');
    std::getline(ss, gitCommitMessage, ',');
    std::getline(ss, branch, ',');
    std::getline(ss, userName, ',');

    std::stringstream timeStringStream(dateTime);
    std::tm t = {};
    timeStringStream >> std::get_time(&t, "%Y-%m-%d %H:%M:%S");
    host.time_point = std::chrono::system_clock::from_time_t(std::mktime(&t));
    
    host.branch = branch;
    host.userName = userName;
}
