/**
 * @file
 * Implementation of the Software Debug panel (not currently shown; see SoftwareDebugPanel.h)
 */
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

/// Format a time point like "Mon Oct  4 14:20:00 2026"
std::string timePointAsString(const std::chrono::system_clock::time_point& tp)
{
    std::time_t t = std::chrono::system_clock::to_time_t(tp);
    std::string ts = std::ctime(&t);
    ts.resize(ts.size() - 1);  // Dropping the trailing newline ctime adds

    return ts;
}

/**
 * Run a shell command and return everything it printed
 *
 * Parameters (inputs):
 *   cmd - the command line to run
 *
 * Return value:
 *   the command's standard output
 *
 * Example: sshpass -p {pass} ssh {user}@{ip} cat {path}
 */
std::string exec(const char* cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    // Syntax: unique_ptr with a custom deleter calls pclose() automatically when pipe goes out of scope
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
    // A four-column table: host, time since deploy, branch, git user
    ImGui::Columns(4, "debugInfoColumns");
    ImGui::Text("Host");
    ImGui::NextColumn();
    ImGui::Text("Date and Time");
    ImGui::NextColumn();
    ImGui::Text("Git Branch");
    ImGui::NextColumn();
    ImGui::Text("Git User Name");
    ImGui::NextColumn();

    // Copying the host list under the lock, then drawing from the copy
    auto lockHost = hosts.lock();
    auto hosts = *lockHost;
    for (const auto& host : hosts) {
        std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();
        std::chrono::duration<double> timeElapsed = currentTime - host.time_point;

        ImGui::Text("Host: %s", host.hostName.c_str());
        ImGui::NextColumn();

        std::string line = host.debugInfoLines;

        if (line.empty()) {
            ImGui::TextColored(ImVec4(1, 0, 0, 1), "Not connected");
        }
        else {
            ImGui::Text("%.0f s", timeElapsed.count());
        }
        ImGui::NextColumn();
        ImGui::Text("%s", host.branch.c_str());
        ImGui::NextColumn();
        ImGui::Text("%s", host.userName.c_str());
        ImGui::NextColumn();

        ImGui::Separator();
    }
    ImGui::Columns(1);
}

void SoftwareDebugPanel::setup()
{
    auto lockHost = hosts.lock();
    *lockHost = {
        {"192.168.1.3", "Host 1", "/home/jimmy/urc_deploy/debugInfo.csv", "jimmy", "lusi"},
        {"192.168.1.5", "Host 2", "/home/pi/urc_deploy/debugInfo.csv", "pi", "lusi"}
    };
}

void SoftwareDebugPanel::update()
{
    std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();
    if ((currentTime - lastUpdateTime) > refreshInterval) {
        // Reading over SSH on a background thread so a slow or unreachable host doesn't freeze the GUI
        std::thread([this]() {
            auto lockHost = hosts.lock();
            for (auto& host : *lockHost) {
                readDebugInfoFile(host);
            }
        }).detach();
        lastUpdateTime = currentTime;
    }
}

/**
 * Steps:
 *   1. SSH in (2 s timeout) and print the host's debug CSV
 *   2. Keep the text after the header, then split it on commas: date, commit hash, commit message, branch, user
 *   3. Parse the date ("YYYY-MM-DD HH:MM:SS") into a time point
 */
void SoftwareDebugPanel::readDebugInfoFile(Host& host)
{
    std::string cmd = "sshpass -p " + host.password + " ssh -o ConnectTimeout=2 " + host.user + "@" + host.hostIpAddress + " cat " + host.debugFilePath;
    std::string result = exec(cmd.c_str());

    // Bug: 'Email' is a multi-character literal (an int), not a string, so this doesn't find the word "Email"
    std::size_t found = result.find_first_of('Email');
    std::string line = result.substr(found + 1);
    host.debugInfoLines = line;

    std::string dateTime, branch, userName;
    std::string gitCommitHash, gitCommitMessage;  // Parsed but unused

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
