/**
 * @file
 * Software Debug panel: shows which git branch and author each rover computer last deployed (NOT currently shown)
 *
 * Status: compiled into the GUI but not added in guiMain.cpp's loadPanels(), so it never appears
 *
 * Why it's disabled (fix these before wiring it in):
 *   - It logs in over SSH with hardcoded usernames and the password "lusi" via sshpass
 *   - The host IPs (192.168.1.x) don't match the rover's 10.0.0.x network (see softwareUpdate/urc_deploy.py)
 *   - readDebugInfoFile parses with find_first_of('Email'), a multi-character literal that doesn't do what it looks like
 */
#pragma once
#include "../Panel.h"
#include "../GUI.h"
#include <vector>
#include <string>
#include <cs_plain_guarded.h>

class SoftwareDebugPanel : public Panel {
protected:
    virtual void drawBody() override;

public:
    using Panel::Panel;

    /// Fill in the list of hosts to query
    virtual void setup() override;
    /// Every 2 s, re-read each host's debug file on a background thread
    virtual void update() override;

    /// One computer to query, and the deploy info last read from it
    struct Host {
        Host(std::string hostIpAddress, std::string hostName, std::string debugFilePath, std::string user, std::string password)
            : hostIpAddress(hostIpAddress), hostName(hostName), debugFilePath(debugFilePath), user(user), password(password), debugInfoLines(""),
              time_point(std::chrono::system_clock::time_point()), dateTime(""), branch(""), userName("")
        {
        }

        // Written out by hand because std::stringstream (dateTime) can't be copied automatically
        Host(const Host& other)
            : hostIpAddress(other.hostIpAddress), hostName(other.hostName), debugFilePath(other.debugFilePath),
              user(other.user), password(other.password), debugInfoLines(other.debugInfoLines),
              time_point(other.time_point), branch(other.branch), userName(other.userName)
        {
        }

        Host& operator=(const Host& other)
        {
            hostIpAddress = other.hostIpAddress;
            hostName = other.hostName;
            debugFilePath = other.debugFilePath;
            user = other.user;
            password = other.password;
            debugInfoLines = other.debugInfoLines;
            time_point = other.time_point;
            branch = other.branch;
            userName = other.userName;

            return *this;
        }

        std::string hostIpAddress;
        std::string hostName;
        std::string debugFilePath;  // CSV the deploy process writes on that computer
        std::string user;
        std::string password;
        std::string debugInfoLines;  // Last line read from the file (empty means not reached)
        std::chrono::system_clock::time_point time_point;  // Deploy time parsed from the file
        std::stringstream dateTime;
        std::string branch;
        std::string userName;
    };

private:
    // Guarded because update()'s background thread writes it while drawBody() reads it
    libguarded::plain_guarded<std::vector<Host>> hosts;
    std::chrono::system_clock::time_point lastUpdateTime = std::chrono::system_clock::now();
    const std::chrono::milliseconds refreshInterval = std::chrono::milliseconds(2000);

    std::chrono::system_clock::time_point lastFileReadTime = std::chrono::system_clock::now();

    /// SSH into the host, read its debug CSV, and parse the deploy time, branch, and git user
    void readDebugInfoFile(Host& host);
};
