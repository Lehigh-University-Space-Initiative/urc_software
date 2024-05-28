#pragma once
#include "../Panel.h"
#include "../GUI.h"
#include <vector>
#include <string>
#include <cs_plain_guarded.h>

class SoftwareDebugPanel: public Panel {
protected:
    virtual void drawBody() override;
public:
    using Panel::Panel;

    virtual void setup() override;
    virtual void update() override;

    // Host struct
    struct Host {
        Host(std::string hostIpAddress, std::string hostName, std::string debugFilePath, std::string user, std::string password)
        : hostIpAddress(hostIpAddress), hostName(hostName), debugFilePath(debugFilePath), user(user), password(password), debugInfoLines(""),
        time_point(std::chrono::system_clock::time_point()), dateTime(""), branch(""), userName("")
        {}

        // Copy constructor
        Host(const Host& other)
        : hostIpAddress(other.hostIpAddress), hostName(other.hostName), debugFilePath(other.debugFilePath),
        user(other.user), password(other.password), debugInfoLines(other.debugInfoLines),
        time_point(other.time_point), branch(other.branch), userName(other.userName)
        {}

        // Copy assignment operator
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
        std::string debugFilePath;
        std::string user;
        std::string password;
        std::string debugInfoLines;
        std::chrono::system_clock::time_point time_point;
        std::stringstream dateTime;
        std::string branch;
        std::string userName;
    };

private:
    // std::vector<Host> hosts;
    libguarded::plain_guarded<std::vector<Host>> hosts;
    // struct with 3 properties: ip address, host name, path to debug file
    // array of the struct called host which defines each computer
    // in the setup() method will populate all that
    std::chrono::system_clock::time_point lastUpdateTime = std::chrono::system_clock::now();
    const std::chrono::milliseconds refreshInterval = std::chrono::milliseconds(2000);

    std::chrono::system_clock::time_point lastFileReadTime = std::chrono::system_clock::now();

    void readDebugInfoFile(Host& host);
};