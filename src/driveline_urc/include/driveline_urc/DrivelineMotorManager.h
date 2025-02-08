#pragma once

#include "CANDriver.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>
#include <vector>
#include <cs_plain_guarded.h>
#include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"
#include <hardware_interface/system_interface.hpp>
#include "MotorManager.h"


class DrivelineMotorManager : public MotorManager {
private:
    
    void setupMotors() override;

public:
    DrivelineMotorManager();
    virtual ~DrivelineMotorManager();
};
