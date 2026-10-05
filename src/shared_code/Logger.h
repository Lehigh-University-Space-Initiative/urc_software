/**
 * @file
 * Shared ROS logger used by the rover motor code (CANDriver, MotorManager, and their subclasses)
 */
#pragma once

#include "rclcpp/rclcpp.hpp"

// Syntax: "extern" declares that this variable exists without creating it
// The one real definition is in MotorManager.cpp, so every file that includes this header shares the same logger
extern rclcpp::Logger dl_logger;
