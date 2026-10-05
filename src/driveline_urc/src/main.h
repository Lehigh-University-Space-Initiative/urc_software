/**
 * @file
 * Shares the driveline's ROS node with the other source files in this package
 */
#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"

// Defined in main.cpp; "extern" lets other files (e.g. DriveTrainMotorManager.cpp) refer to it
extern std::shared_ptr<rclcpp::Node> node;
