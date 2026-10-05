/**
 * @file
 * Shares the arm's ROS node with the other source files in this package
 */
#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"

// Defined in main.cpp; "extern" lets other files refer to it without creating a second copy
extern std::shared_ptr<rclcpp::Node> node;
