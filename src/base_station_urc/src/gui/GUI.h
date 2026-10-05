/**
 * @file
 * Common includes for the Ground Station GUI (Dear ImGui + GLFW + OpenGL, plus the ROS types the panels use)
 *
 * How the GUI is built:
 *   - Dear ImGui (libs/imgui) is an "immediate mode" GUI library: every frame, code calls ImGui::Button(...), ImGui::Text(...), etc.
 *   - ImGui draws those widgets and reports clicks right away, so there are no widget objects to keep around
 *   - GLFW opens the window and reads the mouse and keyboard; OpenGL does the actual drawing
 *   - guiMain.cpp runs the frame loop; each Panel subclass in panels/ draws one window inside it
 */
#pragma once

#include <rclcpp/rclcpp.hpp>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h>  // Will drag system OpenGL headers

#include "cs_plain_guarded.h"

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>

/// Set to true (e.g. by the "Close Ground Station" button) to end the frame loop in guiMain.cpp
extern bool close_ui;
