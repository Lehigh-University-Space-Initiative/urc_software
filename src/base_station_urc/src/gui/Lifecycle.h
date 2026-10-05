/**
 * @file
 * Window setup and per-frame rendering for the Ground Station GUI
 */
#pragma once

#include "GUI.h"

/**
 * Open the GUI window and initialize Dear ImGui
 *
 * Return value:
 *   the GLFW window, or nullptr if no window can be created (e.g. no X11 display is reachable)
 */
GLFWwindow* setupIMGUI();

/**
 * Draw everything queued by the panels this frame and show it
 *
 * Parameters (inputs):
 *   window - the window from setupIMGUI()
 *   clear_color - background color behind the panels
 */
void renderFrame(GLFWwindow* window, ImVec4 clear_color);
