/**
 * @file
 * Implementation of the Telemetry panel
 *
 * Subscribes:
 *   motorVels (cross_pkg_messages/RoverComputerDriveCMD) - per-wheel speeds for the wheel bars
 *   cmd_vel (geometry_msgs/Twist) - whole-rover drive command from JoyMapper
 *   /armInputRaw (cross_pkg_messages/ArmInputRaw) - SpaceMouse input for the arm bars
 *
 * Known gap:
 *   - Nothing publishes motorVels yet (DriveTrainMotorManager declares a wheelVelPub but never creates it)
 *   - So the six wheel bars stay at zero; subscribing to roverDriveCommands instead would show commanded speeds
 */
#include "TelemetryPanel.h"
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>

/**
 * Draw a progress bar for a value in [-1, 1], filled by its magnitude and red when negative
 *
 * Parameters (inputs):
 *   val - value to show (values beyond +/-1 just fill the bar)
 *   overlay - text drawn on top of the bar
 *   barSize - size in pixels; {0, 0} means "fill the available width"
 */
void signedProgressBar(float val, std::string overlay = "", ImVec2 barSize = {0, 0})
{
    // Syntax: "= {0, 0}" in a parameter list is a default value used when the caller leaves it out
    auto negative = val < 0;
    auto mag = abs(val);
    if (negative) {
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, IM_COL32(255, 0, 0, 255));
    }
    if ((barSize.x == 0) && (barSize.y == 0)) {
        ImGui::ProgressBar(mag, {-10, 0}, overlay.c_str());  // Negative width means "leave 10 px at the right edge"
    }
    else {
        ImGui::ProgressBar(mag, barSize, overlay.c_str());
    }
    if (negative) {
        ImGui::PopStyleColor();
    }
}

/**
 * Steps:
 *   1. Draw a rover outline with three wheels per side and a speed bar next to each wheel
 *   2. Show the forward and turn commands as text and bars, plus the Swap Joysticks checkbox
 *   3. Draw six bars for the SpaceMouse input: move X/Y/Z on the left, rotate X/Y/Z on the right
 */
void TelemetryPanel::drawBody()
{
    ImGui::Text("Driveline Telemetry");

    ImGui::Separator();

    {
        // Drawing shapes directly in screen pixels; p is the top-left corner of the space below the title
        ImDrawList* draw_list = ImGui::GetWindowDrawList();

        const ImVec2 p = ImGui::GetCursorScreenPos();
        auto win_size = ImGui::GetWindowSize();
        float outlineThickness = 2;
        auto outlineColor = ImU32(ImColor(200, 200, 0));  // Yellow

        // The rover body: an 80x150 px rectangle centered horizontally
        ImVec2 roverBoxSize = {80, 150};
        ImVec2 roverBoxStart = {p.x + win_size.x / 2 - roverBoxSize.x / 2, p.y + 25};
        draw_list->AddRect(roverBoxStart, {roverBoxStart.x + roverBoxSize.x, roverBoxStart.y + roverBoxSize.y}, outlineColor, 0, 0, outlineThickness);

        // side 0 = left, side 1 = right; wheel 0/1/2 = front/middle/back
        for (size_t side = 0; side < 2; side++) {
            for (size_t wheel = 0; wheel < 3; wheel++) {
                float wheelRad = 20;
                ImVec2 wheelSize = {10, wheelRad};
                ImVec2 wheelCenter = {roverBoxStart.x - wheelSize.x, roverBoxStart.y + wheelRad + wheel * roverBoxSize.y / 3};
                if (side) {
                    wheelCenter.x += roverBoxSize.x + 2 * wheelSize.x;
                }

                ImVec2 wheelMin = {wheelCenter.x - wheelSize.x / 2, wheelCenter.y - wheelSize.y / 2};
                ImVec2 wheelMax = {wheelCenter.x + wheelSize.x / 2, wheelCenter.y + wheelSize.y / 2};
                draw_list->AddRectFilled(wheelMin, wheelMax, outlineColor);

                // Placing this wheel's speed bar 30 px outside the wheel
                ImVec2 sliderSize = {50, wheelSize.y};
                ImVec2 sliderStart = {wheelCenter.x - 30 - sliderSize.x, wheelCenter.y - sliderSize.y / 2};
                if (side) {
                    sliderStart.x = wheelCenter.x + 30;
                }
                ImGui::SetCursorScreenPos(sliderStart);
                // Syntax: "condition ? a : b" picks a when the condition is true, otherwise b
                auto sliderSide = side ? lastDriveCMD.cmd_r : lastDriveCMD.cmd_l;
                float sliderVal = 0;

                if (wheel == 0) {
                    sliderVal = sliderSide.x;
                }
                else if (wheel == 1) {
                    sliderVal = sliderSide.y;
                }
                else {
                    sliderVal = sliderSide.z;
                }

                signedProgressBar(sliderVal / 25, " ", sliderSize);  // Full bar at 25 rad/s of wheel speed
            }
        }

        // Moving the cursor back and reserving space so the next widgets go below the drawing
        ImGui::SetCursorScreenPos(p);
        ImGui::Dummy({10, roverBoxSize.y + 50});

        ImGui::Text("Linear Velocity CMD: %.2f m/s", lastCmdVel.linear.x);
        signedProgressBar(lastCmdVel.linear.x / 1, " ");  // Full bar at 1 m/s
        ImGui::Text("Angular Velocity CMD: %.1f deg/s", lastCmdVel.angular.y);
        signedProgressBar(lastCmdVel.angular.y / 120, " ");  // Full bar at 120 deg/s (JoyMapper's maximum)

        // Syntax: a static local keeps its value across frames; it starts true to match JoyMapper's default
        static bool joy_swap = true;

        if (ImGui::Checkbox("Swap Joysticks", &joy_swap)) {
            // Setting JoyMapper's parameter remotely; the callback runs later, when the reply arrives
            joyMapParamClient_->set_parameters({rclcpp::Parameter("swap_joysticks", joy_swap)},
                                               [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
                                                   future.wait();
                                                   auto results = future.get();
                                                   if (results.size() != 1) {
                                                       RCLCPP_ERROR_STREAM(node_->get_logger(), "expected 1 result, got " << results.size());
                                                   }
                                                   else {
                                                       if (results[0].successful) {
                                                           RCLCPP_INFO(node_->get_logger(), "success");
                                                       }
                                                       else {
                                                           RCLCPP_ERROR(node_->get_logger(), "failure");
                                                       }
                                                   }
                                               });
        }
    }

    ImGui::Separator();

    ImGui::Text("Arm Telemetry");

    ImGui::Separator();

    {
        // Same layout as the wheels, reused for the six SpaceMouse axes (no rover outline this time)
        const ImVec2 p = ImGui::GetCursorScreenPos();
        auto win_size = ImGui::GetWindowSize();

        ImVec2 roverBoxSize = {20, 150};
        ImVec2 roverBoxStart = {p.x + win_size.x / 2 - roverBoxSize.x / 2, p.y + 25};

        // side 0 = linear (move) axes, side 1 = angular (rotate) axes
        for (size_t side = 0; side < 2; side++) {
            for (size_t wheel = 0; wheel < 3; wheel++) {
                float wheelRad = 20;
                ImVec2 wheelSize = {10, wheelRad};
                ImVec2 wheelCenter = {roverBoxStart.x - wheelSize.x, roverBoxStart.y + wheelRad + wheel * roverBoxSize.y / 3};
                if (side) {
                    wheelCenter.x += roverBoxSize.x + 2 * wheelSize.x;
                }

                ImVec2 sliderSize = {50, wheelSize.y};
                ImVec2 sliderStart = {wheelCenter.x - 30 - sliderSize.x, wheelCenter.y - sliderSize.y / 2};
                if (side) {
                    sliderStart.x = wheelCenter.x + 30;
                }
                ImGui::SetCursorScreenPos(sliderStart);
                auto sliderSide = side ? lastArmCMD.angular_input : lastArmCMD.linear_input;
                float sliderVal = 0;
                std::string label = "";  // Built but not drawn yet

                if (side) {
                    label += "Rotate ";
                }
                else {
                    label += "Move ";
                }

                if (wheel == 0) {
                    sliderVal = sliderSide.x;
                    label += "X";
                }
                else if (wheel == 1) {
                    sliderVal = sliderSide.y;
                    label += "Y";
                }
                else {
                    sliderVal = sliderSide.z;
                    label += "Z";
                }
                signedProgressBar(sliderVal, " ", sliderSize);
            }
        }
    }
}

/**
 * Steps:
 *   1. Subscribe to the drive and arm topics (first, so telemetry works even if JoyMapper isn't running)
 *   2. Create the parameter client for JoyMapper, and wait briefly for it so the first toggle isn't lost
 */
void TelemetryPanel::setup()
{
    // Syntax: [this](...) { ... } is a lambda; capturing "this" lets it write to this panel's members
    auto f = [this](const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg) {
        this->lastDriveCMD = *msg;

        // Inverting the right side so forward shows the same direction on both sides of the drawing
        this->lastDriveCMD.cmd_r.x *= -1;
        this->lastDriveCMD.cmd_r.y *= -1;
        this->lastDriveCMD.cmd_r.z *= -1;
    };

    auto f2 = [this](const cross_pkg_messages::msg::ArmInputRaw::SharedPtr msg) {
        this->lastArmCMD = *msg;
    };

    auto f3 = [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        this->lastCmdVel = *msg;
    };

    sub = node_->create_subscription<cross_pkg_messages::msg::RoverComputerDriveCMD>("motorVels", 10, f);
    sub2 = node_->create_subscription<cross_pkg_messages::msg::ArmInputRaw>("/armInputRaw", 10, f2);
    cmdVelSub = node_->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, f3);

    joyMapParamClient_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "joy_mapper");

    // This runs before the window opens, so a long wait here delays the whole GUI (it used to be 10 s)
    if (!joyMapParamClient_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(node_->get_logger(), "joy_mapper not reachable yet; the Swap Joysticks toggle works once it starts");
    }
}

void TelemetryPanel::update()
{
}

TelemetryPanel::~TelemetryPanel()
{
}
