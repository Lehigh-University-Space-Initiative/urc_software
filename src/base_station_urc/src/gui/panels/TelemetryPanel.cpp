#include "TelemetryPanel.h"
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>


void signedProgressBar(float val, std::string overlay = "", ImVec2 barSize = {0,0}) {
    auto negative = val < 0;
    auto mag = abs(val);
    if (negative)
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, IM_COL32(255, 0, 0, 255));
    if (barSize.x == 0 && barSize.y == 0)
        ImGui::ProgressBar(mag,{-10,0},overlay.c_str()); 
    else
        ImGui::ProgressBar(mag,barSize, overlay.c_str()); 
    if (negative)
        ImGui::PopStyleColor();
}

void TelemetryPanel::drawBody() {
    ImGui::Text("Driveline Telemetry");

    ImGui::Separator();

    // auto roverColor =

    {
        ImDrawList *draw_list = ImGui::GetWindowDrawList();

        const ImVec2 p = ImGui::GetCursorScreenPos();
        auto win_size = ImGui::GetWindowSize();
        float outlineThickness = 2;
        auto outlineColor = ImU32(ImColor(200, 200, 0));

        ImVec2 roverBoxSize = {80, 150};
        ImVec2 roverBoxStart = {p.x + win_size.x / 2 - roverBoxSize.x / 2, p.y + 25};
        draw_list->AddRect(roverBoxStart, {roverBoxStart.x + roverBoxSize.x, roverBoxStart.y + roverBoxSize.y}, outlineColor, 0, 0, outlineThickness);

        for (size_t side = 0; side < 2; side++)
        {
            for (size_t wheel = 0; wheel < 3; wheel++)
            {
                float wheelRad = 20;
                ImVec2 wheelSize = {10, wheelRad};
                ImVec2 wheelCenter = {roverBoxStart.x - wheelSize.x, roverBoxStart.y + wheelRad + wheel * roverBoxSize.y / 3};
                if (side)
                {
                    wheelCenter.x += roverBoxSize.x + 2 * wheelSize.x;
                }

                // draw_list->AddCircle(wheelCenter,wheelRad,outlineColor,0,outlineThickness);
                ImVec2 wheelMin = {wheelCenter.x - wheelSize.x / 2, wheelCenter.y - wheelSize.y / 2};
                ImVec2 wheelMax = {wheelCenter.x + wheelSize.x / 2, wheelCenter.y + wheelSize.y / 2};
                draw_list->AddRectFilled(wheelMin, wheelMax, outlineColor);

                ImVec2 sliderSize = {50, wheelSize.y};
                ImVec2 sliderStart = {wheelCenter.x - 30 - sliderSize.x, wheelCenter.y - sliderSize.y / 2};
                if (side)
                {
                    sliderStart.x = wheelCenter.x + 30;
                }
                ImGui::SetCursorScreenPos(sliderStart);
                auto sliderSide = side ? lastDriveCMD.cmd_r : lastDriveCMD.cmd_l;
                float sliderVal = 0;

                if (wheel == 0)
                {
                    sliderVal = sliderSide.x;
                }
                else if (wheel == 1)
                {
                    sliderVal = sliderSide.y;
                }
                else
                {
                    sliderVal = sliderSide.z;
                }

                signedProgressBar(sliderVal / 25, " ", sliderSize);
            }
        }

        ImGui::SetCursorScreenPos(p);
        ImGui::Dummy({10, roverBoxSize.y + 50});

        ImGui::Text("Linear Velocity CMD: %.2f m/s", lastCmdVel.linear.x);
        signedProgressBar(lastCmdVel.linear.x / 1, " ");
        ImGui::Text("Angular Velocity CMD: %.1f deg/s", lastCmdVel.angular.y);
        signedProgressBar(lastCmdVel.angular.y / 120, " ");

        static bool joy_swap = true;

        if (ImGui::Checkbox("Swap Joysticks", &joy_swap))
        {
            joyMapParamClient_->set_parameters({rclcpp::Parameter("swap_joysticks", joy_swap)},
                                               [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future)
                                               {
                                                   future.wait();
                                                   auto results = future.get();
                                                   if (results.size() != 1)
                                                   {
                                                       RCLCPP_ERROR_STREAM(node_->get_logger(), "expected 1 result, got " << results.size());
                                                   }
                                                   else
                                                   {
                                                       if (results[0].successful)
                                                       {
                                                           RCLCPP_INFO(node_->get_logger(), "success");
                                                       }
                                                       else
                                                       {
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
        ImDrawList *draw_list = ImGui::GetWindowDrawList();

        const ImVec2 p = ImGui::GetCursorScreenPos();
        auto win_size = ImGui::GetWindowSize();
        float outlineThickness = 2;
        auto outlineColor = ImU32(ImColor(200, 200, 0));

        ImVec2 roverBoxSize = {20, 150};
        ImVec2 roverBoxStart = {p.x + win_size.x / 2 - roverBoxSize.x / 2, p.y + 25};
        // draw_list->AddRect(roverBoxStart, {roverBoxStart.x + roverBoxSize.x, roverBoxStart.y + roverBoxSize.y}, outlineColor, 0, 0, outlineThickness);

        for (size_t side = 0; side < 2; side++)
        {
            for (size_t wheel = 0; wheel < 3; wheel++)
            {
                float wheelRad = 20;
                ImVec2 wheelSize = {10, wheelRad};
                ImVec2 wheelCenter = {roverBoxStart.x - wheelSize.x, roverBoxStart.y + wheelRad + wheel * roverBoxSize.y / 3};
                if (side)
                {
                    wheelCenter.x += roverBoxSize.x + 2 * wheelSize.x;
                }


                ImVec2 sliderSize = {50, wheelSize.y};
                ImVec2 sliderStart = {wheelCenter.x - 30 - sliderSize.x, wheelCenter.y - sliderSize.y / 2};
                if (side)
                {
                    sliderStart.x = wheelCenter.x + 30;
                }
                ImGui::SetCursorScreenPos(sliderStart);
                auto sliderSide = side ? lastArmCMD.angular_input : lastArmCMD.linear_input;
                float sliderVal = 0;

                if (wheel == 0)
                {
                    sliderVal = sliderSide.x;
                }
                else if (wheel == 1)
                {
                    sliderVal = sliderSide.y;
                }
                else
                {
                    sliderVal = sliderSide.z;
                }

                signedProgressBar(sliderVal, " ", sliderSize);
            }
        }
    }
}

void TelemetryPanel::setup() {

    //setup parameter setting
    joyMapParamClient_ = std::make_shared<rclcpp::AsyncParametersClient>(node_,"joy_mapper");

    // Wait for the service to become available
    if (!joyMapParamClient_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_ERROR(node_->get_logger(), "SetParameters service not available");
      return;
    }

    auto f = [this](const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg) {
        this->lastDriveCMD = *msg;

        // Invert right side
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

    // sub = node_->create_subscription<cross_pkg_messages::msg::RoverComputerDriveCMD>(
    //     "roverDriveCommands", 10, f);
    sub = node_->create_subscription<cross_pkg_messages::msg::RoverComputerDriveCMD>(
        "motorVels", 10, f);
    sub2 = node_->create_subscription<cross_pkg_messages::msg::ArmInputRaw>(
        "/armInputRaw", 10, f2);

    cmdVelSub = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,f3);

}

void TelemetryPanel::update() {
    // Add any update logic if needed
}

TelemetryPanel::~TelemetryPanel() {
}
