#include "TelemetryPanel.h"

void TelemetryPanel::drawBody()
{
    ImGui::Text("Motor Inputs");

    ImGui::Separator();

    ImGui::Text("Left Motors");
    
    auto l1_n = lastDriveCMD.CMD_L.x < 0;
    auto l2_n = lastDriveCMD.CMD_L.y < 0;
    auto l3_n = lastDriveCMD.CMD_L.z < 0;
    auto r1_n = lastDriveCMD.CMD_R.x < 0;
    auto r2_n = lastDriveCMD.CMD_R.y < 0;
    auto r3_n = lastDriveCMD.CMD_R.z < 0;


    if (l1_n)
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, IM_COL32(255,0,0,255));
    ImGui::ProgressBar(abs(lastDriveCMD.CMD_L.x)); 
    if (l1_n)
        ImGui::PopStyleColor();

    if (l2_n)
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, IM_COL32(255,0,0,255));
    ImGui::ProgressBar(abs(lastDriveCMD.CMD_L.y)); 
    if (l2_n)
        ImGui::PopStyleColor();

    if (l3_n)
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, IM_COL32(255,0,0,255));
    ImGui::ProgressBar(abs(lastDriveCMD.CMD_L.z)); 
    if (l3_n)
        ImGui::PopStyleColor();

    ImGui::Text("Right Motors");

    if (r1_n)
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, IM_COL32(255,0,0,255));
    ImGui::ProgressBar(abs(lastDriveCMD.CMD_R.x)); 
    if (r1_n)
        ImGui::PopStyleColor();

    if (r2_n)
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, IM_COL32(255,0,0,255));
    ImGui::ProgressBar(abs(lastDriveCMD.CMD_R.y));
    if (r2_n)
        ImGui::PopStyleColor();

    if (r3_n)
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, IM_COL32(255,0,0,255));
    ImGui::ProgressBar(abs(lastDriveCMD.CMD_R.z));
    if (r3_n)
        ImGui::PopStyleColor();


    ImGui::Separator();

    ImGui::Text("Arm Inputs");

    ImGui::Text("Left Stick");

    ImGui::Text("Pitch");
    ImGui::SameLine();
    ImGui::ProgressBar(lastArmCMD.CMD_L.x *0.5 + 0.5);

    ImGui::Text("Base");
    ImGui::SameLine();
    ImGui::ProgressBar(lastArmCMD.CMD_L.y *0.5 + 0.5);

    ImGui::Text("Wrist Pitch");
    ImGui::SameLine();
    ImGui::ProgressBar(lastArmCMD.CMD_L.z *0.5 + 0.5);

    ImGui::Separator();

    ImGui::Text("Right Stick");

    ImGui::Text("Elbow Pitch");
    ImGui::SameLine();
    ImGui::ProgressBar(lastArmCMD.CMD_R.x *0.5 + 0.5);

    ImGui::Text("Wrist Rotate");
    ImGui::SameLine();
    ImGui::ProgressBar(lastArmCMD.CMD_R.y *0.5 + 0.5);

    ImGui::Text("Wrist Rotate");
    ImGui::SameLine();
    ImGui::ProgressBar(lastArmCMD.CMD_R.z *0.5 + 0.5);
}

void TelemetryPanel::setup()
{
    auto f = boost::function<void(const cross_pkg_messages::RoverComputerDriveCMDConstPtr&)>([this](auto p){

        this->lastDriveCMD = *p;

        //invert right side
        this->lastDriveCMD.CMD_R.x *= -1;
        this->lastDriveCMD.CMD_R.y *= -1;
        this->lastDriveCMD.CMD_R.z *= -1;
    });
    auto f2 = boost::function<void(const cross_pkg_messages::RoverComputerDriveCMDConstPtr&)>([this](auto p){
        this->lastArmCMD = *p;
    });
    sub = n.subscribe("roverDriveCommands", 10, f);
    sub2 = n.subscribe("manualArmControl", 10, f2);
}

void TelemetryPanel::update()
{
}

TelemetryPanel::~TelemetryPanel()
{
}
