#include "SystemControlPanel.h"

void SystemControlPanel::drawBody()
{
    // ImGui::Text("Left Motors");
    // ImGui::ProgressBar(lastDriveCMD.CMD_L.x); 
    // ImGui::ProgressBar(lastDriveCMD.CMD_L.y); 
    // ImGui::ProgressBar(lastDriveCMD.CMD_L.z); 

    // ImGui::Text("Right Motors");
    // ImGui::ProgressBar(lastDriveCMD.CMD_R.x); 
    // ImGui::ProgressBar(lastDriveCMD.CMD_R.y); 
    // ImGui::ProgressBar(lastDriveCMD.CMD_R.z); 

    //software locks all motors to stop
    if (ImGui::Button("Motor Freeze")) {
        
    }

    ImGui::SameLine();

    int op_mode;
    ros::param::get("/op_mode", op_mode);

    if (op_mode) {
    if (ImGui::Button("Switch to Drive Control")) {
        ros::param::set("/op_mode", 0);
    }
    }
    else {
    if (ImGui::Button("Switch to Arm Control")) {
        ros::param::set("/op_mode", 1);
    }
    }

    if (ImGui::Button("Reboot")) {
        system("~/URC_2022/URC_DeployTools/reboot.sh");
    }

    ImGui::SameLine();

    if (ImGui::Button("Deploy Code")) {
        
    }
    
    ImGui::SameLine();

    if (ImGui::Button("Close Ground Station")) {
        close_ui = true;
    }

    int lusi_vision_mode;
    ros::param::get("/lusi_vision_mode", lusi_vision_mode);

    ImGui::PushStyleColor(ImGuiCol_Button,IM_COL32(10,90,230,255));

    if (lusi_vision_mode) {
    if (ImGui::Button("Enable 3D LUSI Vision")) {
        ros::param::set("/lusi_vision_mode", 0);
    }
    }
    else {
    if (ImGui::Button("Disable 3D LUSI Vision")) {
        ros::param::set("/lusi_vision_mode", 1);
    }
    }

    ImGui::PopStyleColor();

    if (ImGui::Button("Reboot")) {
        system("~/URC_2022/URC_DeployTools/reboot.sh");
    }


}

void SystemControlPanel::setup()
{
//     auto f = boost::function<void(const cross_pkg_messages::RoverComputerDriveCMDConstPtr&)>([this](auto p){
//         this->lastDriveCMD = *p;
//     });
//     sub = n.subscribe("roverDriveCommands", 10, f);
}

void SystemControlPanel::update()
{

}

SystemControlPanel::~SystemControlPanel()
{

}
