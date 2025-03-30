#include "LUSIVisionTelem.h"
#include <chrono>

std::atomic<int> numClientsConnected{0};
std::atomic<int> numStreamClientsConnected{0};

LUSIVIsionGenerator::LUSIVIsionGenerator(rclcpp::Node::SharedPtr node)
{
    auto f = [this](const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr p) {
    this->lastDriveCMD = *p;
    };
    auto f2 = [this](const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr p) {
    this->lastArmCMD = *p;
    };
    this->sub = node->create_subscription<cross_pkg_messages::msg::RoverComputerDriveCMD>(
        "roverDriveCommands", rclcpp::QoS(10), f);
    this->sub2 = node->create_subscription<cross_pkg_messages::msg::RoverComputerDriveCMD>(
        "manualArmControl", rclcpp::QoS(10), f2);

    // gps
    auto f3 = [this](const cross_pkg_messages::msg::GPSData::SharedPtr p) {
    this->lastGPS = *p;
    };
    this->sub3 = node->create_subscription<cross_pkg_messages::msg::GPSData>(
        "/gps_data", rclcpp::QoS(10), f3);

    node->declare_parameter<bool>("hootl", false);
    if (!node->get_parameter("hootl", hootl)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get hootl param");
    }
}

LUSIVisionTelem LUSIVIsionGenerator::generate()
{
  LUSIVisionTelem telem{};

  telem.softwareInTheLoopTestMode = hootl ? 1 : 0;
  telem.controlScheme = 0;
  telem.operationMode = 0;

  telem.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();

  telem.driveInputsLeft[0] = lastDriveCMD.cmd_l.x;
  telem.driveInputsLeft[1] = lastDriveCMD.cmd_l.y;
  telem.driveInputsLeft[2] = lastDriveCMD.cmd_l.z;

  telem.driveInputsRight[0] = -lastDriveCMD.cmd_r.x;
  telem.driveInputsRight[1] = -lastDriveCMD.cmd_r.y;
  telem.driveInputsRight[2] = -lastDriveCMD.cmd_r.z;

  telem.armInputs[0] = lastArmCMD.cmd_l.x;
  telem.armInputs[1] = lastArmCMD.cmd_l.y;
  telem.armInputs[2] = lastArmCMD.cmd_l.z;
  telem.armInputs[3] = lastArmCMD.cmd_r.x;
  telem.armInputs[4] = lastArmCMD.cmd_r.y;
  telem.armInputs[5] = lastArmCMD.cmd_r.z;

  telem.numClientsConnected = numClientsConnected.load();
  telem.numStreamClientsConnected = numStreamClientsConnected.load();

  telem.gpsLLA[0] = lastGPS.lla.x;
  telem.gpsLLA[1] = lastGPS.lla.y;
  telem.gpsLLA[2] = lastGPS.lla.z;

  telem.gpsSatellites = lastGPS.sats;

  telem.course = lastGPS.course;
  telem.speed = lastGPS.speed;

  return telem;
}



