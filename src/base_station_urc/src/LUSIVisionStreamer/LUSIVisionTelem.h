/**
 * @file
 * Telemetry packet layout and generator for LUSI Vision (the team's external video/telemetry viewer)
 *
 * The packet is sent as raw bytes, so the client must use exactly the same struct layout (field order, types, padding)
 * Adding, removing, or reordering a field breaks existing clients until they are updated to match
 */
#pragma once
#include "stdio.h"

#include "rclcpp/rclcpp.hpp"
#include <sstream>

#include <opencv2/opencv.hpp>
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#include <cv_bridge/cv_bridge.h>

#include "sockpp/udp_socket.h"
#include "sockpp/tcp_acceptor.h"

#include "cs_plain_guarded.h"
#include <thread>

#include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"
#include "cross_pkg_messages/msg/gps_data.hpp"

#include <atomic>

// Syntax: std::atomic<int> can be safely read and changed from several threads at once without a mutex
// Defined in LUSIVisionTelem.cpp; counts of clients connected to the telemetry and video sockets
extern std::atomic<int> numClientsConnected;
extern std::atomic<int> numStreamClientsConnected;

/// Sent before each video frame: the byte size of the JPEG(s) that follow
struct LUSIStreamHeader {
    uint64_t sizeFrame1;  // Left/only eye
    uint64_t sizeFrame2;  // Right eye in stereo mode, 0 otherwise
};

/// One telemetry packet, sent 30 times a second to each telemetry client
struct LUSIVisionTelem {
    uint64_t timestamp;  // Milliseconds since the Unix epoch

    int controlScheme;  // 0 = driveline, 1 = arm (always 0 for now)
    int operationMode;  // 0 = manual, 1 = autonomous (always 0 for now)

    int numClientsConnected;  // Number of LUSI Vision telemetry clients connected

    float driveInputsLeft[3];  // Left wheel speeds: front, middle, back
    float driveInputsRight[3];  // Right wheel speeds: front, middle, back (sign flipped)

    float armInputs[6];  // x, y, z, roll, pitch, yaw

    float gpsLLA[3];  // Latitude, longitude, altitude
    float course;  // Heading over ground in degrees
    float speed;  // Ground speed
    int gpsSatellites;  // Number of satellites in the fix

    // Added in the "Feb 29" version of the packet
    int numStreamClientsConnected;
    int softwareInTheLoopTestMode;  // 1 in hootl mode
};

/**
 * Keeps the latest drive, arm, and GPS messages and packs them into a LUSIVisionTelem on demand
 */
class LUSIVIsionGenerator {
public:
    cross_pkg_messages::msg::RoverComputerDriveCMD lastDriveCMD{};
    cross_pkg_messages::msg::RoverComputerDriveCMD lastArmCMD{};
    cross_pkg_messages::msg::GPSData lastGPS{};

    bool hootl = false;

    rclcpp::Subscription<cross_pkg_messages::msg::RoverComputerDriveCMD>::SharedPtr sub;
    rclcpp::Subscription<cross_pkg_messages::msg::RoverComputerDriveCMD>::SharedPtr sub2;
    rclcpp::Subscription<cross_pkg_messages::msg::GPSData>::SharedPtr sub3;

    /**
     * Subscribe to the inputs and read the hootl parameter
     *
     * Parameters (inputs):
     *   node - the LUSIVisionStreamer node
     */
    LUSIVIsionGenerator(rclcpp::Node::SharedPtr node);

    /// Build a telemetry packet from the latest messages
    LUSIVisionTelem generate();
};
