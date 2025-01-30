#include <libgpsmm.h>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <gps.h>

#include "rclcpp/rclcpp.hpp"
#include "cross_pkg_messages/msg/gps_data.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("gps_driver");

    RCLCPP_INFO(node->get_logger(), "GPS Driver");

    auto gpsPublisher = node->create_publisher<cross_pkg_messages::msg::GPSData>("/gps_data", 10);

    gpsmm gps_rec("localhost", DEFAULT_GPSD_PORT);

    if (gps_rec.stream(WATCH_ENABLE | WATCH_JSON) == nullptr) {
        RCLCPP_ERROR(node->get_logger(), "No GPSD running.");
        return 1;
    }

    rclcpp::Rate loop_rate(30);  // 30 Hz loop rate
    constexpr auto kWaitingTime{1000000};  // 1 second

    while (rclcpp::ok()) {
        if (!gps_rec.waiting(kWaitingTime)) {
            continue;
        }

        struct gps_data_t* gpsd_data;

        if ((gps_rec.read()) == nullptr) {
            RCLCPP_ERROR(node->get_logger(), "GPSD read error.");
            return 1;
        }

        while (((gpsd_data = gps_rec.read()) == nullptr) || (gpsd_data->fix.mode < MODE_2D)) {
            // Busy wait mitigation
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        const auto latitude{gpsd_data->fix.latitude};
        const auto longitude{gpsd_data->fix.longitude};
        const auto alt(gpsd_data->fix.altitude);
        const auto speed(gpsd_data->fix.speed);
        const auto course(gpsd_data->fix.track);

        const auto sats(gpsd_data->satellites_used);
        const auto lat_acc(gpsd_data->fix.epy);
        const auto lon_acc(gpsd_data->fix.epx);
        const auto alt_acc(gpsd_data->fix.epv);

        cross_pkg_messages::msg::GPSData gpsData;
        gpsData.status = gpsd_data->fix.mode;
        gpsData.lla.x = latitude;
        gpsData.lla.y = longitude;
        gpsData.lla.z = alt;
        gpsData.speed = speed;
        gpsData.course = course;

        gpsData.sats = sats;
        gpsData.lla_acc.x = lat_acc;
        gpsData.lla_acc.y = lon_acc;
        gpsData.lla_acc.z = alt_acc;

        gpsPublisher->publish(gpsData);

        loop_rate.sleep();  // Maintain the loop rate
    }

    rclcpp::shutdown();
    return 0;
}
