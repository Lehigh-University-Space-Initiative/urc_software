//reference for formulas used in here: https://www.movable-type.co.uk/scripts/latlong.html

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cross_pkg_messages/msg/gps_data.hpp"

#include <cmath>
#include <algorithm>

// globals
cross_pkg_messages::msg::GPSData latestGPS{};
bool hasFix = false;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPublisher;
std::shared_ptr<rclcpp::Node> node;



void gpsCallback(const cross_pkg_messages::msg::GPSData::SharedPtr msg) {
    latestGPS = *msg;
    hasFix = true;
}

/// great-circle distance and initial bearing from (lat1,lon1)(our startiing point) to (lat2,lon2)(goal point), in degrees.
/// Haversine formula for distance, standard initial-bearing formula for bearing - both treat WGS84 lat/lon as points on a sphere, which is accurate enough at the <2km range
void computeBearingAndDistance(double lat1, double lon1, double lat2, double lon2,
                                double &outDistanceMeters, double &outBearingDegrees) {
    constexpr double EARTH_RADIUS_M = 6371000.0;

    double phi1 = lat1 * M_PI / 180.0;
    double phi2 = lat2 * M_PI / 180.0;
    double dPhi = (lat2 - lat1) * M_PI / 180.0;
    double dLambda = (lon2 - lon1) * M_PI / 180.0;

    // haversine computation (good to look at reference link at the top of the document for better understanding of this stuff)
    // formula: a = sin²(Δφ/2) + cosφ1·cosφ2 · sin²(Δλ/2); d = 2 · atan2(√a, √(a-1))
    // definition: calculates the shortest great-circle distance between two points on a sphere (like Earth) using their latitudes and longitudes
    double a = std::sin(dPhi / 2.0) * std::sin(dPhi / 2.0) +
               std::cos(phi1) * std::cos(phi2) *
               std::sin(dLambda / 2.0) * std::sin(dLambda / 2.0);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    outDistanceMeters = EARTH_RADIUS_M * c;

    // initial bearing, normalized to 0-360
    double y = std::sin(dLambda) * std::cos(phi2);
    double x = std::cos(phi1) * std::sin(phi2) -
               std::sin(phi1) * std::cos(phi2) * std::cos(dLambda);
    double theta = std::atan2(y, x);
    outBearingDegrees = std::fmod((theta * 180.0 / M_PI) + 360.0, 360.0);
}

/// proportional controller: turn toward the target, drive forward scaled by distance,
/// and throttle back linear speed when heading error is large so we don't drive sideways.
geometry_msgs::msg::Twist computeDriveCommand(double distanceMeters, double bearingDegrees,
                                               double headingDegrees) {
    geometry_msgs::msg::Twist cmd{};

    const double maxLinearSpeed = 0.6;   // m/s
    const double maxAngularSpeed = 0.8;  // rad/s
    const double kAngular = 0.03;        // rad/s per degree of heading error
    const double kLinear = 0.3;          // (m/s) per meter of distance

    // Heading error normalized to [-180, 180] so we always turn the short way.
    double headingError = std::fmod(bearingDegrees - headingDegrees + 180.0, 360.0);
    if (headingError < 0) headingError += 360.0;
    headingError -= 180.0;

    cmd.angular.z = std::clamp(kAngular * headingError, -maxAngularSpeed, maxAngularSpeed);

    double headingErrorAbs = std::abs(headingError);
    double forwardScale = std::max(0.0, 1.0 - headingErrorAbs / 90.0); // 0 at 90+ deg off, 1 when on-heading
    cmd.linear.x = std::clamp(kLinear * distanceMeters, 0.0, maxLinearSpeed) * forwardScale;

    return cmd;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("WaypointFollower");

    RCLCPP_INFO(node->get_logger(), "WaypointFollower is running");

    node->declare_parameter<double>("target_lat", 0.0);
    node->declare_parameter<double>("target_lon", 0.0);
    node->declare_parameter<double>("arrival_radius_m", 3.0);

    cmdVelPublisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // GPSData.lla is a bare geometry_msgs/Vector3 (no named lat/lon/alt fields).
    // assuming lla.x = latitude, lla.y = longitude, lla.z = altitude -- confirm this
    // against whatever GPS driver ends up publishing to /gps_data.
    auto gpsSubscription = node->create_subscription<cross_pkg_messages::msg::GPSData>(
        "gps_data", 10, gpsCallback);

    rclcpp::Rate loop_rate(10); // 10 Hz because matches other nodes in this repo? definitely up for changing this

    while (rclcpp::ok()) {
        if (hasFix) {
            double targetLat = node->get_parameter("target_lat").as_double();
            double targetLon = node->get_parameter("target_lon").as_double();
            double arrivalRadius = node->get_parameter("arrival_radius_m").as_double();

            double distance, bearing;
            computeBearingAndDistance(latestGPS.lla.x, latestGPS.lla.y, targetLat, targetLon,
                                       distance, bearing);

            if (distance <= arrivalRadius) {
                cmdVelPublisher->publish(geometry_msgs::msg::Twist{}); // zero Twist = stop
                RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                                      "Arrived at target (%.2f m away)", distance);
            } else {
                // TODO: course-over-ground only means something while moving; at low/zero
                // speed this heading estimate degrades. Fine for a first pass, revisit later.
                double headingDegrees = latestGPS.course;
                auto cmd = computeDriveCommand(distance, bearing, headingDegrees);
                cmdVelPublisher->publish(cmd);
            }
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
