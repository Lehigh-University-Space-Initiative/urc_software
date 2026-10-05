/**
 * @file
 * WaypointFollower node: drives the rover toward one GNSS (GPS) waypoint
 *
 * Runs on: the main computer on the rover, or anywhere in simulation
 * Started by: navigation_launch.py (with real GPS) or navigation_sim_launch.py (run mode "nav_sim", with fake GPS)
 *
 * Subscribes:
 *   gps_data (cross_pkg_messages/GPSData) - current position (lla.x = latitude, lla.y = longitude) and course
 *
 * Publishes:
 *   cmd_vel (geometry_msgs/Twist) - linear.x forward speed in m/s, angular.z turn rate in rad/s (counterclockwise positive)
 *
 * Parameters:
 *   target_lat, target_lon (degrees) - the waypoint; arrival_radius_m (meters) - how close counts as arrived
 *
 * How it connects to the system:
 *   - In simulation, fake_gps_node.py reads this node's cmd_vel and publishes where the rover would be
 *   - On the rover, cmd_vel goes to DriveTrainManager, which turns with angular.y in deg/s instead of angular.z
 *   - So on real hardware this node's turn commands are currently ignored (see "Known integration issues" in the README)
 *
 * Reference for the formulas: https://www.movable-type.co.uk/scripts/latlong.html
 */
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cross_pkg_messages/msg/gps_data.hpp"

#include <cmath>
#include <algorithm>

cross_pkg_messages::msg::GPSData latestGPS{};  // Most recent GPS message
bool hasFix = false;  // True once the first GPS message arrives (no driving before then)
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPublisher;
std::shared_ptr<rclcpp::Node> node;

/// Store the latest GPS message
void gpsCallback(const cross_pkg_messages::msg::GPSData::SharedPtr msg)
{
    latestGPS = *msg;
    hasFix = true;
}

/**
 * Compute the great-circle distance and initial compass bearing from one GNSS point to another
 *
 * Treats the Earth as a sphere, which is accurate enough at URC distances (under ~2 km)
 *
 * Parameters (inputs):
 *   lat1, lon1 - starting point (the rover) in degrees
 *   lat2, lon2 - goal point (the waypoint) in degrees
 *   outDistanceMeters - set to the distance in meters (output)
 *   outBearingDegrees - set to the compass bearing in degrees, 0 = north, 90 = east (output)
 *
 * Steps:
 *   1. Convert the latitudes and the latitude/longitude differences to radians
 *   2. Haversine formula for distance
 *   3. Initial-bearing formula, normalized from (-180, 180] to [0, 360)
 */
// Syntax: "double &outDistanceMeters" is a reference parameter, so assigning to it changes the caller's variable
void computeBearingAndDistance(double lat1, double lon1, double lat2, double lon2, double& outDistanceMeters, double& outBearingDegrees)
{
    // Syntax: constexpr means the value is fixed at compile time
    constexpr double EARTH_RADIUS_M = 6371000.0;  // Mean Earth radius in meters

    // Trig functions take radians; degrees * pi / 180 converts
    double phi1 = lat1 * M_PI / 180.0;
    double phi2 = lat2 * M_PI / 180.0;
    double dPhi = (lat2 - lat1) * M_PI / 180.0;
    double dLambda = (lon2 - lon1) * M_PI / 180.0;

    // Haversine: a = sin^2(dPhi/2) + cos(phi1) * cos(phi2) * sin^2(dLambda/2); c = 2 * atan2(sqrt(a), sqrt(1-a)); d = R * c
    // It gives the shortest distance along the Earth's surface between two latitude/longitude points
    double a = std::sin(dPhi / 2.0) * std::sin(dPhi / 2.0) + std::cos(phi1) * std::cos(phi2) * std::sin(dLambda / 2.0) * std::sin(dLambda / 2.0);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    outDistanceMeters = EARTH_RADIUS_M * c;

    // Initial bearing: theta = atan2(sin(dLambda) * cos(phi2), cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(dLambda))
    double y = std::sin(dLambda) * std::cos(phi2);
    double x = std::cos(phi1) * std::sin(phi2) - std::sin(phi1) * std::cos(phi2) * std::cos(dLambda);
    double theta = std::atan2(y, x);
    // Adding 360 then taking the remainder maps negative angles (west of north) into [0, 360)
    outBearingDegrees = std::fmod((theta * 180.0 / M_PI) + 360.0, 360.0);
}

/**
 * Proportional controller: turn toward the target and drive forward, slowing down when pointed the wrong way
 *
 * Parameters (inputs):
 *   distanceMeters - distance to the target
 *   bearingDegrees - compass bearing to the target
 *   headingDegrees - compass direction the rover is currently moving
 *
 * Return value:
 *   the drive command (linear.x in m/s, angular.z in rad/s)
 *
 * Steps:
 *   1. Heading error = bearing - heading, wrapped into [-180, 180] so the rover always turns the short way
 *   2. Turn rate proportional to the heading error, clamped, and sign-flipped into ROS's counterclockwise-positive angular.z
 *   3. Forward speed proportional to distance, scaled down to 0 as the heading error reaches 90 degrees
 */
geometry_msgs::msg::Twist computeDriveCommand(double distanceMeters, double bearingDegrees, double headingDegrees)
{
    geometry_msgs::msg::Twist cmd{};

    const double maxLinearSpeed = 0.6;  // m/s
    const double maxAngularSpeed = 0.8;  // rad/s
    const double kAngular = 0.03;  // rad/s per degree of heading error
    const double kLinear = 0.3;  // (m/s) per meter of distance

    double headingError = std::fmod(bearingDegrees - headingDegrees + 180.0, 360.0);
    // Syntax: std::fmod keeps the sign of its first argument, so a negative result needs +360 to wrap
    if (headingError < 0) {
        headingError += 360.0;
    }
    headingError -= 180.0;

    // Negating to convert between conventions (the minus sign below)
    // A positive compass heading error means the target is clockwise, to the right
    // ROS's angular.z is counterclockwise-positive (REP 103), so turning right needs a negative angular.z
    // Without the minus sign the rover turns away from the target and settles facing directly opposite it
    // Syntax: std::clamp(value, low, high) limits value to the range [low, high]
    cmd.angular.z = -std::clamp(kAngular * headingError, -maxAngularSpeed, maxAngularSpeed);

    double headingErrorAbs = std::abs(headingError);
    double forwardScale = std::max(0.0, 1.0 - headingErrorAbs / 90.0);  // 0 at 90+ degrees off, 1 when on heading
    cmd.linear.x = std::clamp(kLinear * distanceMeters, 0.0, maxLinearSpeed) * forwardScale;

    return cmd;
}

/**
 * Steps:
 *   1. Start ROS, declare the waypoint parameters, and set up the publisher and GPS subscription
 *   2. Loop at 10 Hz: once a GPS fix exists, compute distance and bearing to the target
 *   3. Inside the arrival radius, publish a stop command; otherwise publish a drive command toward the target
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("WaypointFollower");

    RCLCPP_INFO(node->get_logger(), "WaypointFollower is running");

    // Syntax: declare_parameter<double>(name, default) registers a parameter that launch files or the CLI can set
    node->declare_parameter<double>("target_lat", 0.0);
    node->declare_parameter<double>("target_lon", 0.0);
    node->declare_parameter<double>("arrival_radius_m", 3.0);

    // System: DriveTrainManager on the main computer (real rover) or fake_gps_node.py (simulation) reads this
    cmdVelPublisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // GPSData.lla is a bare geometry_msgs/Vector3 (no named lat/lon/alt fields)
    // Assuming lla.x = latitude, lla.y = longitude, lla.z = altitude; confirm against whatever GPS driver publishes /gps_data
    auto gpsSubscription = node->create_subscription<cross_pkg_messages::msg::GPSData>("gps_data", 10, gpsCallback);

    rclcpp::Rate loop_rate(10);  // 10 Hz to match other nodes in this repo; up for changing

    while (rclcpp::ok()) {
        if (hasFix) {
            // Reading the parameters every loop, so they can be changed while running (ros2 param set ...)
            double targetLat = node->get_parameter("target_lat").as_double();
            double targetLon = node->get_parameter("target_lon").as_double();
            double arrivalRadius = node->get_parameter("arrival_radius_m").as_double();

            double distance, bearing;
            computeBearingAndDistance(latestGPS.lla.x, latestGPS.lla.y, targetLat, targetLon, distance, bearing);

            if (distance <= arrivalRadius) {
                cmdVelPublisher->publish(geometry_msgs::msg::Twist{});  // A zero Twist means stop
                RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000, "Arrived at target (%.2f m away)", distance);
            }
            else {
                // TODO: course over ground only means something while moving; at low or zero speed this heading estimate degrades
                double headingDegrees = latestGPS.course;
                auto cmd = computeDriveCommand(distance, bearing, headingDegrees);
                cmdVelPublisher->publish(cmd);

                // Watch this while testing: distance should trend down toward arrival_radius, not oscillate or climb
                // If it oscillates, kAngular/kLinear in computeDriveCommand are too aggressive
                RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "distance=%.2fm bearing=%.1f heading=%.1f cmd(lin=%.2f ang=%.2f)", distance, bearing, headingDegrees, cmd.linear.x, cmd.angular.z);
            }
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
