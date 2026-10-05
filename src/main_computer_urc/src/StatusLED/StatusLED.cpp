/**
 * @file
 * StatusLED node: placeholder for the rover's status light (not yet implemented)
 *
 * Runs on: the main computer (built and installed, but commented out of main_computer_launch.py)
 *
 * Planned behavior (from the original notes; no light is driven yet):
 *   - Read the roverStatus parameter: 0 = error, 1 = manual, 2 = autonomous, 3 = autonomous at waypoint
 *   - Show red for autonomous, blue for teleoperation, and flashing green on successful arrival
 *
 * These colors match URC's required autonomy status indicator, so this is a likely candidate to finish rather than delete
 */
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("StatusLED");

    RCLCPP_INFO(node->get_logger(), "StatusLED startup");

    rclcpp::Rate loop_rate(10);

    int status = 0;
    bool lightOn = false;  // Unused; intended for blinking the light at a waypoint

    node->declare_parameter<int>("roverStatus", 0);

    while (rclcpp::ok()) {
        // Reading roverStatus, and resetting it to 0 (error) if it somehow can't be read
        if (!node->get_parameter("roverStatus", status)) {
            node->set_parameter(rclcpp::Parameter("roverStatus", 0));
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
