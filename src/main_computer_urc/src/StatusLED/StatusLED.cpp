#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("StatusLED");

    RCLCPP_INFO(node->get_logger(), "StatusLED startup");

    rclcpp::Rate loop_rate(10);

    int status = 0;
    //if at waypoint this is used to blink light
    bool lightOn = false;

    // Declare the roverStatus parameter with a default value of 0
    node->declare_parameter<int>("roverStatus", 0);

    while (rclcpp::ok()) {
        //check paramter value of /roverStatus
        //this is an int which could have the following values:
        //0 - error
        //1 - manual
        //2 - autonomous
        //3 - autonomous at waypoint
        //if the paramater is not set, set it to 0 (error)

        // Check if the parameter exists and retrieve it
        if (node->get_parameter("roverStatus", status)) {
            RCLCPP_INFO(node->get_logger(), "Status: %d", status);
        } else {
            RCLCPP_WARN(node->get_logger(), "Status not set, setting to error");
            node->set_parameter(rclcpp::Parameter("roverStatus", 0));
        }

        //the light colors: red autonomus, blue teleop, flashing green on suc arrival 

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
