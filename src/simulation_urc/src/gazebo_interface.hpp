#include <string>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class GazeboInterface : public rclcpp::Node {
public:
    GazeboInterface() : Node("gazebo_interface") {
        RCLCPP_INFO(this->get_logger(), "GazeboInterface node has been initialized.");
    }
private:

}
