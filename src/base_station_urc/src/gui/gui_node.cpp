#include "gui_node.hpp"
#include <memory>

GuiNode::GuiNode() : Node("gui_node") {
joy_subscription_ = this->create_subscription

sensor_msgs::msg::Joy>(
"joy", 10, std::bind(&GuiNode::joy_callback, this, std::placeholders::_1));
}

void GuiNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
// Handle joystick messages and update GUI accordingly
}

int main(int argc, char **argv) {
rclcpp::init(argc, argv);
auto node = std::make_shared

GuiNode>();
rclcpp::spin(node);
rclcpp::shutdown();
return 0;
}