// gui_node.hpp
#ifndef GUI_NODE_HPP
#define GUI_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

class GuiNode : public rclcpp::Node {
public:
    GuiNode();
private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    rclcpp::Subscription
    sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
};

#endif // GUI_NODE_HPP