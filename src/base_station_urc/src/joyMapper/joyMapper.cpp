#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

class JoyMapper : public rclcpp::Node
{
public:
  JoyMapper() : Node("joy_mapper")
  {
    joy0_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy0", 10, std::bind(&JoyMapper::joy0Callback, this, std::placeholders::_1));

    joy1_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy1", 10, std::bind(&JoyMapper::joy1Callback, this, std::placeholders::_1));

    drive_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  void joy0Callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy0_msg_ = *msg;
    sendCommand();
  }

  void joy1Callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy1_msg_ = *msg;
    sendCommand();
  }

  void sendCommand()
  {
    if (last_joy0_msg_.axes.empty() || last_joy1_msg_.axes.empty())
      return;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = last_joy0_msg_.axes[1] * sensitivity_;
    cmd.angular.z = last_joy1_msg_.axes[1] * -sensitivity_;

    drive_pub_->publish(cmd);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy0_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy1_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drive_pub_;

  sensor_msgs::msg::Joy last_joy0_msg_;
  sensor_msgs::msg::Joy last_joy1_msg_;

  const double sensitivity_ = 0.6;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto joy_mapper = std::make_shared<JoyMapper>();
  rclcpp::spin(joy_mapper);
  rclcpp::shutdown();
  return 0;
}