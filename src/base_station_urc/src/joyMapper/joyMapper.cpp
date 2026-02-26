#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <cross_pkg_messages/msg/rover_computer_arm_cmd.hpp>

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
    arm_pub_ = this->create_publisher<cross_pkg_messages::msg::RoverComputerArmCMD>("/roverArmCommands", 10);

    this->declare_parameter("swap_joysticks", true);
  }

private:
  void joy0Callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy0_msg_ = *msg;
    sendCommand();
    sendArmCommand();
  }

  void joy1Callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy1_msg_ = *msg;
    sendCommand();
    sendArmCommand();
  }

  void sendCommand()
  {
    if (last_joy0_msg_.axes.empty() || last_joy1_msg_.axes.empty())
      return;

    // Rover Centric Coordinate System: +X is rover front, +Y is rover top, Right Handed
    geometry_msgs::msg::Twist cmd;

    auto joy_left = last_joy0_msg_.axes[1];
    auto joy_right = last_joy1_msg_.axes[1];

    auto joy_left_twist = last_joy0_msg_.axes[2];
    auto joy_right_twist = last_joy1_msg_.axes[2];

    auto flip_joystics = this->get_parameter("swap_joysticks");
    if (flip_joystics.as_bool()) {
      std::swap(joy_left,joy_right);
    }
    
    cmd.linear.x = (joy_left + joy_right) / 2 * linearSensativity;
    cmd.angular.y = (joy_right - joy_left) / 2 * angularSensativity;
    
    // Independent steering: left joystick twist -> motor 5, right joystick twist -> motor 6
    const double deadzone = 0.1; // Deadzone for twist input
    double twistScaleLeft = 0.1; // Scale down twist input to prevent excessive speeds
    double twistScaleRight = 0.1; // Scale down twist input to prevent excessive speeds
    if (std::abs(joy_left_twist) < deadzone) twistScaleLeft = 0; // Deadband for left twist
    if (std::abs(joy_right_twist) < deadzone) twistScaleRight = 0; // Deadband for right twist
    cmd.linear.y = joy_left_twist * twistScaleLeft;  // Left joystick twist -> motor 5
    cmd.angular.z = joy_right_twist * twistScaleRight;  // Right joystick twist -> motor 6

    drive_pub_->publish(cmd);
  }

  // 55: base linear actuator (not exist yet)
  // 51: shoulder
  // 52: wrist roll
  // 53: elbow
  // 57: wrist pitch
  // 54: end effector

  void sendArmCommand()
  {
    if (last_joy0_msg_.buttons.empty() && last_joy1_msg_.buttons.empty())
      return;

    cross_pkg_messages::msg::RoverComputerArmCMD arm_cmd;
    const double armPower = 0.05;  // Very slow arm power (5%)
    bool anyButtonPressed = false;

    // Button mapping (Logitech Extreme 3D Pro):
    // Axis 4 (mini joystick left/right): Base (CAN 55) - CW/CCW
    // Buttons 6-7: Shoulder (CAN 51) - CW/CCW
    // Buttons 8-9: Elbow (CAN 53) - CW/CCW
    // Buttons 4-5: Wrist Roll (CAN 52) - CW/CCW
    // Buttons 2-3: Wrist Pitch (CAN 57) - CW/CCW
    // Buttons 0-1: End Effector (CAN 54) - Close/Open

    // Base motor (CAN 55)
    // Axis 4 is left/right with the mini thumb joystick
    if (last_joy0_msg_.axes.size() > 4) {
      auto base = last_joy0_msg_.axes[4];
      if (base) {
        arm_cmd.cmd_b = base * armPower; // Axis 4 (mini joystick left/right) controls base linear actuator
        anyButtonPressed = true;
      }
    }

    // Shoulder motor (CAN 51) 
    // Button 6 is labeled with 7 on the joystick
    // Button 7 is labeled with 8 on the joystick
    if (last_joy0_msg_.buttons.size() > 7) {
      auto shoulder = last_joy0_msg_.buttons[6] - last_joy0_msg_.buttons[7];
      if (shoulder) {
        arm_cmd.cmd_s = shoulder * armPower;     // Button 6 = CW, Button 7 = CCW
        anyButtonPressed = true;
      }
    }

    // Elbow motor (CAN 53)
    // Button 8 is labeled with 9 on the joystick
    // Button 9 is labeled with 10 on the joystick
    if (last_joy0_msg_.buttons.size() > 9) {
      auto elbow = last_joy0_msg_.buttons[8] - last_joy0_msg_.buttons[9];
      if (elbow) {
        arm_cmd.cmd_e = elbow * 0.25;     // Button 8 = CW, Button 9 = CCW
        anyButtonPressed = true;
      }
    }

    // Wrist Roll (CAN 52) - using cmd_wr
    // Button 4 is labeled with 5 on the joystick
    // Button 5 is labeled with 6 on the joystick
    if (last_joy0_msg_.buttons.size() > 5) {
      auto wrist_roll = last_joy0_msg_.buttons[4] - last_joy0_msg_.buttons[5];
      if (wrist_roll) {
        arm_cmd.cmd_wr = wrist_roll * armPower;     // Button 4 = CW, Button 5 = CCW
        anyButtonPressed = true;
      }
    }

    // Wrist Pitch (CAN 57) - using cmd_wp
    // Button 2 is labeled with 3 on the joystick
    // Button 3 is labeled with 4 on the joystick
    if (last_joy0_msg_.buttons.size() > 3) {
      auto wrist_pitch = last_joy0_msg_.buttons[2] - last_joy0_msg_.buttons[3];
      if (wrist_pitch) {
        arm_cmd.cmd_wp = wrist_pitch * armPower;     // Button 2 = CW, Button 3 = CCW
        anyButtonPressed = true;
      }
    }

      // End Effector (CAN 54) - using cmd_endeff
      // Button 0 is labeled with 1 on the joystick
      // Button 1 is labeled with 2 on the joystick
      if (last_joy0_msg_.buttons.size() > 1) {
        auto end_eff = last_joy0_msg_.buttons[0] - last_joy0_msg_.buttons[1];
        if (end_eff) {
          arm_cmd.cmd_endeff = end_eff * armPower;     // Button 0 = close, Button 1 = open
          anyButtonPressed = true;
        }
      }

    // Only publish if at least one button is pressed
    if (anyButtonPressed) {
      arm_pub_->publish(arm_cmd);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy0_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy1_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drive_pub_;
  rclcpp::Publisher<cross_pkg_messages::msg::RoverComputerArmCMD>::SharedPtr arm_pub_;

  sensor_msgs::msg::Joy last_joy0_msg_;
  sensor_msgs::msg::Joy last_joy1_msg_;

  
  // 100% forward thottle should be this speed m/s
  const double linearSensativity = -1;
  // 100% twist throttle should be this speed in deg/s
  const double angularSensativity = 120;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto joy_mapper = std::make_shared<JoyMapper>();
  rclcpp::spin(joy_mapper);
  rclcpp::shutdown();
  return 0;
}