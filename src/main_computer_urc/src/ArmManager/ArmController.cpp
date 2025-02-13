#include <controller_interface/controller_interface.hpp>
#include "geometry_msgs/msg/twist.hpp"


namespace drive_train_controller
{


class ArmController : public controller_interface::ControllerInterface
{
public:
  ArmController() = default;


  controller_interface::CallbackReturn on_init() override
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }


  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    // Subscribe to velocity commands from "cmd_vel"
    auto callback = [this](const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      processCommand(msg);
    };


    cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, callback);


    RCLCPP_INFO(get_node()->get_logger(), "ArmController configured and subscribed to /cmd_vel");
    return controller_interface::CallbackReturn::SUCCESS;
  }


  // Tell the controller which command interfaces it claims.
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = {
      "lf_wheel_joint/velocity",
      "lm_wheel_joint/velocity",
      "lb_wheel_joint/velocity",
      "rf_wheel_joint/velocity",
      "rm_wheel_joint/velocity",
      "rb_wheel_joint/velocity"
    };
    return config;
  }


  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::NONE;
    return config;
  }


  // Process the incoming Twist message and update command interfaces.
  void processCommand(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // // Kinematic data (metric system)
    // const float roverWidth = 0.5;    // meters
    // const float wheelRadius = 0.1143; // meters


    // // Breakdown commanded linear and angular velocity into left and right side velocities.
    // float leftSideVel  = msg->linear.x + msg->angular.y * 3.14 / 180 * roverWidth;
    // float rightSideVel = msg->linear.x - msg->angular.y * 3.14 / 180 * roverWidth;


    // float leftSideAlpha  = leftSideVel / wheelRadius;
    // float rightSideAlpha = rightSideVel / wheelRadius;


    RCLCPP_INFO(get_node()->get_logger(), "Processing /cmd_vel: leftAlpha=%f, rightAlpha=%f", leftSideAlpha, rightSideAlpha);

    // Write the computed commands into the six command interfaces:
    if (command_interfaces_.size() >= 6) {
      // First three interfaces (left side)
      command_interfaces_[0].set_value(leftSideAlpha);
      command_interfaces_[1].set_value(leftSideAlpha);
      command_interfaces_[2].set_value(leftSideAlpha);
      // Last three interfaces (right side)
      command_interfaces_[3].set_value(rightSideAlpha);
      command_interfaces_[4].set_value(rightSideAlpha);
      command_interfaces_[5].set_value(rightSideAlpha);
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Command interfaces not properly initialized. Size: %zu", command_interfaces_.size());
    }
  }


  controller_interface::return_type update(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/) override
  {
    return controller_interface::return_type::OK;
  }


private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};


}  // namespace drive_train_controller


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(drive_train_controller::ArmController, controller_interface::ControllerInterface)

