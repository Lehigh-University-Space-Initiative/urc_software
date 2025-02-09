#include <controller_interface/controller_interface.hpp>
#include "geometry_msgs/msg/twist.hpp"


namespace drive_train_controller
{


class DriveTrainController : public controller_interface::ControllerInterface
{
public:
  DriveTrainController() = default;


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


    return controller_interface::CallbackReturn::SUCCESS;
  }


  // Tell the controller which command interfaces it claims.
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = {
      "lf_wheel_joint",
      "lm_wheel_joint",
      "lb_wheel_joint",
      "rf_wheel_joint",
      "rm_wheel_joint",
      "rb_wheel_joint"
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
    // kinematic data (metric system)
    const float roverWidth = 0.5;    // meters
    const float wheelRadius = 0.1143; // meters


    // Breakdown commanded linear and angular velocity into left and right side velocities.
    float leftSideVel  = msg->linear.x + msg->angular.y * 3.14 / 180 * roverWidth;
    float rightSideVel = msg->linear.x - msg->angular.y * 3.14 / 180 * roverWidth;


    float leftSideAlpha  = leftSideVel / wheelRadius;
    float rightSideAlpha = rightSideVel / wheelRadius;


    // Write the computed commands into the six command interfaces:
    if (command_interfaces_.size() >= 6) {
      // First three joints are on the left side.
      command_interfaces_[0].set_value(leftSideAlpha);
      command_interfaces_[1].set_value(leftSideAlpha);
      command_interfaces_[2].set_value(leftSideAlpha);
      // Last three joints are on the right side.
      command_interfaces_[3].set_value(rightSideAlpha);
      command_interfaces_[4].set_value(rightSideAlpha);
      command_interfaces_[5].set_value(rightSideAlpha);
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
PLUGINLIB_EXPORT_CLASS(drive_train_controller::DriveTrainController, controller_interface::ControllerInterface)

