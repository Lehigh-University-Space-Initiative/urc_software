#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_servo/servo.h>
#include <control_msgs/msg/joint_jog.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <chrono>
#include <mutex>
#include <algorithm>  // for std::clamp
#include "cross_pkg_messages/msg/arm_input_raw.hpp"


class ArmCommandEncoder : public rclcpp::Node
{
public:
  explicit ArmCommandEncoder(const rclcpp::Node::SharedPtr & node)
    : Node("ArmCommandEncoder"), node_(node)
  {
    // Setup the planning scene monitor
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_, "robot_description", tf_buffer, "planning_scene_monitor");


    if (planning_scene_monitor_->getPlanningScene())
    {
      planning_scene_monitor_->startStateMonitor("/joint_states");
      planning_scene_monitor_->startSceneMonitor();
      planning_scene_monitor_->providePlanningSceneService();
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Planning scene not configured");
      return;
    }


    // Load MoveIt Servo parameters correctly
    servo_parameters_ = moveit_servo::ServoParameters::makeServoParameters(node_);
    if (!servo_parameters_)
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to load Servo parameters");
      return;
    }


    // Initialize MoveIt Servo
    servo_ = std::make_unique<moveit_servo::Servo>(node_, servo_parameters_, planning_scene_monitor_);
    servo_->start();


    // Publisher for velocity commands (TwistStamped)
    twist_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "servo_node/delta_twist_cmds", 10);


    // Subscription to armInputRaw messages
    arm_input_sub_ = this->create_subscription<cross_pkg_messages::msg::ArmInputRaw>(
      "/armInputRaw", 10,
      std::bind(&ArmCommandEncoder::armInputCallback, this, std::placeholders::_1));


    // Timer to send velocity commands at 50ms intervals
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&ArmCommandEncoder::publishVelocityCommand, this));


    // Initialize last command time to now
    last_command_time_ = std::chrono::steady_clock::now();


    RCLCPP_INFO(this->get_logger(), "MoveIt Servo running...");
  }


private:
  // Callback to receive arm input commands
  void armInputCallback(const cross_pkg_messages::msg::ArmInputRaw::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    latest_command_ = *msg;
    last_command_time_ = std::chrono::steady_clock::now();
  }


  // Timer callback to send velocity commands
  void publishVelocityCommand()
  {
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    twist_msg->header.stamp = this->now();
    twist_msg->header.frame_id = "arm";


    auto now_time = std::chrono::steady_clock::now();


    // Scaling factors
    const double kLinearScale = 0.01;
    const double kAngularScale = 0.1;


    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      // Check for loss-of-signal: if no command for over 1 second, zero the velocities.
      if (std::chrono::duration_cast<std::chrono::seconds>(now_time - last_command_time_) > std::chrono::seconds(1))
      {
        twist_msg->twist.linear.x  = 0.0;
        twist_msg->twist.linear.y  = 0.0;
        twist_msg->twist.linear.z  = 0.0;
        twist_msg->twist.angular.x = 0.0;
        twist_msg->twist.angular.y = 0.0;
        twist_msg->twist.angular.z = 0.0;
        RCLCPP_WARN(this->get_logger(), "LOS: No manual command for 1 second. Safety stop applied.");
      }
      else
      {
        // Clamp each input to [-1,1] then apply scaling factors
        twist_msg->twist.linear.x  = std::clamp(latest_command_.linear_input.x, -1.0, 1.0) * kLinearScale;
        twist_msg->twist.linear.y  = std::clamp(latest_command_.linear_input.y, -1.0, 1.0) * kLinearScale;
        twist_msg->twist.linear.z  = std::clamp(latest_command_.linear_input.z, -1.0, 1.0) * kLinearScale;
        twist_msg->twist.angular.x = std::clamp(latest_command_.angular_input.x, -1.0, 1.0) * kAngularScale;
        twist_msg->twist.angular.y = std::clamp(latest_command_.angular_input.y, -1.0, 1.0) * kAngularScale;
        twist_msg->twist.angular.z = std::clamp(latest_command_.angular_input.z, -1.0, 1.0) * kAngularScale;
      }
    }


    twist_cmd_pub_->publish(std::move(twist_msg));
    RCLCPP_INFO(this->get_logger(), "Published velocity command.");
  }


  // Shared node pointer passed from main
  rclcpp::Node::SharedPtr node_;


  // Servo components
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
  moveit_servo::ServoParameters::SharedConstPtr servo_parameters_;
  std::unique_ptr<moveit_servo::Servo> servo_;


  // ROS publisher, timer, and subscription
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<cross_pkg_messages::msg::ArmInputRaw>::SharedPtr arm_input_sub_;


  // Variables for managing manual command timeout
  std::mutex command_mutex_;
  std::chrono::steady_clock::time_point last_command_time_;
  cross_pkg_messages::msg::ArmInputRaw latest_command_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  // Create the shared node first
  auto node = std::make_shared<rclcpp::Node>("arm_command_encoder");
  RCLCPP_INFO(node->get_logger(), "ArmCommandEncoder node has been initialized.");
  
  // Pass the shared node into the ArmCommandEncoder class
  auto arm_command_encoder = std::make_shared<ArmCommandEncoder>(node);


  // Use MultiThreadedExecutor for Servo and subscriptions
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(arm_command_encoder);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
