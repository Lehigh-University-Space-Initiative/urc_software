#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_servo/servo.h>
#include <control_msgs/msg/joint_jog.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

class ArmCommandEncoder : public rclcpp::Node
{
public:
    explicit ArmCommandEncoder(const rclcpp::Node::SharedPtr& node) 
        : Node("ArmCommandEncoder"), node_(node)
    {
        // Setup the planning scene monitor
        auto tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            node_, "robot_description", tf_buffer, "planning_scene_monitor"
        );

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

        // Timer to send velocity commands
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ArmCommandEncoder::publishVelocityCommand, this));

        RCLCPP_INFO(this->get_logger(), "MoveIt Servo running...");
    }

private:
    void publishVelocityCommand()
    {
        auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        msg->header.stamp = this->now();
        msg->header.frame_id = "panda_link0"; // Change based on your robot's base frame

        // Set linear and angular velocities
        msg->twist.linear.x = 0.1;  // Move in X direction at 0.1 m/s
        msg->twist.angular.z = 0.2; // Rotate about Z-axis at 0.2 rad/s

        twist_cmd_pub_->publish(std::move(msg));
        RCLCPP_INFO(this->get_logger(), "Published velocity command.");
    }

    // Store shared node pointer
    rclcpp::Node::SharedPtr node_;

    // Servo components
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
    moveit_servo::ServoParameters::SharedConstPtr servo_parameters_;
    std::unique_ptr<moveit_servo::Servo> servo_;

    // ROS publisher and timer
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // Create the shared node first
    auto node = std::make_shared<rclcpp::Node>("arm_command_encoder");
    
    // Pass the shared node into the ArmCommandEncoder class
    auto arm_command_encoder = std::make_shared<ArmCommandEncoder>(node);

    // Use MultiThreadedExecutor for Servo
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(arm_command_encoder);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
