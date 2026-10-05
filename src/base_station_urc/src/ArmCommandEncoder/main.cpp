/**
 * @file
 * ArmCommandEncoder node: an alternative arm-teleop path that embeds MoveIt Servo directly (NOT currently launched)
 *
 * Status: built and installed, but not in any launch file; SpaceMouseMapper + main_computer's servo_node is the live path
 *
 * What it does:
 *   - Runs its own MoveIt Servo instance and planning scene monitor
 *   - Turns /armInputRaw into TwistStamped velocity commands every 50 ms, with a 1 s loss-of-signal stop
 *
 * Before using it, note:
 *   - It publishes servo_node/delta_twist_cmds, but main_computer's Servo config listens on /delta_twist_cmds
 *   - It needs robot_description and the Servo parameters, which only main_computer_launch.py provides
 *   - It logs at INFO on every 50 ms tick
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_servo/servo.h>
#include <control_msgs/msg/joint_jog.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <chrono>
#include <mutex>
#include <algorithm>  // For std::clamp
#include "cross_pkg_messages/msg/arm_input_raw.hpp"

class ArmCommandEncoder : public rclcpp::Node
{
public:
    /**
     * Steps:
     *   1. Start a planning scene monitor that tracks the arm's joint states
     *   2. Load the Servo parameters and start an embedded Servo instance
     *   3. Subscribe to /armInputRaw and publish a velocity command every 50 ms
     *
     * Parameters (inputs):
     *   node - a separate node that holds the MoveIt/Servo parameters
     */
    // Syntax: "explicit" stops the compiler from silently converting a node pointer into an ArmCommandEncoder
    explicit ArmCommandEncoder(const rclcpp::Node::SharedPtr& node)
        : Node("ArmCommandEncoder"), node_(node)
    {
        // The tf2 buffer stores coordinate-frame transforms that MoveIt looks up
        auto tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description", tf_buffer, "planning_scene_monitor");

        if (planning_scene_monitor_->getPlanningScene()) {
            planning_scene_monitor_->startStateMonitor("/joint_states");
            planning_scene_monitor_->startSceneMonitor();
            planning_scene_monitor_->providePlanningSceneService();
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Planning scene not configured");
            return;
        }

        servo_parameters_ = moveit_servo::ServoParameters::makeServoParameters(node_);
        if (!servo_parameters_) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load Servo parameters");
            return;
        }

        servo_ = std::make_unique<moveit_servo::Servo>(node_, servo_parameters_, planning_scene_monitor_);
        servo_->start();

        twist_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("servo_node/delta_twist_cmds", 10);

        arm_input_sub_ = this->create_subscription<cross_pkg_messages::msg::ArmInputRaw>("/armInputRaw", 10, std::bind(&ArmCommandEncoder::armInputCallback, this, std::placeholders::_1));

        // Syntax: create_wall_timer calls the bound function on a fixed period (wall clock time, not simulation time)
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ArmCommandEncoder::publishVelocityCommand, this));

        last_command_time_ = std::chrono::steady_clock::now();

        RCLCPP_INFO(this->get_logger(), "MoveIt Servo running...");
    }

private:
    /// Store the latest operator input and when it arrived
    void armInputCallback(const cross_pkg_messages::msg::ArmInputRaw::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        latest_command_ = *msg;
        last_command_time_ = std::chrono::steady_clock::now();
    }

    /**
     * Publish one velocity command from the latest input
     *
     * Steps:
     *   1. If no input arrived in the last second, send all zeros (loss-of-signal safety stop)
     *   2. Otherwise clamp each axis to [-1, 1] and scale it to a velocity
     */
    void publishVelocityCommand()
    {
        auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        twist_msg->header.stamp = this->now();
        twist_msg->header.frame_id = "tool_joint";

        auto now_time = std::chrono::steady_clock::now();

        const double kLinearScale = 0.01;  // Max linear velocity
        const double kAngularScale = 0.1;  // Max angular velocity

        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            if (std::chrono::duration_cast<std::chrono::seconds>(now_time - last_command_time_) > std::chrono::seconds(1)) {
                twist_msg->twist.linear.x = 0.0;
                twist_msg->twist.linear.y = 0.0;
                twist_msg->twist.linear.z = 0.0;
                twist_msg->twist.angular.x = 0.0;
                twist_msg->twist.angular.y = 0.0;
                twist_msg->twist.angular.z = 0.0;
                RCLCPP_WARN(this->get_logger(), "LOS: No manual command for 1 second. Safety stop applied.");
            }
            else {
                twist_msg->twist.linear.x = std::clamp(latest_command_.linear_input.x, -1.0, 1.0) * kLinearScale;
                twist_msg->twist.linear.y = std::clamp(latest_command_.linear_input.y, -1.0, 1.0) * kLinearScale;
                twist_msg->twist.linear.z = std::clamp(latest_command_.linear_input.z, -1.0, 1.0) * kLinearScale;
                twist_msg->twist.angular.x = std::clamp(latest_command_.angular_input.x, -1.0, 1.0) * kAngularScale;
                twist_msg->twist.angular.y = std::clamp(latest_command_.angular_input.y, -1.0, 1.0) * kAngularScale;
                twist_msg->twist.angular.z = std::clamp(latest_command_.angular_input.z, -1.0, 1.0) * kAngularScale;
            }
        }

        // Syntax: std::move hands ownership of the message to publish(), avoiding a copy
        twist_cmd_pub_->publish(std::move(twist_msg));
        RCLCPP_INFO(this->get_logger(), "Published velocity command.");
    }

    rclcpp::Node::SharedPtr node_;  // Node holding the MoveIt/Servo parameters (passed in from main)

    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
    moveit_servo::ServoParameters::SharedConstPtr servo_parameters_;
    std::unique_ptr<moveit_servo::Servo> servo_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<cross_pkg_messages::msg::ArmInputRaw>::SharedPtr arm_input_sub_;

    // Guards latest_command_ and last_command_time_, since callbacks may run on different threads
    std::mutex command_mutex_;
    std::chrono::steady_clock::time_point last_command_time_;
    cross_pkg_messages::msg::ArmInputRaw latest_command_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("arm_command_encoder");
    RCLCPP_INFO(node->get_logger(), "ArmCommandEncoder node has been initialized.");

    auto arm_command_encoder = std::make_shared<ArmCommandEncoder>(node);

    // A multi-threaded executor runs callbacks on several threads, so Servo and the subscriptions don't block each other
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(arm_command_encoder);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
