/**
 * @file
 * JoyMapper node: turns two joysticks into a whole-rover drive command (tank-style driving)
 *
 * Runs on: the base station laptop
 * Started by: base_station_launch.py (run modes "base_station" and "hootl")
 *
 * Subscribes:
 *   joy0, joy1 (sensor_msgs/Joy) - one joystick each, published by the two joy_node drivers in the launch file
 *
 * Publishes:
 *   cmd_vel (geometry_msgs/Twist) - linear.x forward speed (m/s), angular.y turn rate (deg/s)
 *
 * Parameters:
 *   swap_joysticks (bool, default true) - swap which joystick drives which side (the GUI's Telemetry panel toggles this)
 *
 * How it connects to the system:
 *   - Each joystick's forward/back axis drives one side of the rover, like a tank
 *   - DriveTrainManager on the main computer turns cmd_vel into per-wheel speeds
 *   - Turn rate goes in angular.y because this code treats +Y as "up" (the ROS standard uses angular.z; see the README)
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

/**
 * Remembers the latest message from each joystick and publishes a combined drive command
 *
 * Syntax: deriving from rclcpp::Node makes the class itself the ROS node, so it calls create_subscription etc. directly
 */
class JoyMapper : public rclcpp::Node
{
public:
    // Syntax: ": Node("joy_mapper")" passes the node's name to the rclcpp::Node base-class constructor
    JoyMapper() : Node("joy_mapper")
    {
        // Syntax: std::bind(&JoyMapper::joy0Callback, this, std::placeholders::_1) makes a callable that runs this->joy0Callback(msg)
        // Syntax: std::placeholders::_1 stands for the message argument the subscription passes in
        joy0_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy0", 10, std::bind(&JoyMapper::joy0Callback, this, std::placeholders::_1));

        joy1_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy1", 10, std::bind(&JoyMapper::joy1Callback, this, std::placeholders::_1));

        // System: DriveTrainManager on the main computer subscribes to this
        drive_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        this->declare_parameter("swap_joysticks", true);
    }

private:
    void joy0Callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        last_joy0_msg_ = *msg;  // Syntax: *msg copies the message the shared pointer points to
        sendCommand();
    }

    void joy1Callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        last_joy1_msg_ = *msg;
        sendCommand();
    }

    /**
     * Combine the two joysticks into one drive command and publish it
     *
     * Steps:
     *   1. Wait until both joysticks have reported at least once
     *   2. Read each stick's forward/back axis (axes[1]) and swap sides if swap_joysticks is set
     *   3. Average the sticks for forward speed, and take half their difference for turn rate
     */
    void sendCommand()
    {
        if (last_joy0_msg_.axes.empty() || last_joy1_msg_.axes.empty()) {
            return;
        }

        // Rover-centric coordinate system: +X is rover front, +Y is rover top, right handed
        geometry_msgs::msg::Twist cmd;

        // Syntax: "auto" lets the compiler work out the variable's type from the value (here, float)
        auto joy_left = last_joy0_msg_.axes[1];
        auto joy_right = last_joy1_msg_.axes[1];

        auto flip_joystics = this->get_parameter("swap_joysticks");
        if (flip_joystics.as_bool()) {
            std::swap(joy_left, joy_right);
        }

        // Both sticks forward drives straight; one forward and one back spins in place
        cmd.linear.x = (joy_left + joy_right) / 2 * linearSensativity;
        cmd.angular.y = (joy_right - joy_left) / 2 * angularSensativity;

        drive_pub_->publish(cmd);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy0_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy1_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drive_pub_;

    sensor_msgs::msg::Joy last_joy0_msg_;  // Latest message from joystick 0 (empty until the first one arrives)
    sensor_msgs::msg::Joy last_joy1_msg_;  // Latest message from joystick 1

    // Speed at 100% throttle, in m/s (the negative sign inverts the joystick's forward axis)
    const double linearSensativity = -1;
    // Turn rate at 100% twist, in deg/s
    const double angularSensativity = 120;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto joy_mapper = std::make_shared<JoyMapper>();
    rclcpp::spin(joy_mapper);  // Runs callbacks until Ctrl+C
    rclcpp::shutdown();

    return 0;
}
