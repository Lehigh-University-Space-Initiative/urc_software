/**
 * @file
 * SpaceMouseMapper node: reads a 3Dconnexion SpaceMouse and publishes arm motion commands
 *
 * Runs on: the base station laptop
 * Started by: base_station_launch.py (run modes "base_station" and "hootl")
 *
 * Publishes:
 *   /armInputRaw (cross_pkg_messages/ArmInputRaw) - raw 6-axis input plus the two buttons
 *   /delta_twist_cmds (geometry_msgs/TwistStamped) - end-effector velocity for MoveIt Servo, in frame tool_link
 *
 * How it connects to the system:
 *   - MoveIt Servo on the main computer subscribes to /delta_twist_cmds and turns it into joint motion
 *   - Servo stops if no command arrives for 1 s (incoming_command_timeout), so this node keeps publishing every loop
 *   - arm_urc on the arm Pi reads the buttons from /armInputRaw to open and close the gripper
 *   - The GUI's Telemetry panel also shows /armInputRaw
 *
 * Without a SpaceMouse (WSL, hootl): logs one warning, publishes nothing, and idles instead of spinning the CPU
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "cross_pkg_messages/msg/arm_input_raw.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

/**
 * Reads SpaceMouse events from Linux's input subsystem and republishes them as ROS messages
 */
class SpaceMouseMapper : public rclcpp::Node
{
public:
    SpaceMouseMapper() : Node("SpaceMouseMapper")
    {
        drive_pub_ = this->create_publisher<cross_pkg_messages::msg::ArmInputRaw>("/armInputRaw", 10);
        servo_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("/delta_twist_cmds", 10);

        loadJoystick();
    }

    // Syntax: "~SpaceMouseMapper()" is the destructor, which runs when the object is destroyed
    ~SpaceMouseMapper()
    {
        if (joyFd >= 0) {
            close(joyFd);
        }
    }

    /**
     * Read one input event (if any) and publish the current input state
     *
     * Return value:
     *   true if a SpaceMouse is connected, false if not (the caller then sleeps instead of busy-looping)
     */
    bool tick()
    {
        return readEvents();
    }

private:
    int joyFd = -1;  // File descriptor of the opened SpaceMouse device, or -1 if none was found

    bool left_btn = 0;  // Left SpaceMouse button currently held
    bool right_btn = 0;  // Right SpaceMouse button currently held

    /**
     * Find and open the SpaceMouse among the Linux input devices
     *
     * Steps:
     *   1. Try /dev/input/event0 through event31 (each input device gets one of these files)
     *   2. Ask each device for its USB vendor ID, and keep the first one made by 3Dconnexion (0x256f)
     *   3. Close every other device; if none matches, leave joyFd at -1 and warn once
     */
    void loadJoystick()
    {
        char fname[64];
        struct input_id ID;

        for (int i = 0; i < 32; i++) {
            snprintf(fname, sizeof(fname), "/dev/input/event%d", i);
            // Opening non-blocking, so read() returns immediately when no event is waiting
            int fd = open(fname, O_RDWR | O_NONBLOCK);
            if (fd < 0) {
                continue;
            }

            // EVIOCGID asks the kernel for the device's bus, vendor, product, and version IDs
            ioctl(fd, EVIOCGID, &ID);
            printf("device %d has %X vendor and %X product\n", i, ID.vendor, ID.product);

            if (ID.vendor == 0x256f) {
                RCLCPP_INFO(get_logger(), "Found a Space Mouse: %s", fname);
                joyFd = fd;
                return;
            }

            close(fd);
        }

        RCLCPP_WARN(get_logger(), "No SpaceMouse found under /dev/input; arm input is disabled (expected without the device, e.g. WSL or hootl)");
    }

    /**
     * Turn a raw axis reading into a smoothed value in [-1, 1]
     *
     * Parameters (inputs):
     *   axis - raw reading from the device (roughly -350 to 350)
     *
     * Return value:
     *   0 inside the deadband, otherwise the rescaled value squared (keeping its sign)
     *   Squaring gives fine control near the center and full speed at the edges
     */
    float fixAxis(int axis)
    {
        const float axisBounds = 350;  // Approximate full-deflection reading
        const float deadband = 50.0 / 350;  // Ignore the first ~14% of travel so a resting hand doesn't move the arm

        float val = static_cast<float>(axis) / axisBounds;
        bool neg = val < 0;
        if (abs(val) < deadband) {
            val = 0;
        }
        else {
            // Rescaling so the output starts at 0 right at the deadband edge instead of jumping
            val = (abs(val) - deadband) / (1 - deadband);
        }
        val = val * val;
        if (neg) {
            val = -val;
        }

        return val;
    }

    /// Linear interpolation between a and b (unused)
    float lerp(float a, float b, float t)
    {
        return a + (b - a) * t;
    }

    /**
     * Publish the current axis and button state on both output topics
     *
     * Parameters (inputs):
     *   axes - latest raw reading of each of the six SpaceMouse axes
     */
    void processEventInput(const std::vector<int>& axes)
    {
        cross_pkg_messages::msg::ArmInputRaw msg;

        // The device's axis order and signs differ from our linear xyz / angular xyz layout, hence the reordering
        msg.linear_input.x = fixAxis(-axes[1]);
        msg.linear_input.y = fixAxis(-axes[0]);
        msg.linear_input.z = fixAxis(-axes[2]);

        msg.angular_input.x = fixAxis(-axes[4]);
        msg.angular_input.y = fixAxis(-axes[3]);
        msg.angular_input.z = fixAxis(-axes[5]);
        msg.left_btn = left_btn * 0.01;  // Sent as 0.01 while held (arm_urc only checks for nonzero)
        msg.right_btn = right_btn * 0.01;

        drive_pub_->publish(msg);

        auto twist_msg = geometry_msgs::msg::TwistStamped();
        twist_msg.header.stamp = this->now();
        twist_msg.header.frame_id = "tool_link";  // Velocities are relative to the end effector (Servo's command frame)

        const double kLinearScale = 0.5;  // Max end-effector speed in m/s
        const double kAngularScale = 1;  // Max end-effector rotation rate in rad/s

        // Remapping to the tool frame's axes and clamping to [-1, 1] before scaling
        twist_msg.twist.linear.x = std::clamp(msg.linear_input.z * -1, -1.0, 1.0) * kLinearScale;
        twist_msg.twist.linear.y = std::clamp(msg.linear_input.y * 1, -1.0, 1.0) * kLinearScale;
        twist_msg.twist.linear.z = std::clamp(msg.linear_input.x * 1, -1.0, 1.0) * kLinearScale;
        twist_msg.twist.angular.x = std::clamp(msg.angular_input.z * -1, -1.0, 1.0) * kAngularScale;
        twist_msg.twist.angular.y = std::clamp(msg.angular_input.y * 1, -1.0, 1.0) * kAngularScale;
        twist_msg.twist.angular.z = std::clamp(msg.angular_input.x * 1, -1.0, 1.0) * kAngularScale;

        servo_pub->publish(twist_msg);
    }

    std::vector<int> axes = {0, 0, 0, 0, 0, 0};  // Latest raw value of each axis (x, y, z, rx, ry, rz)

    /**
     * Read at most one waiting input event, then publish the current state
     *
     * Return value:
     *   true if a SpaceMouse is open, false if there is none
     *
     * Steps:
     *   1. Return right away if no SpaceMouse was found
     *   2. Read one event without blocking; a button event updates the button state, a motion event updates one axis
     *   3. Publish the current state every call, even with no new event (MoveIt Servo needs a steady command stream)
     */
    bool readEvents()
    {
        if (joyFd < 0) {
            return false;
        }

        struct input_event ev;

        // Syntax: read() returns the number of bytes read, or -1 when nothing is waiting (non-blocking mode)
        // ssize_t keeps that -1 signed; comparing a raw -1 against sizeof() would wrap it to a huge positive number
        ssize_t n = read(joyFd, &ev, sizeof(struct input_event));
        if (n == static_cast<ssize_t>(sizeof(struct input_event))) {
            switch (ev.type) {
                case EV_KEY:
                    // Button codes 256 and 257 are BTN_0 and BTN_1, the SpaceMouse's two side buttons
                    if (ev.code == 256) {
                        left_btn = ev.value;
                    }
                    else if (ev.code == 257) {
                        right_btn = ev.value;
                    }
                    break;

                /*
                    Kernels up to 2.6.31 send EV_REL events for SpaceNavigator movement
                    Kernels from 2.6.35 on send EV_ABS instead
                    The meaning of the numbers is the same (spotted by Thomax, thomax23@googlemail.com)
                */
                case EV_REL:
                    // Ignoring axis codes beyond the six we track, which would otherwise write past the end of axes
                    if (ev.code < axes.size()) {
                        axes[ev.code] = ev.value;
                    }
                    break;

                default:
                    break;
            }
        }
        processEventInput(axes);

        return true;
    }

    rclcpp::Publisher<cross_pkg_messages::msg::ArmInputRaw>::SharedPtr drive_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_pub;

    // 100% forward throttle should be this speed m/s (unused)
    const double linearSensativity = 0.5;
    // 100% twist throttle should be this speed in deg/s (unused)
    const double angularSensativity = 120;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto joy_mapper = std::make_shared<SpaceMouseMapper>();

    rclcpp::Rate loop_rate(60);  // Only used to idle at 60 Hz when no SpaceMouse is connected

    while (rclcpp::ok()) {
        // With a SpaceMouse this loop runs as fast as possible, draining one input event per pass
        bool hasDevice = joy_mapper->tick();
        rclcpp::spin_some(joy_mapper);

        if (!hasDevice) {
            loop_rate.sleep();
        }
    }
    rclcpp::shutdown();

    return 0;
}
