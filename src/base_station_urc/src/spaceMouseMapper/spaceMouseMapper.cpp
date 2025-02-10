#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "cross_pkg_messages/msg/arm_input_raw.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


class SpaceMouseMapper : public rclcpp::Node
{
public:
  SpaceMouseMapper() : Node("SpaceMouseMapper")
  {
    drive_pub_ = this->create_publisher<cross_pkg_messages::msg::ArmInputRaw>("/armInputRaw", 10);
    servo_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("/delta_twist_cmds", 10);

    loadJoystick();
  }

  ~SpaceMouseMapper() {
    if (joyFd) {
      close(joyFd);
    }
  }

  void tick() {
    readEvents();
  }

private:


  int joyFd = 0;


  bool left_btn = 0;
  bool right_btn = 0;

  void loadJoystick() {
    int i = 0;
    char *fname = NULL;
    struct input_id ID;


    // find the SpaceNavigator or similar device
    fname = (char *)malloc(1000 * sizeof(char));
    while (i < 32)
    {
      sprintf(fname, "/dev/input/event%d", i++);
      joyFd = open(fname, O_RDWR | O_NONBLOCK);
      if (joyFd > 0)
      {
        printf("Openned device %d\n", i);
        ioctl(joyFd, EVIOCGID, &ID);

        printf("device %d has %X vendor and %X product\n", i, ID.vendor, ID.product);

        if (ID.vendor == 0x256f)
        {
          printf("Using device: %s\n", fname);
          RCLCPP_INFO(get_logger(), "Found a Space Mouse: %s",fname);
          break;
        }
      }
      else
      {
        //      printf("could not open device %d\n",i);
      }
    }
  }

  float fixAxis(int axis) {
    const float axisBounds = 350;
    const float deadband = 50.0 / 350;

    float val = static_cast<float>(axis) / axisBounds;
    bool neg = val < 0;
    if (abs(val) < deadband) {
      val = 0;
    } else {
      val = (abs(val) - deadband) / (1 - deadband);
    }
    val = val * val;
    if (neg) {
      val = -val;
    }
    return val;
  }

  float lerp(float a, float b, float t) {
    return a + (b - a) * t;
  }

  void processEventInput(const std::vector<int>& axes) {

    cross_pkg_messages::msg::ArmInputRaw msg;

    //numbers are in wrong orders to map axes to our linear xyz, angular xyz system
    msg.linear_input.x = fixAxis(-axes[1]);
    msg.linear_input.y = fixAxis(-axes[0]);
    msg.linear_input.z = fixAxis(-axes[2]);

    msg.angular_input.x = fixAxis(-axes[4]);
    msg.angular_input.y = fixAxis(-axes[3]);
    msg.angular_input.z = fixAxis(-axes[5]);
    msg.left_btn = left_btn * 0.1;
    msg.right_btn = right_btn * 0.1;

    drive_pub_->publish(msg);

    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = "tool_link";

    // Scaling factors
    const double kLinearScale = 0.5;
    const double kAngularScale = 1;

    twist_msg.twist.linear.x  = std::clamp(msg.linear_input.z*-1, -1.0, 1.0) * kLinearScale;
    twist_msg.twist.linear.y  = std::clamp(msg.linear_input.y*1, -1.0, 1.0) * kLinearScale;
    twist_msg.twist.linear.z  = std::clamp(msg.linear_input.x*1, -1.0, 1.0) * kLinearScale;
    twist_msg.twist.angular.x = std::clamp(msg.angular_input.z*-1, -1.0, 1.0) * kAngularScale;
    twist_msg.twist.angular.y = std::clamp(msg.angular_input.y*1, -1.0, 1.0) * kAngularScale;
    twist_msg.twist.angular.z = std::clamp(msg.angular_input.x*1, -1.0, 1.0) * kAngularScale;

    servo_pub->publish(twist_msg);

    // RCLCPP_INFO(get_logger(), "Sending with L1: %d, %.2f",axes[0],msg.linear_input.x);
  }

  std::vector<int> axes = {0, 0, 0, 0, 0, 0};

  void readEvents()
  {

    if (joyFd == 0)
      return;


    struct input_event ev;


    int n = read(joyFd, &ev, sizeof(struct input_event));
    if (n >= sizeof(struct input_event))
    {
      switch (ev.type)
      {
      case EV_KEY:
        // printf("Key %d pressed %d.\n", ev.code, ev.value);
        if (ev.code == 256) {
          left_btn = ev.value;
        } else if (ev.code == 257) {
          right_btn = ev.value;
        }
        break;

      /*
          older kernels than and including 2.6.31 send EV_REL events for SpaceNavigator movement
          newer - 2.6.35 and upwards send the more logical EV_ABS instead.

          The meaning of the numbers is the same. Spotted by Thomax, thomax23@googlemail.com
      */
      case EV_REL:
        // printf("REL %d %d\n", ev.code, ev.value);
        axes[ev.code] = ev.value;
        break;


      default:
        break;
      }
    }
    processEventInput(axes);
    // printf("%d %d %d %d %d %d\n", axes[0], axes[1], axes[2], axes[3], axes[4], axes[5]);
  }

  rclcpp::Publisher<cross_pkg_messages::msg::ArmInputRaw>::SharedPtr drive_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_pub;

  // 100% forward thottle should be this speed m/s
  const double linearSensativity = 0.5;
  // 100% twist throttle should be this speed in deg/s
  const double angularSensativity = 120;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto joy_mapper = std::make_shared<SpaceMouseMapper>();

  rclcpp::Rate loop_rate(60);  // Set rate to 10 Hz

  while(rclcpp::ok()) {
    joy_mapper->tick();
    rclcpp::spin_some(joy_mapper);

  }
  rclcpp::shutdown();
  return 0;
}