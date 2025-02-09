#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


class SpaceMouseMapper : public rclcpp::Node
{
public:
  SpaceMouseMapper() : Node("SpaceMouseMapper")
  {
    drive_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

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
          break;
        }
      }
      else
      {
        //      printf("could not open device %d\n",i);
      }
    }
  }

  void processEventInput(int axes[6]) {

  }

  void readEvents()
  {

    if (joyFd == 0)
      return;

    int axes[6] = {0, 0, 0, 0, 0, 0};

    struct input_event ev;


    int n = read(joyFd, &ev, sizeof(struct input_event));
    if (n >= sizeof(struct input_event))
    {
      switch (ev.type)
      {
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

  void sendCommand()
  {
    // if (last_joy0_msg_.axes.empty() || last_joy1_msg_.axes.empty())
    //   return;

    // // Rover Centric Coordinate System: +X is rover front, +Y is rover top, Right Handed
    // geometry_msgs::msg::Twist cmd;

    // auto joy_left = last_joy0_msg_.axes[1];
    // auto joy_right = last_joy1_msg_.axes[1];

    // auto flip_joystics = this->get_parameter("swap_joysticks");
    // if (flip_joystics.as_bool()) {
    //   std::swap(joy_left,joy_right);
    // }
    
    // cmd.linear.x = (joy_left + joy_right) / 2 * linearSensativity;
    // cmd.angular.y = (joy_right - joy_left) / 2 * angularSensativity;

    // drive_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drive_pub_;

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

    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}