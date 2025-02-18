#include <pigpio.h>
#include <vector>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"


cross_pkg_messages::msg::RoverComputerDriveCMD currentDriveCommand{};
rclcpp::Publisher<cross_pkg_messages::msg::RoverComputerDriveCMD>::SharedPtr driveTrainPublisher;

std::shared_ptr<rclcpp::Node> node;

std::vector<int> gpiopins;

std::vector<uint32_t> last_edges;
std::vector<size_t> edge_width;
void gpio_callback(int gpio, int level, uint32_t tick);
void free_gpio();

void setup_pins() {
    //GPIO pins for each PWM reader
    gpiopins.push_back(14); //22
    // gpiopins.push_back(1);
    // gpiopins.push_back(2);
    // gpiopins.push_back(3);
    // gpiopins.push_back(4);

    gpioSetPullUpDown(22, PI_PUD_DOWN); //temp ground, don't forget to remove when done

    for(int pin : gpiopins) {
        //set up gpio
        gpioSetMode(pin, PI_INPUT);
        gpioSetPullUpDown(pin, PI_PUD_UP);

        //register callback for the pin
        gpioSetAlertFunc(pin, gpio_callback);

        //for keeping track of pulse lengths
        last_edges.push_back(0);
        
    }
    edge_width.reserve(gpiopins.size());
}

/**
    Read PWM values from the GPIO encoders and figure out where each motor is in radians
*/
void read_pwm() {

    // for (auto width : edge_width) {
        auto width = edge_width[0];
        width %= 1024;
        //double radians = (width - 1) * (2 * 3.14159265) / 1023;
        double degrees = (width - 1) * (360 / 1023);
        // RCLCPP_INFO(node->get_logger(), "PWM Pulse length: %zu,\tMotor degrees: %f", width, degrees);
        // break;
    // }
}

int main(int argc, char** argv) 
{
    // initialize pi gpio library and rclcpp
    if (gpioInitialise() < 0) return 1;
    rclcpp::init(argc, argv);

    // Create ROS2 node
    // TODO: instead of making a new node, use another thread on the existing arm node
    node = rclcpp::Node::make_shared("ArmPWM");
    RCLCPP_INFO(node->get_logger(), "ArmPWM is running");

    //TODO: create publisher

    // setup which pin does what
    setup_pins();

    rclcpp::Rate loop_rate(10);  // Set rate to 10 Hz

    // start spinning node
    while(rclcpp::ok()) {
        rclcpp::spin_some(node);
        read_pwm();

        loop_rate.sleep();
    }

    // free gpio pins and shut down the node
    free_gpio();
    rclcpp::shutdown();
    return 0;
}

/**
    Callback function for pins. Called when an initialized pin changes its value
    @param gpio the gpio pin number that changed
    @param level the value that the pin changed to. 0 if low, 1 if high, 2 if no change (meaning there was a watchdog timeout)
    @param tick The number of microseconds since boot
 */
void gpio_callback(int gpio, int level, uint32_t tick) {
    static int count = 0;
    for(int i=0; i<gpiopins.size(); i++) {
        if(gpiopins[i] == gpio) {
            
            if (count < 50) {
                // calculate edge width
                // TODO: try instead using tick parameter
                uint32_t curtime = tick; // get current clock time
                uint32_t elapsed = curtime - last_edges[i];
                
                // log the elapsed time of this pulse
                // RCLCPP_INFO(node->get_logger(), "PWM time GPIO %d: %zu (state=%s) (tick=%u)", gpio, elapsed, level == 0 ? "OFF" : "ON", tick);
                RCLCPP_INFO(node->get_logger(), "PWM pulse: %zu (state=%s)", edge_width[i], level == 0 ? "OFF" : "ON");


                if (level == 0) edge_width[i] = elapsed; // update the current edge width of the pulse
                last_edges[i] = curtime; // update the last time
            }
            // std::this_thread::sleep_for(std::chrono::milliseconds(100));
            // count++;
            break;
        }
    }
}

void free_gpio() {
    for(int pin : gpiopins)
        gpioSetAlertFunc(pin, 0);
}