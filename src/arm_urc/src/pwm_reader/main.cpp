#include <pigpio.h>
#include <vector>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"

cross_pkg_messages::msg::RoverComputerDriveCMD currentDriveCommand{};
rclcpp::Publisher<cross_pkg_messages::msg::RoverComputerDriveCMD>::SharedPtr driveTrainPublisher;

std::shared_ptr<rclcpp::Node> node;

std::vector<int> gpiopins;

std::vector<std::chrono::steady_clock::time_point> last_edges;
void gpio_callback(int gpio, int level, uint32_t tick);

void setup_pins() {
    //GPIO pins for each PWM reader
    gpiopins.push_back(10); //22
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
        last_edges.push_back(std::chrono::steady_clock::now());
    }
}

void release_gpio() {
    for(int pin : gpiopins) {
        gpioSetAlertFuncEx(pin, 0);
    }
}

/**
    Read PWM values from the GPIO encoders and figure out where each motor
*/
void read_pwm() {

}

int main(int argc, char** argv) 
{
    if (gpioInitialise() < 0) return 1;

    rclcpp::init(argc, argv);

    // Create ROS2 node
    node = rclcpp::Node::make_shared("ArmPWM");
    RCLCPP_INFO(node->get_logger(), "DriveTrainManager is running");

    //TODO: create publisher

    setup_pins();

    rclcpp::Rate loop_rate(10);  // Set rate to 10 Hz

    while(rclcpp::ok()) {
        rclcpp::spin_some(node);
        read_pwm();

        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

void gpio_callback(int gpio, int level, uint32_t tick) {
    for(int i=0; i<gpiopins.size(); i++) {
        if(gpiopins[i] == gpio) {
            //calculate edge length
            std::chrono::steady_clock::time_point curtime;
            size_t elapsed = std::chrono::duration_cast<std::chrono::microseconds>(curtime - last_edges[i]).count()
            
            RCLCPP_INFO(node->get_logger(), "PWM time GPIO %d: %zu", gpio, elapsed);

            last_edges[i] = curtime; //update the last time
            break;
        }
    }
}