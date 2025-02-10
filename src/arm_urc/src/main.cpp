#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cross_pkg_messages/msg/rover_computer_arm_cmd.hpp" // TODO: figure these out
#include "ArmMotorManager.h"
#include "main.h"
#include <chrono>

// Global variables
std::shared_ptr<rclcpp::Node> node;

std::unique_ptr<ArmMotorManager> manager;

// Callback function
void callback(const cross_pkg_messages::msg::RoverComputerArmCMD::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("Motor_CTR"), "Received command with CMD_S: %f", msg->cmd_s);
    // wrist_yaw.setVelocity(msg->cmd_r.z);  // Uncomment and set velocity when integrating
    manager->setArmCommand(msg);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create ROS2 node
    node = rclcpp::Node::make_shared("ArmMotorManager");

    node->declare_parameter("kp",1.0);
    node->declare_parameter("kd",0.01);
    node->declare_parameter("ki",1.0);
    node->declare_parameter("max_i",0.01);
    node->declare_parameter("readOnly",false);

    RCLCPP_INFO(node->get_logger(), "ArmMotorManager is running");

    // Construct the manager
    manager = std::make_unique<ArmMotorManager>(node,true);
    manager->init();

        // Subscriber for rover drive commands
    auto driveCommandsSub = node->create_subscription<cross_pkg_messages::msg::RoverComputerArmCMD>(
        "/roverArmCommands", 10, callback);

    auto armPosPub = node->create_publisher<cross_pkg_messages::msg::RoverComputerArmCMD>(
        "/roverArmPos", 10);

    rclcpp::Rate loop_rate(100); // Set rate to 6k Hz
    
    bool firstTime = true;

    size_t itr = 0;

    /*
        to add
        - logging to see period of all updating events and incoming topics 
        - see if moving velocity and positining reading to another thread will help
        - need to understand better what effect queue length has
        - use rqt plot to try to see if delay is occuring pre during or post servo node


    DATA
        full freq:  around 2000-7000
        pid tick freq: about 600
        publish freq: 131


        MORE DATA

        - looks like esp with new fixes but even before this node is not a problem and the data received from servo is oscilating

    */

    while (rclcpp::ok())
    {
        static std::chrono::system_clock::time_point last_update;
        double delta = std::chrono::duration<double>(std::chrono::system_clock::now() - last_update).count();
        last_update = std::chrono::system_clock::now();
        RCLCPP_INFO(rclcpp::get_logger("Arm"), "full cycle delta: %f", delta);


        // Spin and process ROS callbacks
        rclcpp::spin_some(node);
        manager->tick();

        if (!firstTime && itr++ % 2 == 0) {
            static std::chrono::system_clock::time_point last_update;
            double delta = std::chrono::duration<double>(std::chrono::system_clock::now() - last_update).count();
            last_update = std::chrono::system_clock::now();
            RCLCPP_INFO(rclcpp::get_logger("Arm"), "publish cycle delta: %f", delta);

            //reading of movotrs for movit should be in radians from veterical
            manager->readMotors(delta);

            auto& positions = manager->getMotorPositions();
            if (positions.size() >= 6) {
                cross_pkg_messages::msg::RoverComputerArmCMD msg{};


                msg.cmd_b = positions[0];
                msg.cmd_s = positions[1];
                msg.cmd_e = positions[2];
                msg.cmd_w.x = positions[3];
                msg.cmd_w.y = positions[4];
                msg.cmd_w.z = positions[5];

                armPosPub->publish(msg);
            }

        }
        firstTime = false;

        {
        double delta = std::chrono::duration<double>(std::chrono::system_clock::now() - last_update).count();
        RCLCPP_INFO(rclcpp::get_logger("Arm"), "full cycle actual duration: %f", delta);
        }
        loop_rate.sleep();
    }

    //dealloc
    manager = nullptr;

    rclcpp::shutdown();
    return 0;
}
