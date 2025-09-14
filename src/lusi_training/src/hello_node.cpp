#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class HelloNode : public rclcpp::Node {
    public:
        HelloNode() : Node("hello_node") {

            message_= this->declare_parameter<std::string>("message", "Hello from the LUSI rover software team!");
            double rate_hz = this->declare_parameter<double>("rate_hz", 1.0);

            if (rate_hz <= 0.0) {
                RCLCPP_WARN(this->get_logger(), "rate_hz parameter must be positive. Setting to 1.0 Hz.");
                rate_hz = 1.0;
            }
            auto period = std::chrono::duration<double>(1.0 / rate_hz);
            timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::milliseconds>(period),
                std::bind(&HelloNode::timerCallback, this)
            );

            RCLCPP_INFO(this->get_logger(), "HelloNode started with message: '%s' at rate: %.2f Hz", message_.c_str(), rate_hz);
        }

    private:
        void timerCallback() {
            RCLCPP_INFO(this->get_logger(), "%s", message_.c_str());
        }

        std::string message_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HelloNode>());
    rclcpp::shutdown();
    return 0;
}