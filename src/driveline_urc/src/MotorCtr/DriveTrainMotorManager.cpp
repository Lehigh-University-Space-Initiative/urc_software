#include "DriveTrainMotorManager.h"

DriveTrainMotorManager::DriveTrainMotorManager()
{
    auto node = rclcpp::Node::make_shared("drive_train_motor_manager");

    setupMotors();

    // Start heartbeat thread
    heartbeatThreadObj = std::thread(&DriveTrainMotorManager::heartbeatThread, this);

    // Subscribe to drive commands
    driveCommandsSub = node->create_subscription<cross_pkg_messages::msg::RoverComputerDriveCMD>(
        "/roverDriveCommands", 10, [this](const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg) {
            auto lock = lastManualCommandTime.lock();
            *lock = std::chrono::system_clock::now();
            parseDriveCommands(msg);
        });

    nodeHandle = node;
}

DriveTrainMotorManager::~DriveTrainMotorManager()
{
    // Signal heartbeat thread to shut down
    {
        auto lock = heartbeatThreadShutDown.lock();
        *lock = true;
    }

    // Wait for the heartbeat thread to shut down
    heartbeatThreadObj.join();
}

void DriveTrainMotorManager::setupMotors()
{
    // Add motors to the motors vector
    // Left side (BUS 0)
    motors.push_back(SparkMax(1, 1)); // LF
    motors.push_back(SparkMax(1, 2)); // LM
    motors.push_back(SparkMax(1, 3)); // LB
    // Right side (BUS 1)
    motors.push_back(SparkMax(1, 4)); // RB
    motors.push_back(SparkMax(1, 5)); // RM
    motors.push_back(SparkMax(1, 6)); // RF

    RCLCPP_INFO(nodeHandle->get_logger(), "Testing Motors");
    for (auto &motor : motors) {
        motor.ident();
    }
}

void DriveTrainMotorManager::stopAllMotors()
{
    for (auto &motor : motors) {
        motor.sendPowerCMD(0);
    }
}

void DriveTrainMotorManager::sendHeartbeats()
{
    for (auto &motor : motors) {
        motor.sendHeartbeat();
    }
}

void DriveTrainMotorManager::heartbeatThread()
{
    rclcpp::Rate loop_rate(50);  // 50 Hz loop rate
    while (rclcpp::ok()) {
        {  // lock block
            auto lock = heartbeatThreadShutDown.lock();
            if (*lock) {
                return;
            }
        }

        {  // lock block
            auto lock = sendHeartbeatsFlag.lock();
            if (*lock) {
                sendHeartbeats();
            }
        }

        {  // lock block for LOS safety stop
            auto lock = lastManualCommandTime.lock();
            auto now = std::chrono::system_clock::now();
            if (now - *lock > manualCommandTimeout) {
                RCLCPP_WARN(nodeHandle->get_logger(), "LOS Safety Stop");
                stopAllMotors();
            }
        }

        loop_rate.sleep();
    }
}

void DriveTrainMotorManager::parseDriveCommands(const cross_pkg_messages::msg::RoverComputerDriveCMD::SharedPtr msg)
{
    RCLCPP_INFO(nodeHandle->get_logger(), "Drive Commands Received with L: %f, R: %f", msg->cmd_l.x, msg->cmd_r.x);

    // Send power commands to motors based on drive command message
    motors[0].sendPowerCMD(msg->cmd_l.x);
    motors[1].sendPowerCMD(msg->cmd_l.y);
    motors[2].sendPowerCMD(msg->cmd_l.z);

    motors[3].sendPowerCMD(msg->cmd_r.x);
    motors[4].sendPowerCMD(msg->cmd_r.y);
    motors[5].sendPowerCMD(msg->cmd_r.z);
}
