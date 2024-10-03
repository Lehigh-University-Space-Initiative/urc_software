#include "CANDriver.h"
#include <string>
#include "Limits.h"

#define MAX_PWM 2000
#define MIN_PWM 1000

#define MAX_DUTY_CYCLE 255
#define MIN_DUTY_CYCLE 0

std::array<CANDriver::CANStaticDataGuarded, 2> CANDriver::canStaticData;
std::array<libguarded::plain_guarded<size_t>, 2> CANDriver::canStaticDataUsers;

bool CANDriver::setupCAN(int canBus) {
    auto data = canStaticData[canBus].lock();

    if (data->canBussesSetup)
        return true;

    data->soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (data->soc < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("CANDriver"), "socket PF_CAN failed");
        return false;
    }

    if (canBus)
        strcpy(data->ifr.ifr_name, "can1");
    else
        strcpy(data->ifr.ifr_name, "can0");

    int ret = ioctl(data->soc, SIOCGIFINDEX, &(data->ifr));
    if (ret < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("CANDriver"), "ioctl failed");
        return false;
    }

    data->socketAddress.can_family = AF_CAN;
    data->socketAddress.can_ifindex = data->ifr.ifr_ifindex;
    ret = bind(data->soc, (struct sockaddr *)&(data->socketAddress), sizeof(data->socketAddress));
    if (ret < 0) {
        perror("bind failed");
        return false;
    }

    setsockopt(data->soc, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    data->canBussesSetup = true;

    return true;
}

bool CANDriver::sendMSG(int canBus, can_frame frame) {
    if (canBus < 0 || canBus > 1) return false;

    auto data = canStaticData[canBus].lock();
    frame.can_id |= CAN_EFF_FLAG;

    int nbytes = write(data->soc, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        RCLCPP_ERROR(rclcpp::get_logger("CANDriver"), "CAN Frame Send Error!\r\n");
        return false;
    }
    return true;
}

CANDriver::CANDriver(int busNum) {
    assert(busNum < 2 && busNum >= 0);
    this->canBus = busNum;

    if (setupCAN(busNum)) {
        RCLCPP_INFO(rclcpp::get_logger("CANDriver"), "CAN setup successful");
    } else {
        RCLCPP_WARN(rclcpp::get_logger("CANDriver"), "Cannot setup CAN");
    }
}

void CANDriver::closeCAN(int canBus) {
    if (canBus < 0 || canBus > 1) return;

    auto data = canStaticData[canBus].lock();

    if (!data->canBussesSetup) return;

    close(data->soc);
    data->canBussesSetup = false;
}

CANDriver::~CANDriver() {
    auto data = canStaticDataUsers[canBus].lock();
    *data = *data - 1;

    if (*data == 0) {
        RCLCPP_DEBUG(rclcpp::get_logger("CANDriver"), "Shutting down CAN bus");
        closeCAN(canBus);
    }
}

SparkMax::SparkMax(int canBUS, int canID) : CANDriver(canBUS), canID(canID) {}

bool SparkMax::sendHeartbeat() {
    can_frame frame{};
    frame.can_id = 0x02052C80 + canID;
    frame.can_dlc = 8;
    for (int i = 0; i < 8; i++) {
        frame.data[i] = 0xFF;
    }

    return sendMSG(canBus, frame);
}

void SparkMax::sendPowerCMD(float power) {
    power = std::min(std::max(power, -MAX_DRIVE_POWER), MAX_DRIVE_POWER);
    if (abs(power) < DRIVE_DEADBAND) power = 0;

    can_frame frame{};
    frame.can_id = 0x2050080 + canID;
    frame.can_dlc = 6;
    memcpy(frame.data, (int*)(&power), sizeof(float));
    frame.data[4] = 0;
    frame.data[5] = 0;

    if (sendMSG(canBus, frame)) {
        return;
    } else {
        RCLCPP_WARN(rclcpp::get_logger("SparkMax"), "CAN Spark MAX Speed message failed to send");
    }
}

void SparkMax::ident() {
    can_frame frame{};
    frame.can_id = 0x2051D80 + canID;
    frame.can_dlc = 0;

    RCLCPP_INFO(rclcpp::get_logger("SparkMax"), "Sending ident message to CAN ID: %x", frame.can_id);

    sendMSG(canBus, frame);
}

bool PWMSparkMax::gpioSetup = false;

void PWMSparkMax::setupGPIO() {
    if (gpioSetup) return;

    if (gpioInitialise() < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("PWMSparkMax"), "pigpio initialisation failed");
        throw std::runtime_error("pigpio initialisation failed");
    }

    gpioSetup = true;
}

PWMSparkMax::PWMSparkMax(int pin) : pin(pin) {
    setupGPIO();
    setPower(0);
}

void PWMSparkMax::terminateGPIO() {
    gpioTerminate();
}

void PWMSparkMax::setPower(float power) {
    float limitPower = 0.8f;
    float deadZone = 0.08f;

    power = std::min(std::max(power, -limitPower), limitPower);

    if (power == lastSentValue) return;
    lastSentValue = power;

    if (gpioInitialise() < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("PWMSparkMax"), "pigpio initialisation failed");
        throw std::runtime_error("pigpio initialisation failed");
    }

    auto freq = 100;
    auto maxRange = 2000;
    gpioSetPWMrange(pin, maxRange);
    if (!initialSetup) {
        gpioSetPWMfrequency(pin, freq);
        initialSetup = true;
    }

    float pulseWidth = (power * 500 + 1500);
    int dutyCycle = (int)(pulseWidth / 1000000 * freq * maxRange);

    RCLCPP_INFO(rclcpp::get_logger("PWMSparkMax"), "Duty cycle: %d", dutyCycle);
    gpioPWM(pin, dutyCycle);
}

PWMSparkMax::~PWMSparkMax() {
    setPower(0);
}
