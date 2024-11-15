#include "CANDriver.h"
#include <string>
#include "Limits.h"
#include "main.h"

#define MAX_PWM 2000
#define MIN_PWM 1000

#define MAX_DUTY_CYCLE 255
#define MIN_DUTY_CYCLE 0

std::array<CANDriver::CANStaticDataGuarded, 2> CANDriver::canStaticData;
std::array<libguarded::plain_guarded<size_t>, 2> CANDriver::canStaticDataUsers;


// TODO: only works with one can bus
std::thread CANDriver::canReadThread;


bool CANDriver::setupCAN(int canBus) {
    auto data = canStaticData[canBus].lock();

    if (data->canBussesSetup) {
        {
            auto data = canStaticDataUsers[canBus].lock();
            *data += 1;
        }
        return true;
    }

    RCLCPP_INFO(node->get_logger(), "Settting up CAN %d",canBus);

    data->soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (data->soc < 0) {
        RCLCPP_ERROR(node->get_logger(), "socket PF_CAN failed");
        return false;
    }

    if (canBus)
        strcpy(data->ifr.ifr_name, "can1");
    else
        strcpy(data->ifr.ifr_name, "can0");

    int ret = ioctl(data->soc, SIOCGIFINDEX, &(data->ifr));
    if (ret < 0) {
        RCLCPP_ERROR(node->get_logger(), "ioctl failed");
        return false;
    }

    data->socketAddress.can_family = AF_CAN;
    data->socketAddress.can_ifindex = data->ifr.ifr_ifindex;
    ret = bind(data->soc, (struct sockaddr *)&(data->socketAddress), sizeof(data->socketAddress));
    if (ret < 0) {
        perror("bind failed");
        return false;
    }

    // setgccsockopt(data->soc, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);


    // CANDriver::canReadThread = std::thread(&CANDriver::startCanReadThread, canBus);   
    data->canBussesSetup = true;

    {
        auto data = canStaticDataUsers[canBus].lock();
        *data = 1;
    }

    return true;
}

bool CANDriver::sendMSG(int canBus, can_frame frame) {
    if (canBus < 0 || canBus > 1) return false;

    auto data = canStaticData[canBus].lock();
    frame.can_id |= CAN_EFF_FLAG;

    int nbytes = write(data->soc, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        RCLCPP_ERROR(node->get_logger(), "CAN Frame Send Error!\r\n");
        return false;
    }
    return true;
}

bool CANDriver::receiveMSG(int canBus, can_frame &frame)
{
    if (canBus < 0 || canBus > 1) return false;

    //Lock the data to get the socket
    auto data = canStaticData[canBus].lock();

    //clear the frame
    memset(&frame, 0, sizeof(frame));

    //block until a frame is received
    int nbytes = read(data->soc, &frame, sizeof(frame));

    if(nbytes != sizeof(frame)) {
        RCLCPP_ERROR(node->get_logger(),"CAN Frame Receive Error!\r\n");
        return false;
    }
    return true;
}

const uint32_t perioticUpdateCanIDBase = 0x82051840;
const uint32_t maxCANID = 7;

void CANDriver::startCanReadThread(int canBus)
{
    RCLCPP_INFO(node->get_logger(), "Starting CAN Read Thread");
    while (true) {
        can_frame frame;
        if (receiveMSG(canBus, frame)) {
            // ROS_INFO("received item: %x",frame.can_id);

            //check if the frame is a periotic update
            if (frame.can_id >= perioticUpdateCanIDBase && frame.can_id < perioticUpdateCanIDBase + maxCANID) {
                //handle periotic update
                // ROS_INFO("Periotic Update Received");

                //print the frame to formated string
                // ROS_INFO("ID: %x", frame.can_id);
                parsePeriodicData(canBus, frame);
            }
            else {
                //handle other frames
            }

            
            //optional wait a little bit
            // std::this_thread::sleep_for(std::chrono::milliseconds(1));//
        }
    }
}

void CANDriver::parsePeriodicData(int canBus, can_frame frame)
{
    // length, dataout[1], dataout[2], dataout[3], dataout[4], dataout[5], dataout[6], dataout[7]
    // 8, Motor Velocity LSB, Motor Velocity MID_L, Motor Velocity MID_H, Motor Velocity MSB, Motor Temperature, Motor Voltage LSB, Motor Current LSB 4 bits Motor Voltage MSB 4 bits, Motor Current MSB

    //parse the data
    PeriodicUpdateData pdata;
    uint32_t velocityFloat = (frame.data[0] | (frame.data[1] << 8) | (frame.data[2] << 16) | (frame.data[3] << 24));
    pdata.velocity = *(reinterpret_cast<float*>(&velocityFloat));
    pdata.temperature = frame.data[4];
    pdata.voltage = (frame.data[5] | ((frame.data[6] & 0x0F) << 8));
    pdata.current = ((frame.data[6] & 0xF0) | (frame.data[7] << 4));

    auto motorID = frame.can_id - perioticUpdateCanIDBase;

    RCLCPP_INFO(node->get_logger(), "CAN ID %d, velocity %.3f, temperature %i, voltage %i, current %i",motorID,pdata.velocity,pdata.temperature,pdata.voltage,pdata.current);
    
    //assign the data to the correct motor
    {
        auto data = canStaticData[canBus].lock();
        
        auto motorPtr = data->canIDMap[motorID];

        if (motorPtr) {
            *motorPtr->lastPeriodicData.lock() = pdata; 
        }
        //todo broadcast ros messages with new data

    }
}

CANDriver::CANDriver(int busNum, int canID) {
    assert(busNum < 2 && busNum >= 0);
    assert(canID > 0);
    this->canBus = busNum;
    this->canID = canID;

    if (setupCAN(busNum)) {
        {
            auto data = canStaticData[busNum].lock();
            data->canIDMap[canID] = this;
        }
        RCLCPP_INFO(rclcpp::get_logger("CANDriver"), "CAN (%d,%d) setup successful",busNum,canID);
    } else {
        RCLCPP_WARN(rclcpp::get_logger("CANDriver"), "Cannot setup CAN");
    }
}

CANDriver::CANDriver(const CANDriver &other)
{
    this->canBus = other.canBus;
    this->canID = other.canID;

    auto data = canStaticDataUsers[canBus].lock();
    *data = *data + 1;
}

void CANDriver::closeCAN(int canBus) {
    if (canBus < 0 || canBus > 1) return;

    auto data = canStaticData[canBus].lock();

    if (!data->canBussesSetup) return;

    close(data->soc);
    data->canBussesSetup = false;
}

CANDriver& CANDriver::operator=(const CANDriver &other)
{
    canBus = other.canBus;
    canID = other.canID;

    return *this;
}

CANDriver::~CANDriver()
{
    auto data = canStaticDataUsers[canBus].lock();
    *data = *data - 1;

    if (*data == 0) {
        RCLCPP_DEBUG(rclcpp::get_logger("CANDriver"), "Shutting down CAN bus");
        closeCAN(canBus);
    }
}

SparkMax::SparkMax(int canBUS, int canID) : CANDriver(canBUS, canID) {}

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

void SparkMax::pidTick()
{
    if (pidControlled) {
        double currentVel = 0;

        {
            auto data = lastPeriodicData.lock();
            currentVel = data->velocity / 15 * 2 * 3.14159265 / 60;
        }

        double val = pidController.calculate(pidSetpoint,currentVel);
        sendPowerCMD(val);
    }
}

void SparkMax::ident()
{
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
