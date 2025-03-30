#include "CANDriver.h"
#include "Limits.h"
#include <string>
#include "Logger.h"

double moveSingularityInRadians(double in) {
    const double pi = 3.14159265358979;
    RCLCPP_ERROR(dl_logger, "og: %f", in);
    if (in > pi) {
        RCLCPP_ERROR(dl_logger, "fleep");
        return in - 2 * pi;
    }
    return in;
}

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

    RCLCPP_INFO(dl_logger, "Setting up CAN %d",canBus);

    data->soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (data->soc < 0) {
        RCLCPP_ERROR(dl_logger, "socket PF_CAN failed");
        return false;
    }

    if (canBus)
        strcpy(data->ifr.ifr_name, "can1");
    else
        strcpy(data->ifr.ifr_name, "can0");

    int ret = ioctl(data->soc, SIOCGIFINDEX, &(data->ifr));
    if (ret < 0) {
        RCLCPP_ERROR(dl_logger, "ioctl failed");
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
        RCLCPP_ERROR(dl_logger, "CAN Frame Send Error!\r\n");
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

    // Set up the file descriptor set
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(data->soc, &read_fds);

    // Set up the timeout with zero seconds for non-blocking
    struct timeval timeout;
    timeout.tv_sec = 0; // Zero seconds
    timeout.tv_usec = 0; // Zero microseconds

    // Use select to check if data is available
    int result = select(data->soc + 1, &read_fds, NULL, NULL, &timeout);
    if (result > 0 && FD_ISSET(data->soc, &read_fds)) {
        // Data is available; perform the read
        int nbytes = read(data->soc, &frame, sizeof(frame));
        if (nbytes != sizeof(frame)) {
            RCLCPP_ERROR(dl_logger, "CAN Frame Receive Error!\r\n");
            return false;
        }
        return true;
    }
    
    // No data available or error
    return false;

    // //block until a frame is received
    // int nbytes = read(data->soc, &frame, sizeof(frame));
    // int nbytes = select()

    // if(nbytes != sizeof(frame)) {
    //     RCLCPP_ERROR(dl_logger,"CAN Frame Receive Error!\r\n");
    //     return false;
    // }
    // return true;
}

const uint32_t perioticUpdate1CanIDBase = 0x82051840;
//TODO: this is not really periodic update 2 but 5 to get absolute position for SARS COV-2 (SAR2025)
const uint32_t perioticUpdate2CanIDBase = 0x82051880;
const uint32_t perioticUpdate5CanIDBase = 0x82051940;
const uint32_t maxCANID = 63;

void CANDriver::startCanReadThread(int canBus)
{
    RCLCPP_INFO(dl_logger, "Starting CAN Read Thread");
    while (true) {
        can_frame frame;
        if (receiveMSG(canBus, frame)) {
            // ROS_INFO("received item: %x",frame.can_id);

            //check if the frame is a periotic update
            if (frame.can_id >= perioticUpdate1CanIDBase && frame.can_id < perioticUpdate1CanIDBase + maxCANID) {
                //handle periotic update
                // ROS_INFO("Periotic Update Received");

                //print the frame to formated string
                // ROS_INFO("ID: %x", frame.can_id);
                parsePeriodicData1(canBus, frame);
            }
            else {
                //handle other frames
            }

            
            //optional wait a little bit
            // std::this_thread::sleep_for(std::chrono::milliseconds(1));//
        }
    }
}

bool CANDriver::doCanReadIter(int canBus)
{
        can_frame frame;
        // RCLCPP_INFO(dl_logger,"checking can");
        if (receiveMSG(canBus, frame)) {
            // RCLCPP_INFO(dl_logger,"received item: %x",frame.can_id);

            //check if the frame is a periotic update
            if (frame.can_id >= perioticUpdate1CanIDBase && frame.can_id < perioticUpdate1CanIDBase + maxCANID) {
                parsePeriodicData1(canBus, frame);
            }
            else if (frame.can_id >= perioticUpdate2CanIDBase && frame.can_id < perioticUpdate2CanIDBase + maxCANID) {
                parsePeriodicData2(canBus, frame);
            }
            else if (frame.can_id >= perioticUpdate5CanIDBase && frame.can_id < perioticUpdate5CanIDBase + maxCANID) {
                parsePeriodicData5(canBus, frame);
            }
            else {
                //handle other frames
            }

            
            //optional wait a little bit
            // std::this_thread::sleep_for(std::chrono::milliseconds(1));//
            return true;
        } else {
            return false;
        }
}

void CANDriver::parsePeriodicData1(int canBus, can_frame frame)
{
    // length, dataout[1], dataout[2], dataout[3], dataout[4], dataout[5], dataout[6], dataout[7]
    // 8, Motor Velocity LSB, Motor Velocity MID_L, Motor Velocity MID_H, Motor Velocity MSB, Motor Temperature, Motor Voltage LSB, Motor Current LSB 4 bits Motor Voltage MSB 4 bits, Motor Current MSB

    //parse the data
    PeriodicUpdateData1 pdata{};
    uint32_t velocityFloat = (frame.data[0] | (frame.data[1] << 8) | (frame.data[2] << 16) | (frame.data[3] << 24));
    pdata.velocity = *(reinterpret_cast<float*>(&velocityFloat));
    pdata.temperature = frame.data[4];
    pdata.voltage = (frame.data[5] | ((frame.data[6] & 0x0F) << 8));
    pdata.current = ((frame.data[6] & 0xF0) | (frame.data[7] << 4));

    auto motorID = frame.can_id - perioticUpdate1CanIDBase;

    
    //assign the data to the correct motor
    CANDriver* motorPtr = nullptr;
    {
        auto data = canStaticData[canBus].lock();
        
        motorPtr = data->canIDMap[motorID];
    }

    if (motorPtr) {
        // motorPtr->lastPeriodicData.velocity.store(pdata.velocity.load());
        motorPtr->lastPeriodicData1 = pdata;
        // RCLCPP_INFO(dl_logger, "CAN ID %d, velocity %.3f, temperature %i, voltage %i, current %i",motorID,pdata.velocity,pdata.temperature,pdata.voltage,pdata.current);
        // RCLCPP_INFO(rclcpp::get_logger("CANDriver"), "did  motor found yes. big happy! %f cur",motorPtr->lastPeriodicData.velocity);
        //TODO: copy rest of params
    } else {
        RCLCPP_WARN(rclcpp::get_logger("CANDriver"), "no motor found THIS IS A BIG DEAL");
    }
        //todo broadcast ros messages with new data

}

void CANDriver::parsePeriodicData2(int canBus, can_frame frame)
{
    // length, dataout[1], dataout[2], dataout[3], dataout[4], dataout[5], dataout[6], dataout[7]
    // 8, Motor Velocity LSB, Motor Velocity MID_L, Motor Velocity MID_H, Motor Velocity MSB, Motor Temperature, Motor Voltage LSB, Motor Current LSB 4 bits Motor Voltage MSB 4 bits, Motor Current MSB

    //parse the data
    PeriodicUpdateData2 pdata;
    uint32_t positionFloat = (frame.data[0] | (frame.data[1] << 8) | (frame.data[2] << 16) | (frame.data[3] << 24));
    //position from motor is in revolutions from zero
    pdata.position = *(reinterpret_cast<float*>(&positionFloat));

    auto motorID = frame.can_id - perioticUpdate2CanIDBase;
    // RCLCPP_WARN(rclcpp::get_logger("CANDriver"), "GOT motor %d pos of: %.2f",motorID,pdata.position);

    
    //assign the data to the correct motor
    CANDriver* motorPtr = nullptr;
    {
        auto data = canStaticData[canBus].lock();
        
        motorPtr = data->canIDMap[motorID];
    }

    if (motorPtr) {
        // motorPtr->lastPeriodicData.velocity.store(pdata.velocity.load());
        motorPtr->lastPeriodicData2 = pdata;
        // RCLCPP_INFO(dl_logger, "CAN ID %d, velocity %.3f, temperature %i, voltage %i, current %i",motorID,pdata.velocity,pdata.temperature,pdata.voltage,pdata.current);
        // RCLCPP_INFO(rclcpp::get_logger("CANDriver"), "did  motor found yes. big happy! %f cur",motorPtr->lastPeriodicData.velocity);
        //TODO: copy rest of params
    } else {
        RCLCPP_WARN(rclcpp::get_logger("CANDriver"), "PERIODIC 2 no motor found THIS IS A BIG DEAL");
    }
        //todo broadcast ros messages with new data

}


void CANDriver::parsePeriodicData5(int canBus, can_frame frame)
{
    // length, dataout[1], dataout[2], dataout[3], dataout[4], dataout[5], dataout[6], dataout[7]
    // 8, Motor Velocity LSB, Motor Velocity MID_L, Motor Velocity MID_H, Motor Velocity MSB, Motor Temperature, Motor Voltage LSB, Motor Current LSB 4 bits Motor Voltage MSB 4 bits, Motor Current MSB

    //parse the data
    PeriodicUpdateData2 pdata;
    uint32_t positionFloat = (frame.data[0] | (frame.data[1] << 8) | (frame.data[2] << 16) | (frame.data[3] << 24));
    //position from motor is in revolutions from zero
    pdata.position = *(reinterpret_cast<float*>(&positionFloat));

    auto motorID = frame.can_id - perioticUpdate5CanIDBase;
    RCLCPP_WARN(rclcpp::get_logger("CANDriver"), "GOT motor %d pos of: %.2f",motorID,pdata.position);

    
    //assign the data to the correct motor
    CANDriver* motorPtr = nullptr;
    {
        auto data = canStaticData[canBus].lock();
        
        motorPtr = data->canIDMap[motorID];
    }

    if (motorPtr) {
        // motorPtr->lastPeriodicData.velocity.store(pdata.velocity.load());
        motorPtr->lastPeriodicData5 = pdata;
        // RCLCPP_INFO(dl_logger, "CAN ID %d, velocity %.3f, temperature %i, voltage %i, current %i",motorID,pdata.velocity,pdata.temperature,pdata.voltage,pdata.current);
        // RCLCPP_INFO(rclcpp::get_logger("CANDriver"), "did  motor found yes. big happy! %f cur",motorPtr->lastPeriodicData.velocity);
        //TODO: copy rest of params
    } else {
        RCLCPP_WARN(rclcpp::get_logger("CANDriver"), "PERIODIC 2 no motor found THIS IS A BIG DEAL");
    }
        //todo broadcast ros messages with new data

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

    {
        auto data = canStaticDataUsers[canBus].lock();
        *data = *data + 1;
    }

    {
        auto data = canStaticData[canBus].lock();
        data->canIDMap[canID] = this;
    }
}

void CANDriver::closeCAN(int canBus) {
    if (canBus < 0 || canBus > 1) return;

    auto data = canStaticData[canBus].lock();

    if (!data->canBussesSetup) return;

    close(data->soc);
    data->canBussesSetup = false;
}

// CANDriver& CANDriver::operator=(const CANDriver &other)
// {
//     canBus = other.canBus;
//     canID = other.canID;

//     return *this;
// }

CANDriver::~CANDriver()
{
    auto data = canStaticDataUsers[canBus].lock();
    *data = *data - 1;

    if (*data == 0) {
        RCLCPP_DEBUG(rclcpp::get_logger("CANDriver"), "Shutting down CAN bus");
        closeCAN(canBus);
    }
}

double SparkMax::lastVelocityAsRadPerSec()
{
    // in rad / second

    // each wheel has a 3:1 and a 4:1 gear box leading ot a total of 12:1
    //TODO fix for drivetrain
    double rpmToRadPerSec = 2 * 3.14159265 / 60;

    return  lastPeriodicData1.velocity / gearRatio * rpmToRadPerSec;
}

double SparkMax::lastPositionInRad()
{
    double rpmToRadPerSec = 2 * 3.14159265;

    return lastPeriodicData2.position / gearRatio * rpmToRadPerSec;
}

double SparkMax::lastAbsPositionInRad()
{
    double rpmToRadPerSec = 2 * 3.14159265;

    return moveSingularityInRadians(lastPeriodicData5.position * rpmToRadPerSec);
}

double SparkMax::lastCorrectPos()
{
    double pos = 0;

    if (useAbsolute)
    {
        pos = lastAbsPositionInRad();
    }
    else
    {
        pos = lastPositionInRad();
    }

    return pos;
}

void SparkMax::setupPID()
{
    double kp = node_->get_parameter("kp").as_double();
    double ki = node_->get_parameter("ki").as_double();
    double kd = node_->get_parameter("kd").as_double();
    double max_i = node_->get_parameter("max_i").as_double();
    bool viewOnly = node_->get_parameter("readOnly").as_bool();


    RCLCPP_INFO(rclcpp::get_logger("SparkMax"), "Creating PID with Kp: %f, Ki: %f, Kd: %f; readOnly: %d",kp,ki,kd,viewOnly);


    this->pidController = PID(0.005,0.15,-0.15,kp,kd,ki, max_i);
    this->viewOnly = viewOnly;
}

SparkMax::SparkMax(rclcpp::Node::SharedPtr node, int canBUS, int canID, double gearRatio, bool useAbsolute) : CANDriver(canBUS, canID)
{
    assert(gearRatio > 0);
    this->node_ = node;
    this->gearRatio = gearRatio;
    this->useAbsolute = useAbsolute;
    RCLCPP_INFO(rclcpp::get_logger("SparkMax"), "Creating motor %d with gear ratio %.5f",canID,gearRatio);


    setupPID();
}

SparkMax::SparkMax(const SparkMax &other): CANDriver(other)
{
    this->gearRatio = other.gearRatio;
    this->node_ = other.node_;
    this->viewOnly = other.viewOnly;
    this->useAbsolute = other.useAbsolute;

    sendAbsoluteFrameUpdateRate();
    setupPID();
}

bool SparkMax::sendHeartbeat() {
    can_frame frame{};
    frame.can_id = 0x02052C80 + canID;
    frame.can_dlc = 8;
    for (int i = 0; i < 8; i++) {
        frame.data[i] = 0xFF;
    }

    return sendMSG(canBus, frame);
}

bool SparkMax::sendAbsoluteFrameUpdateRate() {
    can_frame frame{};
    frame.can_id = 0x02051940 + canID;
    frame.can_dlc = 2;
    frame.data[0] = 10;

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

void SparkMax::setPIDSetpoint(double pidSetpoint)
{
    this->pidSetpoint = pidSetpoint;
}

void SparkMax::pidTick(double _)
{
    if (pidControlled && !motorLocked) {
        // double currentVel = lastVelocityAsRadPerSec(); 

        double pos = lastCorrectPos();

        double val = pidController.calculate(pidSetpoint,pos);
        RCLCPP_INFO(rclcpp::get_logger("SparkMax"), "running pid %d with set: %f, cur: %f (use abs: %d) output: %f (integral: %f)", canID, pidSetpoint, pos,useAbsolute,val, pidController.i_sum());

        if (!viewOnly) {
            sendPowerCMD(val);
        } else {
            // RCLCPP_INFO(rclcpp::get_logger("SparkMax"), "view only not powering motor");
        }
    }
}

void SparkMax::ident()
{
    can_frame frame{};
    frame.can_id = 0x2051D80 + canID;
    frame.can_dlc = 0;

    RCLCPP_INFO(rclcpp::get_logger("SparkMax"), "Sending ident message to CAN ID: %X (%X)", frame.can_id,canID);

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
