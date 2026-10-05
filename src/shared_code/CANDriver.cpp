/**
 * @file
 * Implementation of the CAN bus driver, the SparkMax CAN protocol, and the PWM SparkMax
 *
 * See CANDriver.h for what each class is for and how it fits into the rover.
 *
 * No-hardware mode:
 *   - If can0/can1 can't be opened (WSL, a laptop, hootl), setupCAN logs one loud error per bus
 *   - After that, every send/receive on that bus quietly does nothing, so the node keeps running
 *   - On the rover with the CAN HAT connected, behavior is unchanged
 */
#include "CANDriver.h"
#include "Limits.h"
#include <string>
#include "Logger.h"

/**
 * Wrap an angle that has gone past pi back into the (-pi, pi] range
 *
 * Parameters (inputs):
 *   in - an angle in radians, expected in [0, 2*pi)
 *
 * Return value:
 *   the same angle shifted down by 2*pi if it was above pi, otherwise unchanged
 */
double moveSingularityInRadians(double in)
{
    const double pi = 3.14159265358979;
    RCLCPP_DEBUG(rclcpp::get_logger("CANDriver"), "Absolute angle before wrap: %f", in);
    if (in > pi) {
        RCLCPP_DEBUG(rclcpp::get_logger("CANDriver"), "Wrapping absolute angle past pi");
        return in - 2 * pi;
    }

    return in;
}

// PWM pulse and duty-cycle limits (currently unused; PWMSparkMax computes its own values)
#define MAX_PWM 2000
#define MIN_PWM 1000

#define MAX_DUTY_CYCLE 255
#define MIN_DUTY_CYCLE 0

// Syntax: static class members declared in a header need exactly one definition in a .cpp file (this is where their storage lives)
std::array<CANDriver::CANStaticDataGuarded, 2> CANDriver::canStaticData;
std::array<libguarded::plain_guarded<size_t>, 2> CANDriver::canStaticDataUsers;

// TODO: only works with one CAN bus
std::thread CANDriver::canReadThread;

/**
 * Log, once, that a CAN bus can't be used and the node is continuing without motor hardware
 *
 * Parameters (inputs):
 *   canBus - the bus that failed (0 or 1)
 *   interfaceName - its Linux network interface name ("can0" or "can1")
 *   reason - a short description of which setup step failed
 */
static void reportBusUnavailable(int canBus, const char* interfaceName, const char* reason)
{
    RCLCPP_ERROR(dl_logger,
                 "CAN bus %d (%s) unavailable: %s. Running WITHOUT motor hardware on this bus; motor "
                 "commands will be ignored. This is expected off the rover (WSL, a laptop, hootl mode). "
                 "On the rover, check the CAN HAT with: ip link show %s",
                 canBus, interfaceName, reason, interfaceName);
}

/**
 * Steps:
 *   1. If the bus is already open, count one more user and return
 *   2. If an earlier attempt already failed, return false without retrying (avoids one error per motor)
 *   3. Open a raw SocketCAN socket, look up the interface index of can0/can1, and bind the socket to it
 *   4. On any failure, close the socket and report the bus as unavailable
 */
bool CANDriver::setupCAN(int canBus)
{
    auto data = canStaticData[canBus].lock();

    if (data->canBussesSetup) {
        {
            auto users = canStaticDataUsers[canBus].lock();
            *users += 1;
        }

        return true;
    }

    if (data->setupAttempted) {
        return false;
    }
    data->setupAttempted = true;

    const char* interfaceName = (canBus ? "can1" : "can0");
    RCLCPP_INFO(dl_logger, "Setting up CAN %d (%s)", canBus, interfaceName);

    // Syntax: socket() returns a file descriptor (an int handle); PF_CAN + CAN_RAW asks Linux for raw CAN frames
    data->soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (data->soc < 0) {
        reportBusUnavailable(canBus, interfaceName, "could not create a SocketCAN socket");
        return false;
    }

    // Looking up the interface's index by name (fails when can0/can1 doesn't exist)
    strcpy(data->ifr.ifr_name, interfaceName);
    int ret = ioctl(data->soc, SIOCGIFINDEX, &(data->ifr));
    if (ret < 0) {
        close(data->soc);
        data->soc = -1;
        reportBusUnavailable(canBus, interfaceName, "no such network interface");
        return false;
    }

    // Binding the socket to that interface so reads and writes go to this bus
    data->socketAddress.can_family = AF_CAN;
    data->socketAddress.can_ifindex = data->ifr.ifr_ifindex;
    ret = bind(data->soc, (struct sockaddr*)&(data->socketAddress), sizeof(data->socketAddress));
    if (ret < 0) {
        close(data->soc);
        data->soc = -1;
        reportBusUnavailable(canBus, interfaceName, "could not bind the socket to the interface");
        return false;
    }

    data->canBussesSetup = true;

    {
        auto users = canStaticDataUsers[canBus].lock();
        *users = 1;
    }

    return true;
}

bool CANDriver::isBusAvailable(int canBus)
{
    if ((canBus < 0) || (canBus > 1)) {
        return false;
    }

    auto data = canStaticData[canBus].lock();

    return data->canBussesSetup;
}

bool CANDriver::sendMSG(int canBus, can_frame frame)
{
    if ((canBus < 0) || (canBus > 1)) {
        return false;
    }

    auto data = canStaticData[canBus].lock();

    // Dropping the frame silently in no-hardware mode (the bus failure was already reported once)
    if (!data->canBussesSetup) {
        return false;
    }

    // SparkMax uses 29-bit extended CAN IDs, so the extended-frame flag must be set
    frame.can_id |= CAN_EFF_FLAG;

    int nbytes = write(data->soc, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        RCLCPP_ERROR(dl_logger, "CAN Frame Send Error!\r\n");
        return false;
    }

    return true;
}

/**
 * Steps:
 *   1. Return false right away if the bus was never opened (no-hardware mode, or a bus this node doesn't use)
 *   2. Use select() with a zero timeout to check whether a frame is waiting, without blocking
 *   3. If one is waiting, read exactly one frame
 */
bool CANDriver::receiveMSG(int canBus, can_frame& frame)
{
    if ((canBus < 0) || (canBus > 1)) {
        return false;
    }

    auto data = canStaticData[canBus].lock();

    // Guarding before FD_SET, which is undefined behavior on a closed (-1) socket
    if (!data->canBussesSetup) {
        return false;
    }

    memset(&frame, 0, sizeof(frame));

    // Syntax: an fd_set is a bitmask of file descriptors that select() should watch
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(data->soc, &read_fds);

    // Zero timeout makes select() return immediately instead of waiting for data
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    int result = select(data->soc + 1, &read_fds, NULL, NULL, &timeout);
    if ((result > 0) && FD_ISSET(data->soc, &read_fds)) {
        int nbytes = read(data->soc, &frame, sizeof(frame));
        if (nbytes != sizeof(frame)) {
            RCLCPP_ERROR(dl_logger, "CAN Frame Receive Error!\r\n");
            return false;
        }

        return true;
    }

    return false;
}

// CAN ID ranges of the SparkMax status ("periodic") frames; the device's CAN ID is added to the base
const uint32_t perioticUpdate1CanIDBase = 0x82051840;  // Velocity, temperature, voltage, current
// TODO: this is not really periodic update 2 but 5, to get absolute position for SAR2025
const uint32_t perioticUpdate2CanIDBase = 0x82051880;  // Relative (built-in encoder) position
const uint32_t perioticUpdate5CanIDBase = 0x82051940;  // Absolute encoder position
const uint32_t maxCANID = 63;  // Highest CAN ID a SparkMax can be assigned

void CANDriver::startCanReadThread(int canBus)
{
    RCLCPP_INFO(dl_logger, "Starting CAN Read Thread");
    while (true) {
        can_frame frame;
        if (receiveMSG(canBus, frame)) {
            if ((frame.can_id >= perioticUpdate1CanIDBase) && (frame.can_id < (perioticUpdate1CanIDBase + maxCANID))) {
                parsePeriodicData1(canBus, frame);
            }
        }
    }
}

/**
 * Steps:
 *   1. Try to read one frame from the bus
 *   2. Work out which status frame it is from its CAN ID range, and hand it to the matching parser
 *   3. Ignore any other frame type
 */
bool CANDriver::doCanReadIter(int canBus)
{
    can_frame frame;
    if (receiveMSG(canBus, frame)) {
        if ((frame.can_id >= perioticUpdate1CanIDBase) && (frame.can_id < (perioticUpdate1CanIDBase + maxCANID))) {
            parsePeriodicData1(canBus, frame);
        }
        else if ((frame.can_id >= perioticUpdate2CanIDBase) && (frame.can_id < (perioticUpdate2CanIDBase + maxCANID))) {
            parsePeriodicData2(canBus, frame);
        }
        else if ((frame.can_id >= perioticUpdate5CanIDBase) && (frame.can_id < (perioticUpdate5CanIDBase + maxCANID))) {
            parsePeriodicData5(canBus, frame);
        }

        return true;
    }

    return false;
}

/**
 * Steps:
 *   1. Reassemble the little-endian 32-bit float velocity from data bytes 0-3
 *   2. Unpack temperature (byte 4) and the 12-bit voltage and current values packed into bytes 5-7
 *   3. Find the device with this CAN ID and store the decoded data on it
 *
 * Frame layout (8 bytes):
 *   Motor Velocity LSB, MID_L, MID_H, MSB, Motor Temperature, Voltage LSB,
 *   Current LSB 4 bits + Voltage MSB 4 bits, Current MSB
 */
void CANDriver::parsePeriodicData1(int canBus, can_frame frame)
{
    PeriodicUpdateData1 pdata{};

    // Combining four bytes into one 32-bit integer, lowest byte first (little-endian)
    uint32_t velocityFloat = (frame.data[0] | (frame.data[1] << 8) | (frame.data[2] << 16) | (frame.data[3] << 24));
    // Syntax: reinterpret_cast reads the same 32 bits as a float instead of an integer (no conversion)
    pdata.velocity = *(reinterpret_cast<float*>(&velocityFloat));
    pdata.temperature = frame.data[4];
    pdata.voltage = (frame.data[5] | ((frame.data[6] & 0x0F) << 8));
    pdata.current = ((frame.data[6] & 0xF0) | (frame.data[7] << 4));

    auto motorID = frame.can_id - perioticUpdate1CanIDBase;

    CANDriver* motorPtr = nullptr;
    {
        auto data = canStaticData[canBus].lock();
        motorPtr = data->canIDMap[motorID];
    }

    if (motorPtr) {
        motorPtr->lastPeriodicData1 = pdata;
        // TODO: copy rest of params
    }
    else {
        RCLCPP_WARN(rclcpp::get_logger("CANDriver"), "no motor found THIS IS A BIG DEAL");
    }
}

void CANDriver::parsePeriodicData2(int canBus, can_frame frame)
{
    PeriodicUpdateData2 pdata;

    // Same little-endian float reassembly as parsePeriodicData1 (position in revolutions from zero)
    uint32_t positionFloat = (frame.data[0] | (frame.data[1] << 8) | (frame.data[2] << 16) | (frame.data[3] << 24));
    pdata.position = *(reinterpret_cast<float*>(&positionFloat));

    auto motorID = frame.can_id - perioticUpdate2CanIDBase;

    CANDriver* motorPtr = nullptr;
    {
        auto data = canStaticData[canBus].lock();
        motorPtr = data->canIDMap[motorID];
    }

    if (motorPtr) {
        motorPtr->lastPeriodicData2 = pdata;
    }
    else {
        RCLCPP_WARN(rclcpp::get_logger("CANDriver"), "PERIODIC 2 no motor found THIS IS A BIG DEAL");
    }
}

void CANDriver::parsePeriodicData5(int canBus, can_frame frame)
{
    PeriodicUpdateData2 pdata;

    // Same little-endian float reassembly (absolute encoder position in revolutions)
    uint32_t positionFloat = (frame.data[0] | (frame.data[1] << 8) | (frame.data[2] << 16) | (frame.data[3] << 24));
    pdata.position = *(reinterpret_cast<float*>(&positionFloat));

    auto motorID = frame.can_id - perioticUpdate5CanIDBase;
    RCLCPP_DEBUG(rclcpp::get_logger("CANDriver"), "GOT motor %d pos of: %.2f", motorID, pdata.position);

    CANDriver* motorPtr = nullptr;
    {
        auto data = canStaticData[canBus].lock();
        motorPtr = data->canIDMap[motorID];
    }

    if (motorPtr) {
        motorPtr->lastPeriodicData5 = pdata;
    }
    else {
        RCLCPP_WARN(rclcpp::get_logger("CANDriver"), "PERIODIC 5 no motor found THIS IS A BIG DEAL");
    }
}

CANDriver::CANDriver(int busNum, int canID)
{
    // Syntax: assert() stops the program in debug builds if the condition is false (a programming error)
    assert((busNum < 2) && (busNum >= 0));
    assert(canID > 0);
    this->canBus = busNum;
    this->canID = canID;

    if (setupCAN(busNum)) {
        {
            auto data = canStaticData[busNum].lock();
            data->canIDMap[canID] = this;
        }
        RCLCPP_INFO(rclcpp::get_logger("CANDriver"), "CAN (%d,%d) setup successful", busNum, canID);
    }
    else {
        // The bus-level failure was already reported once by setupCAN
        RCLCPP_DEBUG(rclcpp::get_logger("CANDriver"), "CAN (%d,%d) running without hardware", busNum, canID);
    }
}

CANDriver::CANDriver(const CANDriver& other)
{
    this->canBus = other.canBus;
    this->canID = other.canID;

    {
        auto data = canStaticDataUsers[canBus].lock();
        *data = *data + 1;
    }

    // Pointing the ID map at the new copy, since the old object may be about to be destroyed
    {
        auto data = canStaticData[canBus].lock();
        data->canIDMap[canID] = this;
    }
}

void CANDriver::closeCAN(int canBus)
{
    if ((canBus < 0) || (canBus > 1)) {
        return;
    }

    auto data = canStaticData[canBus].lock();

    if (!data->canBussesSetup) {
        return;
    }

    close(data->soc);
    data->soc = -1;
    data->canBussesSetup = false;
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

double SparkMax::lastVelocityAsRadPerSec()
{
    // Each wheel has a 3:1 and a 4:1 gearbox, for a total of 12:1
    // TODO: fix for drivetrain
    double rpmToRadPerSec = 2 * 3.14159265 / 60;  // One revolution is 2*pi radians; one minute is 60 seconds

    return lastPeriodicData1.velocity / gearRatio * rpmToRadPerSec;
}

double SparkMax::lastPositionInRad()
{
    double revToRad = 2 * 3.14159265;  // Converts revolutions to radians

    return lastPeriodicData2.position / gearRatio * revToRad;
}

double SparkMax::lastAbsPositionInRad()
{
    double revToRad = 2 * 3.14159265;  // Converts revolutions to radians

    return moveSingularityInRadians(lastPeriodicData5.position * revToRad);
}

double SparkMax::lastCorrectPos()
{
    double pos = 0;

    if (useAbsolute) {
        pos = lastAbsPositionInRad();
    }
    else {
        pos = lastPositionInRad();
    }

    return pos;
}

/**
 * Steps:
 *   1. Read the PID gains and the readOnly flag from the owning node's ROS parameters
 *   2. Build the PID with a fixed 5 ms time step and output limited to +/-0.15 power
 */
void SparkMax::setupPID()
{
    double kp = node_->get_parameter("kp").as_double();
    double ki = node_->get_parameter("ki").as_double();
    double kd = node_->get_parameter("kd").as_double();
    double max_i = node_->get_parameter("max_i").as_double();
    bool viewOnly = node_->get_parameter("readOnly").as_bool();

    RCLCPP_INFO(rclcpp::get_logger("SparkMax"), "Creating PID with Kp: %f, Ki: %f, Kd: %f; readOnly: %d", kp, ki, kd, viewOnly);

    // PID(dt, max, min, Kp, Kd, Ki, max_i): note the argument order puts Kd before Ki
    this->pidController = PID(0.005, 0.15, -0.15, kp, kd, ki, max_i);
    this->viewOnly = viewOnly;
}

// Syntax: ": CANDriver(canBUS, canID)" runs the base-class constructor before this constructor's body
SparkMax::SparkMax(rclcpp::Node::SharedPtr node, int canBUS, int canID, double gearRatio, bool useAbsolute)
    : CANDriver(canBUS, canID)
{
    assert(gearRatio > 0);
    this->node_ = node;
    this->gearRatio = gearRatio;
    this->useAbsolute = useAbsolute;
    RCLCPP_INFO(rclcpp::get_logger("SparkMax"), "Creating motor %d with gear ratio %.5f", canID, gearRatio);

    setupPID();
}

SparkMax::SparkMax(const SparkMax& other) : CANDriver(other)
{
    this->gearRatio = other.gearRatio;
    this->node_ = other.node_;
    this->viewOnly = other.viewOnly;
    this->useAbsolute = other.useAbsolute;

    sendAbsoluteFrameUpdateRate();
    setupPID();
}

bool SparkMax::sendHeartbeat()
{
    can_frame frame{};
    frame.can_id = 0x02052C80 + canID;  // SparkMax heartbeat API ID plus this device's CAN ID
    frame.can_dlc = 8;
    for (int i = 0; i < 8; i++) {
        frame.data[i] = 0xFF;  // All bits set enables every device that listens to this heartbeat
    }

    return sendMSG(canBus, frame);
}

bool SparkMax::sendAbsoluteFrameUpdateRate()
{
    can_frame frame{};
    frame.can_id = 0x02051940 + canID;  // Periodic frame 5 configuration API ID plus this device's CAN ID
    frame.can_dlc = 2;
    frame.data[0] = 10;  // Update period in milliseconds

    return sendMSG(canBus, frame);
}

/**
 * Steps:
 *   1. Clamp the power to +/-MAX_DRIVE_POWER and zero it inside the deadband
 *   2. Pack the float's raw bytes into a duty-cycle set-point frame and send it
 *   3. Warn on a failed send, but only when the bus actually exists (no-hardware mode stays quiet)
 */
void SparkMax::sendPowerCMD(float power)
{
    power = std::min(std::max(power, -MAX_DRIVE_POWER), MAX_DRIVE_POWER);
    if (abs(power) < DRIVE_DEADBAND) {
        power = 0;
    }

    can_frame frame{};
    frame.can_id = 0x2050080 + canID;  // Duty-cycle set-point API ID plus this device's CAN ID
    frame.can_dlc = 6;
    // Copying the float's 4 raw bytes into the frame (the SparkMax expects an IEEE-754 float)
    memcpy(frame.data, (int*)(&power), sizeof(float));
    frame.data[4] = 0;
    frame.data[5] = 0;

    if (sendMSG(canBus, frame)) {
        return;
    }

    if (isBusAvailable(canBus)) {
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
        double pos = lastCorrectPos();

        double val = pidController.calculate(pidSetpoint, pos);
        RCLCPP_INFO(rclcpp::get_logger("SparkMax"), "running pid %d with set: %f, cur: %f (use abs: %d) output: %f (integral: %f)", canID, pidSetpoint, pos, useAbsolute, val, pidController.i_sum());

        if (!viewOnly) {
            sendPowerCMD(val);
        }
    }
}

void SparkMax::ident()
{
    can_frame frame{};
    frame.can_id = 0x2051D80 + canID;  // Identify API ID plus this device's CAN ID
    frame.can_dlc = 0;

    RCLCPP_INFO(rclcpp::get_logger("SparkMax"), "Sending ident message to CAN ID: %X (%X)", frame.can_id, canID);

    sendMSG(canBus, frame);
}

bool PWMSparkMax::gpioSetup = false;

void PWMSparkMax::setupGPIO()
{
    if (gpioSetup) {
        return;
    }

    // gpioInitialise fails when not running on a Raspberry Pi (or without permission to the GPIO hardware)
    if (gpioInitialise() < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("PWMSparkMax"), "pigpio initialisation failed");
        throw std::runtime_error("pigpio initialisation failed");
    }

    gpioSetup = true;
}

// Syntax: ": pin(pin)" is a member initializer list; it sets the member pin from the parameter pin
PWMSparkMax::PWMSparkMax(int pin) : pin(pin)
{
    setupGPIO();
    setPower(0);
}

void PWMSparkMax::terminateGPIO()
{
    gpioTerminate();
}

/**
 * Steps:
 *   1. Clamp the power to +/-0.8 and skip the update if it matches the last value sent
 *   2. Configure the pin's PWM range (and its frequency, the first time)
 *   3. Map power to a 1000-2000 microsecond pulse (1500 is neutral) and convert it to a duty cycle
 */
void PWMSparkMax::setPower(float power)
{
    float limitPower = 0.8f;
    float deadZone = 0.08f;  // Unused

    power = std::min(std::max(power, -limitPower), limitPower);

    if (power == lastSentValue) {
        return;
    }
    lastSentValue = power;

    if (gpioInitialise() < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("PWMSparkMax"), "pigpio initialisation failed");
        throw std::runtime_error("pigpio initialisation failed");
    }

    auto freq = 100;  // PWM frequency in Hz (a 10 ms period)
    auto maxRange = 2000;  // Duty-cycle resolution: 2000 steps per period
    gpioSetPWMrange(pin, maxRange);
    if (!initialSetup) {
        gpioSetPWMfrequency(pin, freq);
        initialSetup = true;
    }

    // Standard RC-style pulse: 1000 us is full reverse, 1500 us is neutral, 2000 us is full forward
    float pulseWidth = (power * 500 + 1500);
    // Converting the pulse width in microseconds to steps out of maxRange at this frequency
    int dutyCycle = (int)(pulseWidth / 1000000 * freq * maxRange);

    RCLCPP_INFO(rclcpp::get_logger("PWMSparkMax"), "Duty cycle: %d", dutyCycle);
    gpioPWM(pin, dutyCycle);
}

PWMSparkMax::~PWMSparkMax()
{
    setPower(0);
}
