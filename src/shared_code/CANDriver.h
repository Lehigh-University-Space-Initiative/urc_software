/**
 * @file
 * CAN bus access and REV SparkMax motor controller drivers shared by the rover packages
 *
 * Runs on: the driveline Pi (driveline_urc) and the arm Pi (arm_urc)
 * Compiled into: MotorCtr_node and ArmMotorManager (via file(GLOB ../shared_code/*.cpp))
 *
 * How it connects to the system:
 *   - The Pis talk to motor controllers over a CAN bus, a two-wire network common in robots and cars
 *   - The rover uses a Waveshare 2-channel CAN HAT, which shows up in Linux as network interfaces can0 and can1
 *   - Each SparkMax on the bus has a CAN ID; MotorManager (MotorManager.h) owns one SparkMax object per motor
 *   - Off the rover (WSL, a laptop, hootl mode) can0/can1 do not exist, so the driver runs in no-hardware mode
 *
 * Classes in this file:
 *   - CANDriver: one device on a CAN bus, plus the shared (static) socket for each bus
 *   - SparkMax: a CANDriver that speaks the SparkMax protocol (power commands, heartbeats, encoder reads)
 *   - PWMSparkMax: a SparkMax driven by a PWM signal from a Pi GPIO pin instead of CAN (uses pigpio)
 */
#pragma once

#include "rclcpp/rclcpp.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <array>
#include <map>
#include <thread>
#include <vector>
#include <pigpio.h>
#include "cs_libguarded/cs_plain_guarded.h"
#include "pid.h"
#include "Limits.h"

/**
 * One device that can be communicated with over a CAN bus
 *
 * There are assumed to be only 2 CAN buses, since the code is designed for the Waveshare 2-channel Pi CAN HAT
 *
 * Every CANDriver on the same bus shares one Linux socket for that bus (the static canStaticData below).
 * The socket is opened by the first device constructed on a bus and closed when the last one is destroyed.
 *
 * @todo This class should not deal with SparkMax-specific concepts like periodic updates
 */
class CANDriver {
protected:
    /// CAN bus this device is connected to (valid values are 0 and 1, meaning can0 and can1)
    int canBus;
    /// CAN ID of this specific device on its bus
    int canID;

    // Syntax: a struct is a class whose members are public by default, used here as a plain data record
    /// Status frame "periodic 1" sent by the SparkMax: motor velocity and electrical readings
    struct PeriodicUpdateData1 {
        float velocity = 0;  // Motor shaft velocity in RPM (before any gearbox)
        uint8_t temperature = 0;  // Motor temperature in degrees C
        uint16_t voltage = 0;  // Raw 12-bit bus voltage reading
        uint16_t current = 0;  // Raw 12-bit output current reading
    };

    /// Status frame carrying a position reading (used for both periodic frame 2 and periodic frame 5)
    struct PeriodicUpdateData2 {
        float position = 0;  // Position in revolutions from zero
    };

    PeriodicUpdateData1 lastPeriodicData1;  // Latest velocity frame for this device
    PeriodicUpdateData2 lastPeriodicData2;  // Latest relative (built-in) encoder position
    PeriodicUpdateData2 lastPeriodicData5;  // Latest absolute encoder position

    /// Everything needed to talk to one CAN bus through a Linux SocketCAN socket
    struct CANStaticData {
        ifreq ifr;  // Interface request used to look up "can0"/"can1" by name
        sockaddr_can socketAddress;  // Address the socket is bound to
        int soc = -1;  // Socket file descriptor (-1 means no socket is open)

        bool canBussesSetup = false;  // True once the socket is open and bound
        bool setupAttempted = false;  // True after the first setup attempt, successful or not

        /// Maps each CAN ID on this bus to the CANDriver object that represents it
        std::map<int, CANDriver*> canIDMap;
    };

    // Syntax: plain_guarded<T> (from cs_libguarded) wraps T in a mutex so two threads can't touch T at once
    // Syntax: .lock() returns a handle to T that unlocks the mutex automatically when it goes out of scope
    typedef libguarded::plain_guarded<CANStaticData> CANStaticDataGuarded;

    // Syntax: static members belong to the class itself, not to one object, so every CANDriver shares these
    // Their storage is defined at the top of CANDriver.cpp

    /// Shared data about each CAN bus (array index is the bus number)
    static std::array<CANStaticDataGuarded, 2> canStaticData;
    /// How many CANDriver objects currently use each bus (the socket closes when this reaches 0)
    static std::array<libguarded::plain_guarded<size_t>, 2> canStaticDataUsers;

    /**
     * Perform one-time setup for one of the CAN buses
     *
     * Parameters (inputs):
     *   canBus - the bus to set up (0 or 1)
     *
     * Return value:
     *   true if the bus is open and usable, false if it is not (e.g. not running on the rover)
     */
    static bool setupCAN(int canBus);

    /**
     * Deallocate resources associated with the given CAN bus
     *
     * Parameters (inputs):
     *   canBus - the bus to clean up (0 or 1)
     */
    static void closeCAN(int canBus);

    /// Unused background reader thread (reads currently happen in doCanReadIter from the control loop)
    static std::thread canReadThread;

    /**
     * Send one frame on a CAN bus
     *
     * Parameters (inputs):
     *   canBus - the bus to send on
     *   frame - the CAN frame to send (its ID is marked as an extended 29-bit ID before sending)
     *
     * Return value:
     *   true if the frame was written, false if the bus is unavailable or the write failed
     */
    static bool sendMSG(int canBus, can_frame frame);

    /**
     * Read one frame from a CAN bus without blocking
     *
     * Parameters (inputs):
     *   canBus - the bus to read from
     *   frame - filled in with the received frame (output)
     *
     * Return value:
     *   true if a frame was read, false if none was waiting or the bus is unavailable
     */
    static bool receiveMSG(int canBus, can_frame& frame);

    /// Background-thread read loop (currently unused, see canReadThread)
    static void startCanReadThread(int canBus);

    /// Decode a "periodic 1" velocity frame and store it on the matching device
    static void parsePeriodicData1(int canBus, can_frame frame);
    /// Decode a "periodic 2" relative-position frame and store it on the matching device
    static void parsePeriodicData2(int canBus, can_frame frame);
    /// Decode a "periodic 5" absolute-position frame and store it on the matching device
    static void parsePeriodicData5(int canBus, can_frame frame);

public:
    /**
     * Perform one iteration of reading received messages off a CAN bus
     *
     * Parameters (inputs):
     *   canBus - the bus to read from
     *
     * Return value:
     *   true if a message was read (call again to drain the queue), false if nothing was waiting
     */
    static bool doCanReadIter(int canBus);

    /**
     * Whether a CAN bus was opened successfully
     *
     * Parameters (inputs):
     *   canBus - the bus to check (0 or 1)
     *
     * Return value:
     *   true on the rover with the CAN HAT connected, false when running without hardware
     */
    static bool isBusAvailable(int canBus);

    /**
     * Create a new CAN device
     *
     * Parameters (inputs):
     *   busNum - the CAN bus it is connected to (0 or 1)
     *   canID - the CAN ID of the physical device this object represents
     */
    CANDriver(int busNum, int canID);

    // Syntax: a copy constructor runs when an object is copied (std::vector copies motors as it grows)
    CANDriver(const CANDriver& other);

    // Syntax: "= delete" forbids assigning one CANDriver to another, since that would confuse the ID map
    CANDriver& operator=(const CANDriver& other) = delete;

    // Syntax: a virtual destructor makes sure the right destructor runs when deleting through a base pointer
    virtual ~CANDriver();
};

/**
 * A REV Robotics SparkMax brushed/brushless DC motor controller on the CAN bus
 *
 * The documentation for the SparkMax CAN protocol can be found
 * [here](https://docs.google.com/spreadsheets/d/1SD-d_iXorli3zYffGwU5WK28JmIU9irfgiajrSXJ930/edit?usp=sharing).
 * The protocol version we have access to only works for SparkMax firmware older than 2025.X.X.
 *
 * Syntax: "class SparkMax : CANDriver" inherits privately (the default for class)
 * Private inheritance means CANDriver's public methods are usable inside SparkMax but not by code holding a SparkMax
 *
 * @todo The PID controller is used for position; there should be a way to configure it for velocity or position
 */
class SparkMax : CANDriver {
protected:
    double pidSetpoint = 0;  // Target for the PID controller (radians, since the PID runs on position)

    double lastVel = 0;  // Unused

    double gearRatio = 1;  // Motor revolutions per output revolution (greater than 1 for a reduction)
    bool useAbsolute = false;  // Read position from the absolute encoder instead of the built-in one

    /// Build pidController from the owning node's kp/ki/kd/max_i/readOnly ROS parameters
    void setupPID();

    rclcpp::Node::SharedPtr node_ = nullptr;  // Owning ROS node, used to read the PID parameters

public:
    /**
     * Create a SparkMax on a CAN bus
     *
     * Parameters (inputs):
     *   node - the ROS node that owns this motor (it must declare kp, ki, kd, max_i, and readOnly)
     *   canBUS - the CAN bus the SparkMax is connected to (0 or 1)
     *   canID - the CAN ID set on the SparkMax (configured with REV's Hardware Client tool)
     *   gearRatio - motor revolutions per output revolution (greater than 1 if speed drops over the gearbox)
     *   useAbsolute - use an absolute encoder plugged into the SparkMax instead of the built-in motor encoder
     */
    SparkMax(rclcpp::Node::SharedPtr node, int canBUS, int canID, double gearRatio, bool useAbsolute);

    SparkMax(const SparkMax& other);

    /**
     * Send the periodic heartbeat message to the SparkMax
     *
     * SparkMax controllers refuse to move their motor unless a heartbeat arrives within a timeout period.
     * This comes from FRC, where it stops robots if the software crashes or the match isn't running.
     *
     * Return value:
     *   true if the message was sent
     */
    bool sendHeartbeat();

    /**
     * Set the update rate of the status frame carrying the absolute encoder reading
     *
     * Return value:
     *   true if the message was sent
     */
    bool sendAbsoluteFrameUpdateRate();

    /**
     * Command a power level to the motor
     *
     * Parameters (inputs):
     *   power - a normalized value in [-1, 1] where 1 is full power forwards (clamped to MAX_DRIVE_POWER)
     */
    void sendPowerCMD(float power);

    /**
     * Update the PID set point
     *
     * Parameters (inputs):
     *   pidSetpoint - target position in radians
     */
    void setPIDSetpoint(double pidSetpoint);

    /// Latest velocity at the output shaft in rad/s (motor RPM divided by the gear ratio)
    double lastVelocityAsRadPerSec();
    /// Latest built-in encoder position at the output shaft, in radians
    double lastPositionInRad();
    /// Latest absolute encoder position in radians, wrapped into (-pi, pi]
    double lastAbsPositionInRad();
    /// Latest position from whichever encoder this motor is configured to use
    double lastCorrectPos();

    bool pidControlled = true;  // Run the PID in pidTick (MotorManager sets this from its usePid flag)
    bool viewOnly = false;  // Compute PID output but never send power (the readOnly parameter)

    bool motorLocked = false;  // Set on loss of signal to stop PID output until a new command arrives

    /**
     * Run one step of the PID controller and send the result as a power command
     *
     * Should be called at a constant rate, since the PID's time step is fixed (see setupPID)
     *
     * Parameters (inputs):
     *   currentPos - unused (the position is read from the encoder data directly)
     */
    void pidTick(double currentPos);

    // Placeholder PID; setupPID() replaces it with one built from the node's parameters
    PID pidController = PID(1, 0, 0, 0, 0, 0, 1);

    /// Send an ident message, which makes the SparkMax's status light blink rapidly (useful to find a motor)
    void ident();
};

/**
 * A SparkMax driven by PWM from a Raspberry Pi GPIO pin (through pigpio) instead of by CAN
 *
 * Only works on a Pi with the pigpio library able to access the GPIO hardware
 */
class PWMSparkMax {
protected:
    int pin;  // Broadcom (BCM) GPIO pin number the PWM signal is sent on
    static bool gpioSetup;  // True once pigpio has been initialized for the process
    /// Initialize pigpio once (throws if it can't, e.g. when not running on a Pi)
    static void setupGPIO();
    float lastSentValue = -10;  // Last power sent, so repeated identical commands are skipped
    bool initialSetup = false;  // True once the PWM frequency has been set on this pin

public:
    /// Release the pigpio library (call once at shutdown)
    static void terminateGPIO();

    /**
     * Create a PWM-driven SparkMax and set it to zero power
     *
     * Parameters (inputs):
     *   pin - BCM GPIO pin number wired to the SparkMax's PWM input
     */
    PWMSparkMax(int pin);
    ~PWMSparkMax();

    /**
     * Send a power level as a PWM pulse width
     *
     * Parameters (inputs):
     *   power - a value in [-1, 1], clamped to [-0.8, 0.8]
     */
    void setPower(float power);
};
