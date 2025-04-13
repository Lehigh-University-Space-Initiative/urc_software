#ifndef CANDriver_FILE
#define CANDriver_FILE

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
#include <vector>
#include <pigpio.h>
#include "cs_libguarded/cs_plain_guarded.h"
#include "pid.h"
#include "Limits.h"


/**
 * A device which can be communicated with over a CAN bus.
 * 
 * There are assumed to be only 2 can busses since the code is designed for the waveshare 2 channel raspberry pi CAN hat
 * 
 * @todo this class should not deal with SparkMax spacific concepts like periodic updates
*/
class CANDriver {
protected:
    /// @brief can bus this device is connected to
    /// Valid values are 0 and 1
    int canBus;
    /// @brief can id of this specific device
    int canID;

    // struct PeriodicUpdateData {
    //     std::atomic<float> velocity = 0;
    //     std::atomic<uint8_t> temperature = 0;
    //     std::atomic<uint16_t> voltage = 0;
    //     std::atomic<uint16_t> current = 0;
    // };

    struct PeriodicUpdateData1 {
        float velocity = 0;
        uint8_t temperature = 0;
        uint16_t voltage = 0;
        uint16_t current = 0;
    };
    struct PeriodicUpdateData2 {
        float position = 0;
    };

    // static std::mutex lastPeriodicMutex;
    PeriodicUpdateData1 lastPeriodicData1;
    PeriodicUpdateData2 lastPeriodicData2;
    PeriodicUpdateData2 lastPeriodicData5;


    struct CANStaticData {
        ifreq ifr;
        sockaddr_can socketAddress;
        int soc;

        bool canBussesSetup = 0;

        //map each can id to a CANDriver pointer
        std::map<int, CANDriver*> canIDMap;
    };

    typedef libguarded::plain_guarded<CANStaticData> CANStaticDataGuarded;

    /// @brief stored data about each can bus that is setup
    /// The array index here is the can id
    static std::array<CANStaticDataGuarded, 2> canStaticData;
    static std::array<libguarded::plain_guarded<size_t>, 2> canStaticDataUsers;

    /// @brief Perform one time setup for one of the can buses
    /// @param canBus the can bus to setup
    /// @return if the setup was successful 
    static bool setupCAN(int canBus);
    /// @brief Deallocate resources associated with the given can bus
    /// @param canBus the can bus to cleanup
    static void closeCAN(int canBus);

    static std::thread canReadThread;
    /// @brief Send a message on a can bus
    /// @param canBus the can bus to send the message on
    /// @param frame the CAN message to send
    /// @return if the message was sent
    static bool sendMSG(int canBus, can_frame frame);
    static bool receiveMSG(int canBus, can_frame& frame);
    static void startCanReadThread(int canBus);
    static void parsePeriodicData1(int canBus, can_frame frame);
    static void parsePeriodicData2(int canBus, can_frame frame);
    static void parsePeriodicData5(int canBus, can_frame frame);

public:

    /// @brief perform one iteration of reading received messages off of a can bus
    /// @param canBus the can bus to read messages from
    /// @return if there was a message to read
    static bool doCanReadIter(int canBus);

    /// @brief Create a new can driver device
    /// @param busNum the can bus it is connected to
    /// @param canID the can id of the physical device on the can bus that this object will represent.
    CANDriver(int busNum, int canID);

    CANDriver(const CANDriver& other);

    CANDriver& operator=(const CANDriver& other) = delete;
    virtual ~CANDriver();
};

/**
 * An object representing a Rev Robotics Spark Max Brushes DC Motor Controller.
 * 
 * The documentation for the SparkMax CAN protocol can be found [here](https://docs.google.com/spreadsheets/d/1SD-d_iXorli3zYffGwU5WK28JmIU9irfgiajrSXJ930/edit?usp=sharing).
 * However, the protocol version we currently have access to only works for SparkMax firmware versions prior to 2025.X.X.
 * 
 * @todo The pid controller might now be position. There should be a way to configure the PID to be velocity or position
*/
class SparkMax : CANDriver {
protected:
    // velocity in rad / s
    double pidSetpoint = 0;// 3.141592 * 0.5;


    double lastVel = 0;

    double gearRatio = 1;
    bool useAbsolute = false;

    void setupPID();

    rclcpp::Node::SharedPtr node_ = nullptr;

public:
    /// @brief 
    /// @param node The ros node object owning this spark max (used for reading parameters)
    /// @param canBUS The can bus this spark max is connect to
    /// @param canID The can id of this spark max on the can bus
    /// @param gearRatio The gear ratio between the motor connected to the spark max and whatever it is connected to. 
    /// This is greater than 1 if rotation speed decreases over the gearbox.
    /// @param useAbsolute Whether to use an absolute position encoder connected to the spark max or the default internal brushes motor position encoder
    SparkMax(rclcpp::Node::SharedPtr node, int canBUS, int canID, double gearRatio, bool useAbsolute);

    SparkMax(const SparkMax& other);

    /// @brief send the periodic heartbeat message to the SparkMax
    /// The spark max motor controllers will prevent any motors from moving unless this heartbeat message is sent within a given period.
    /// If a heartbeat message is not received within that timeout period, the spark max will stop its motor. 
    /// This is from FRC where they want a way to stop the robots if there is a software problem or the round is not running.
    /// @return if the message was sent
    bool sendHeartbeat();

    /// @brief send a message to the spark max to set the update rate of the periodic update frame which contains the absolute encoder readout.
    /// @return if the message was sent
    bool sendAbsoluteFrameUpdateRate();
    /// @brief Command a power level to the motor
    /// @param power a normalized value in the range [-1,1] where 1 represents full power forwards
    void sendPowerCMD(float power);
    
    /// @brief update the PID set point for the PID controller
    /// @param pidSetpoint set point in rad/s
    void setPIDSetpoint(double pidSetpoint);

    double lastVelocityAsRadPerSec();
    double lastPositionInRad();
    double lastAbsPositionInRad();
    double lastCorrectPos();

    bool pidControlled = true;
    bool viewOnly = false;

    //when LOS happens disable motor
    bool motorLocked = false;

    //should be called every Dt
    void pidTick(double currentPos);
    // float dt = 0.01;

    PID pidController = PID(1,0,0,0,0,0,1); //PID(0.01,MAX_DRIVE_POWER,-MAX_DRIVE_POWER,0.06,0.001,0.05);
    //TODO fix for driveline
    //TODO Note: the DT is set as constant here not dynamic
    // PID pidController;// = PID(0.005,0.15,-0.15,0.3,0.01,0.2);

    /// @brief send an ident message to the spark max causing the status light to rapidly blink
    void ident();
};

class PWMSparkMax {
protected:
    int pin;
    static bool gpioSetup;
    static void setupGPIO();
    float lastSentValue = -10;
    bool initialSetup = false;

public:
    static void terminateGPIO();
    PWMSparkMax(int pin);
    ~PWMSparkMax();
    void setPower(float power);
};

#endif
