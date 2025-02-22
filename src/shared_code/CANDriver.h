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

class CANDriver {
protected:
    int canBus;
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


    struct CANStaticData {
        ifreq ifr;
        sockaddr_can socketAddress;
        int soc;

        bool canBussesSetup = 0;

        //map each can id to a CANDriver pointer
        std::map<int, CANDriver*> canIDMap;
    };

    typedef libguarded::plain_guarded<CANStaticData> CANStaticDataGuarded;

    static std::array<CANStaticDataGuarded, 2> canStaticData;
    static std::array<libguarded::plain_guarded<size_t>, 2> canStaticDataUsers;

    static bool setupCAN(int canBus);
    static void closeCAN(int canBus);

    static std::thread canReadThread;
    static bool sendMSG(int canBus, can_frame frame);
    static bool receiveMSG(int canBus, can_frame& frame);
    static void startCanReadThread(int canBus);
    static void parsePeriodicData1(int canBus, can_frame frame);
    static void parsePeriodicData2(int canBus, can_frame frame);

public:

    static void doCanReadIter(int canBus);

    CANDriver(int busNum, int canID);
    CANDriver(const CANDriver& other);
    // CANDriver(const CANDriver&& other);
    CANDriver& operator=(const CANDriver& other) = delete;
    virtual ~CANDriver();
};

class SparkMax : CANDriver {
protected:
    // velocity in rad / s
    double pidSetpoint = 0;// 3.141592 * 0.5;


    double lastVel = 0;

    double gearRatio = 1;

    void setupPID();

    rclcpp::Node::SharedPtr node_ = nullptr;

public:
    SparkMax(rclcpp::Node::SharedPtr node, int canBUS, int canID, double gearRatio);
    SparkMax(const SparkMax& other);
    bool sendHeartbeat();
    void sendPowerCMD(float power);
    // in rad/s
    void setPIDSetpoint(double pidSetpoint);

    double lastVelocityAsRadPerSec();
    double lastPositionInRad();

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
