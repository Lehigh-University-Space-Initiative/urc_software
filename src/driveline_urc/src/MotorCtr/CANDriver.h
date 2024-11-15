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
#include "cs_plain_guarded.h"
#include "PID/pid.h"
#include "Limits.h"

class CANDriver {
protected:
    int canBus;
    int canID;

    struct PeriodicUpdateData {
        float velocity = 0;
        uint8_t temperature = 0;
        uint16_t voltage = 0;
        uint16_t current = 0;
    };

    libguarded::plain_guarded<PeriodicUpdateData> lastPeriodicData;


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
    static void parsePeriodicData(int canBus, can_frame frame);

public:
    CANDriver(int busNum, int canID);
    CANDriver(const CANDriver& other);
    CANDriver& operator=(const CANDriver& other);
    virtual ~CANDriver();
};

class SparkMax : CANDriver {
public:
    SparkMax(int canBUS, int canID);
    bool sendHeartbeat();
    void sendPowerCMD(float power);

    bool pidControlled = false;
    // velocity in ___ / ___
    double pidSetpoint = 0;
    //should be called every Dt
    void pidTick();
    // float dt = 0.01;

    PID pidController = PID(0.01,MAX_DRIVE_POWER,-MAX_DRIVE_POWER,0.1,0,0);

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
