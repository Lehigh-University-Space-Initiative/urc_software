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

class CANDriver {
protected:
    int canBus;

    struct CANStaticData {
        ifreq ifr;
        sockaddr_can socketAddress;
        int soc;

        bool canBussesSetup;
    };

    typedef libguarded::plain_guarded<CANStaticData> CANStaticDataGuarded;

    static std::array<CANStaticDataGuarded, 2> canStaticData;
    static std::array<libguarded::plain_guarded<size_t>, 2> canStaticDataUsers;

    static bool setupCAN(int canBus);
    static void closeCAN(int canBus);

    static bool sendMSG(int canBus, can_frame frame);

public:
    CANDriver(int busNum);
    virtual ~CANDriver();
};

class SparkMax : CANDriver {
protected:
    int canID;
public:
    SparkMax(int canBUS, int canID);
    bool sendHeartbeat();
    void sendPowerCMD(float power);
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
