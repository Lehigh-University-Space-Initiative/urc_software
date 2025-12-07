#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <array>
#include <vector>
#include <chrono>
#include <pigpio.h>
#include "rclcpp/rclcpp.hpp"

class StepperDriver {
protected:
    int dirPin;
    int stepPin;
    int enablePin;

    float gearRatio;
    int stepsPerRev;

    bool initialSetup = false;

    // internal control methods
    void setupGPIO();
    void step();
    void runStepTick();

    // internal relative encoder
    float internalEncoder = 0;

    bool dir = false;

    // velocity in after gear rev/s
    float commandedVelocity = 0;

    // if a new command has not been sent after 0.5 seconds, set velocity to 0
    std::chrono::time_point<std::chrono::system_clock> lastTickTime;
    std::chrono::time_point<std::chrono::system_clock> lastStepTime;

    // time the last command was sent (used to determine if lost connection)
    std::chrono::time_point<std::chrono::system_clock> lastCommandTime;

    // enum for different control modes
    enum class ControlMode {
        VELOCITY,
        MANUAL
    };

    ControlMode controlMode = ControlMode::VELOCITY;

public:
    StepperDriver(int dirPin, int stepPin, int enablePin, float gearRatio, int stepsPerRev);
    virtual ~StepperDriver();

    // Enable/disable stepper
    void setEnable(bool enable);
    bool getEnable();

    // Set/Get direction
    void setDir(bool dir);
    bool getDir();

    // Set/Get velocity
    void setVelocity(float velocity);
    float getVelocity();

    // must be called at a constant rate of 500Hz (every 2ms)
    void tick();
};
