#include "StepperDriver.h"
#include "rclcpp/rclcpp.hpp"
#include "iostream"

void StepperDriver::setupGPIO() {
    // if the GPIO has already been set up, return
    if (initialSetup) {
        return;
    }

    // try to initialize GPIO
    if (gpioInitialise() < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to initialize GPIO");
        return;
    }

    // set the pins to output
    gpioSetMode(dirPin, PI_OUTPUT);
    gpioSetMode(stepPin, PI_OUTPUT);
    gpioSetMode(enablePin, PI_OUTPUT);

    // set the pins to high
    gpioWrite(dirPin, PI_HIGH);
    gpioWrite(stepPin, PI_HIGH);
    gpioWrite(enablePin, PI_HIGH);

    // set the initial setup flag
    initialSetup = true;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GPIO initialized successfully");
}

void StepperDriver::step() {
    // set the step pin low
    gpioWrite(stepPin, PI_LOW);

    // wait 1 millisecond (1000 microseconds)
    gpioDelay(1000);

    // set the step pin high
    gpioWrite(stepPin, PI_HIGH);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stepped for step pin: %d", stepPin);
}

void StepperDriver::runStepTick() {
    // get the current time
    auto currentTime = std::chrono::system_clock::now();

    // get the time since the last step
    std::chrono::duration<float> timeSinceLastStep = currentTime - lastStepTime;

    // convert from ms to seconds
    float timeDelta = timeSinceLastStep.count();

    // determine if a step should be taken
    float timeBetweenSteps = 1.0f / (abs(commandedVelocity) * stepsPerRev / gearRatio);

    if (timeDelta > timeBetweenSteps) {
        // take a step
        step();

        // update the last step time
        lastStepTime = currentTime;
    }
}

StepperDriver::StepperDriver(int dirPin, int stepPin, int enablePin, float gearRatio, int stepsPerRev)
    : dirPin(dirPin), stepPin(stepPin), enablePin(enablePin), gearRatio(gearRatio), stepsPerRev(stepsPerRev) {
    setupGPIO();
    auto now = std::chrono::system_clock::now();
    lastCommandTime = now;
    lastStepTime = now;
    lastTickTime = now;
}

StepperDriver::~StepperDriver() {
    // set dir and step pins to low
    gpioWrite(dirPin, PI_LOW);
    gpioWrite(stepPin, PI_LOW);
}

void StepperDriver::setEnable(bool enable) {
    gpioWrite(enablePin, enable ? PI_HIGH : PI_LOW);
}

bool StepperDriver::getEnable() {
    return gpioRead(enablePin) == PI_HIGH;
}

void StepperDriver::setDir(bool dir) {
    if (dir == this->dir) {
        return;
    }
    gpioWrite(dirPin, dir ? PI_HIGH : PI_LOW);
    this->dir = dir;
}

bool StepperDriver::getDir() {
    return dir;
}

void StepperDriver::setVelocity(float velocity) {
    float maxVelocity = 1.0f;

    // clamp velocity to [-maxVelocity, maxVelocity]
    if (velocity > maxVelocity) {
        velocity = maxVelocity;
    } else if (velocity < -maxVelocity) {
        velocity = -maxVelocity;
    }

    commandedVelocity = velocity;

    // set direction based on velocity
    if (commandedVelocity > 0) {
        setDir(true);
    } else if (commandedVelocity < 0) {
        setDir(false);
    }
}

float StepperDriver::getVelocity() {
    return commandedVelocity;
}

void StepperDriver::tick() {
    auto now = std::chrono::system_clock::now();

    // reset velocity if no command was sent for 0.5 seconds
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastCommandTime).count() > 500) {
        // RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Velocity reset to zero due to timeout");
        commandedVelocity = 0;
    }

    runStepTick();

    // update last tick time
    lastTickTime = now;
}
