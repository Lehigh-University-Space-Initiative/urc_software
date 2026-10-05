/**
 * @file
 * Motor power limits applied to every SparkMax power command (see SparkMax::sendPowerCMD)
 */
#pragma once

// Syntax: #define makes a text substitution done by the preprocessor before compiling (no type, no scope)

#define MAX_DRIVE_POWER 1.0f  // Largest power magnitude sent to a motor (1.0 = 100%)
#define DRIVE_DEADBAND 0.0f  // Powers smaller than this are sent as 0 (0.0 disables the deadband)
