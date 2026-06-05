// RobotConfig.h
#pragma once

#define WHEEL_DIAMETER 0.065
#define TICK_PER_REV 40.0

// Motor voltage limit (volts)
const float MAX_MOTOR_VOLTAGE = 9.0f;

// Battery nominal voltage (for development, will be measured later)
static float BATTERY_VOLTAGE = 0;

// Track width (meters)
const float ROBOT_TRACK_WIDTH = 0.13f;

// Encoder ticks per meter
#define TICKS_PER_METER ((TICK_PER_REV) / (PI * WHEEL_DIAMETER))
// max_radps at 9v max. at 9v max tick is 97
#define MAX_OMEGA_RADPS  ((2 * (97 / TICKS_PER_METER)) / ROBOT_TRACK_WIDTH)

// Best tunnign parameters so far
// // Control loop period (seconds)
float CONTROL_DT = 0.05f; // set in motioncontrollertask

 // PID gains for velocity compensation
const float KP_VEL = 0.15f;
const float KI_VEL = 0.025f;
const float KD_VEL = 0.0f;

const float KP_HEADING = 3.5f;

// PID gains for gyrorate heading compensation
const float KP_OMEGA = 1.95f; //started with 5 then 2, 2.7
const float KI_OMEGA = 0.09f; // increase a bit later, started with 0.1
const float KD_OMEGA = 0.00f;


 // PID gains for velocity compensation
// const float KP_VEL = 0.1f;
// const float KI_VEL = 0.5f;
// const float KD_VEL = 0.0f;

// const float KP_HEADING = 2.0f;
// const float KI_HEADING = 0.15f;     // start small (was 0)

// // PID gains for gyrorate heading compensation
// const float KP_OMEGA = 2.0; //started with 5 then 2
// const float KI_OMEGA = 0.5; // increase a bit later
// const float KD_OMEGA = 0;

