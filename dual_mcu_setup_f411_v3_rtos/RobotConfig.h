// RobotConfig.h
#pragma once

#define WHEEL_DIAMETER 0.065
#define TICK_PER_REV 40.0

// Motor voltage limit (volts)
const float MAX_MOTOR_VOLTAGE = 9.0f;

// Battery nominal voltage (for development, will be measured later)
const float batteryVoltage = 12.6f;

// Track width (meters)
const float ROBOT_TRACK_WIDTH = 0.13f;

// Encoder ticks per meter (adjust to your robot)

#define TICKS_PER_METER (TICK_PER_REV) / (PI * WHEEL_DIAMETER);


// Best tunnign parameters so far
// // Control loop period (seconds)
const float CONTROL_DT = 0.02f;  // 20 ms

// // PID gains (example values – tune these!)
const float KP_VEL = 0.5f;
const float KI_VEL = 0.04f;
const float KD_VEL = 0.0f;
const float KP_HEADING = 0.56f;


// Control loop period (seconds)
// const float CONTROL_DT = 0.02f;  // 20 ms

// // PID gains (example values – tune these!)
// const float KP_VEL = 0.3f;
// const float KI_VEL = 0.005f;
// const float KD_VEL = 0.0f;
// const float KP_HEADING = 0f;