// RobotConfig.h
#pragma once

#define WHEEL_DIAMETER 0.065
#define TICK_PER_REV 40.0

// Motor voltage limit (volts)
const float MAX_MOTOR_VOLTAGE = 9.0f;

// Battery nominal voltage (for development, will be measured later)
volatile float BATTERY_VOLTAGE = 0;

// Track width (meters)
const float ROBOT_TRACK_WIDTH = 0.13f;

// Encoder ticks per meter (adjust to your robot)

#define TICKS_PER_METER (TICK_PER_REV) / (PI * WHEEL_DIAMETER);


// Best tunnign parameters so far
// // Control loop period (seconds)
const float CONTROL_DT = 0.02f;  // 20 ms

// // PID gains (example values – tune these!)
const float KP_VEL = 0.15f;
const float KI_VEL = 0.00f;
const float KD_VEL = 0.0f;
const float KP_HEADING = 0.0f;

// Best tunning parameters so far
// // Control loop period (seconds)
// const float CONTROL_DT = 0.02f;  // 20 ms

// // // PID gains (example values – tune these!)
// const float KP_VEL = 0.1f;
// const float KI_VEL = 0.01f;
// const float KD_VEL = 0.0f;
// const float KP_HEADING = 0.2f;

// // Angular velocity PID gains (inner loop)
// const float KP_OMEGA = 1.0f;
// const float KI_OMEGA = 0.0f;
// const float KD_OMEGA = 0.0f;

// // Maximum angular velocity (rad/s) – adjust based on your robot's capability
// const float MAX_OMEGA_RADPS = 5.0f;



// Control loop period (seconds)
// const float CONTROL_DT = 0.02f;  // 20 ms

// // PID gains (example values – tune these!)
// const float KP_VEL = 0.3f;
// const float KI_VEL = 0.005f;
// const float KD_VEL = 0.0f;
// const float KP_HEADING = 0f;