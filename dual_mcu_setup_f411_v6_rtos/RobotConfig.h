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
// const float KP_VEL = 0.05f;
// const float KI_VEL = 0.00f;
// const float KD_VEL = 0.0f;



// PID gains fpr gyro_rate compensation
const float KP_VEL = 0.05f;
const float KI_VEL = 0.00f;
const float KD_VEL = 0.0f;

const float KP_OMEGA = 0.5;
const float KI_OMEGA = 0;
const float KD_OMEGA = 0;
const float KP_HEADING = 0.2f;


const float MAX_OMEGA_RADPS = 5.0f;  

// Feedforward lookup table (voltage vs average speed) - from ground on both motors
const int FEEDFORWARD_POINTS = 8;
const float feedForward_speed[] = {19.0f, 32.34f, 48.33f, 63.34f, 79.0f, 94.0f, 109.0f, 123.34f}; //ticks/sec
const float feedForward_voltage[] = {2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f}; //voltage to get the corresponding tick rotation


float getFeedforwardVoltage(float desiredSpeed_ticks) {
    if (desiredSpeed_ticks <= feedForward_speed[0]) return feedForward_voltage[0];

    if (desiredSpeed_ticks >= feedForward_speed[FEEDFORWARD_POINTS-1]) return feedForward_voltage[FEEDFORWARD_POINTS-1];

    for (int i = 0; i < FEEDFORWARD_POINTS-1; i++) {
        if (desiredSpeed_ticks >= feedForward_speed[i] && desiredSpeed_ticks <= feedForward_speed[i+1]) {
            float t = (desiredSpeed_ticks - feedForward_speed[i]) / (feedForward_speed[i+1] - feedForward_speed[i]);
            return feedForward_voltage[i] + t * (feedForward_voltage[i+1] - feedForward_voltage[i]);
        }
    }
    return 0;
}