#pragma once

#include "PID.h"
#include <cmath>




// Feedforward lookup table (voltage vs average speed) - from ground on both motors
float getFeedforwardVoltage(float desiredSpeed_ticks) {
    static const int POINTS = 8;
    // Average of left and right speeds from the two test runs
    static const float speed_avg[] = {7.475f, 28.5f, 41.75f, 55.0f, 67.75f, 77.5f, 88.75f, 97.0f};
    static const float volt[]      = {2.0f,   3.0f,  4.0f,   5.0f,  6.0f,   7.0f,  8.0f,   9.0f};

    if (desiredSpeed_ticks <= speed_avg[0]) return volt[0];
    if (desiredSpeed_ticks >= speed_avg[POINTS-1]) return volt[POINTS-1];

    for (int i = 0; i < POINTS-1; i++) {
        if (desiredSpeed_ticks >= speed_avg[i] && desiredSpeed_ticks <= speed_avg[i+1]) {
            float t = (desiredSpeed_ticks - speed_avg[i]) / (speed_avg[i+1] - speed_avg[i]);
            return volt[i] + t * (volt[i+1] - volt[i]);
        }
    }
    return 0.0f;
}

float getFeedforwardVoltageLeft(float desiredSpeed_ticks) {
    static const int POINTS = 8;
    static const float speed[] = {7.35f, 28.00f, 41.50f, 54.50f, 67.50f, 76.50f, 88.50f, 97.00f};
    static const float volt[]  = {2.0f,  3.0f,   4.0f,   5.0f,   6.0f,   7.0f,   8.0f,   9.0f};

    if (desiredSpeed_ticks <= speed[0]) return volt[0];
    if (desiredSpeed_ticks >= speed[POINTS-1]) return volt[POINTS-1];

    for (int i = 0; i < POINTS-1; i++) {
        if (desiredSpeed_ticks >= speed[i] && desiredSpeed_ticks <= speed[i+1]) {
            float t = (desiredSpeed_ticks - speed[i]) / (speed[i+1] - speed[i]);
            return volt[i] + t * (volt[i+1] - volt[i]);
        }
    }
    return 0.0f;
}

float getFeedforwardVoltageRight(float desiredSpeed_ticks) {
    static const int POINTS = 8;
    static const float speed[] = {7.60f, 29.00f, 42.00f, 55.50f, 68.00f, 78.50f, 89.00f, 97.00f};
    static const float volt[]  = {2.0f,  3.0f,   4.0f,   5.0f,   6.0f,   7.0f,   8.0f,   9.0f};

    if (desiredSpeed_ticks <= speed[0]) return volt[0];
    if (desiredSpeed_ticks >= speed[POINTS-1]) return volt[POINTS-1];

    for (int i = 0; i < POINTS-1; i++) {
        if (desiredSpeed_ticks >= speed[i] && desiredSpeed_ticks <= speed[i+1]) {
            float t = (desiredSpeed_ticks - speed[i]) / (speed[i+1] - speed[i]);
            return volt[i] + t * (volt[i+1] - volt[i]);
        }
    }
    return 0.0f;
}


// Feedforward lookup table for angular velocity (voltage vs average yawRate) - from ground on both motors
float getAngularFeedforwardVoltageDiff(float desiredOmega_degps) {
    static const int POINTS = 8;
    static const float omega_deg[] = {11.265f, 23.26f, 36.10f, 46.55f, 58.55f, 62.955f, 69.355f, 72.02f};
    static const float vdiff[]    = {4.0f,    6.0f,   8.0f,   10.0f,  12.0f,  14.0f,   16.0f,   18.0f};

    float absOmega = fabs(desiredOmega_degps);
    float sign = (desiredOmega_degps >= 0) ? 1.0f : -1.0f;

    if (absOmega <= omega_deg[0]) return 0.0;

    if (absOmega >= omega_deg[POINTS-1]) return sign * vdiff[POINTS-1];

    for (int i = 0; i < POINTS-1; i++) {
        if (absOmega >= omega_deg[i] && absOmega <= omega_deg[i+1]) {
            float t = (absOmega - omega_deg[i]) / (omega_deg[i+1] - omega_deg[i]);
            float v = vdiff[i] + t * (vdiff[i+1] - vdiff[i]);
            return sign * v;
        }
    }
    return 0.0f;
}

class MotionController {
public:
    MotionController()
        : _pidLeft(KP_VEL, KI_VEL, KD_VEL, CONTROL_DT, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE),
        _pidRight(KP_VEL, KI_VEL, KD_VEL, CONTROL_DT, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE),
        _pidOmega(KP_OMEGA, KI_OMEGA, KD_OMEGA, CONTROL_DT, -MAX_OMEGA_RADPS, MAX_OMEGA_RADPS),
        prevLeft(0.0), prevRight(0.0) {}

    // Set target velocities (linear m/s, angular rad/s)
    void setTargetVelocity(float v, float omega) {
        _targetV = v;
        _targetOmega = omega;
        _straightMode = false;   // disable heading correction
    }

    // Go straight at given speed (m/s), using current heading as reference
    void setStraight(float speed) {
        _targetV = speed;
        _targetOmega = 0.0f;
        _straightMode = true;
        _headingSetpointValid = false;   // will capture on next update
    }

    // Main control update, to be called at fixed interval (dt seconds)
    void update(float leftTicksAvg, float rightTicksAvg, float yaw, float yawRate_radps, float dt, float *motorsVoltage) {
        // 1. Compute wheel speeds (ticks/s)
        Serial.printf("prevleft tick is %f\n", prevLeft);
        Serial.printf("prev right tick is %f\n", prevRight);   

        Serial.printf("absolute yaw is %.5f\n", yaw);
        Serial.printf("yawRate is %.5f\n", yawRate_radps);   

        float leftTickSpeed = (leftTicksAvg - prevLeft) / dt;
        float rightTickSpeed = (rightTicksAvg - prevRight) / dt;

        float leftTickSpeedF = 0.0f, rightTickSpeedF = 0.0f;
        const int FILTER_WINDOW_SIZE = 3;
        static float leftTickSpeedBuffer[FILTER_WINDOW_SIZE] = {0};
        static float rightTickSpeedBuffer[FILTER_WINDOW_SIZE] = {0};

        static uint8_t idx = 0;
        leftTickSpeedBuffer[idx] = leftTickSpeed;
        rightTickSpeedBuffer[idx] = rightTickSpeed;
        idx = (idx + 1) % FILTER_WINDOW_SIZE;
    

        for(uint8_t i = 0; i < FILTER_WINDOW_SIZE; i++)
        {
            leftTickSpeedF += leftTickSpeedBuffer[i];
            rightTickSpeedF += rightTickSpeedBuffer[i];
        }

        leftTickSpeedF /= FILTER_WINDOW_SIZE;
        rightTickSpeedF /=FILTER_WINDOW_SIZE;
        Serial.printf("leftTickSpeedF is %.f\nrightTickSpeedF is %.f\n", leftTickSpeedF, rightTickSpeedF); 
        prevLeft = leftTicksAvg;
        prevRight = rightTicksAvg;
        
        // Outer heading loop (produces desired yaw rate)
        float desiredOmega = 0.0f;
        // 2. Heading correction if in straight mode
        if (_straightMode) {
            if (!_headingSetpointValid) {
                _headingSetpoint = yaw;
                _headingSetpointValid = true; 
            }
            float headingError = yaw - _headingSetpoint;
            Serial.printf("set point is %.5f\n", _headingSetpoint);
            // Normalize to [-π, π] (assuming yaw in radians)
            headingError = atan2f(sinf(headingError), cosf(headingError));
            Serial.printf("MAX_OMEGA_RADPS is %.5f\n", MAX_OMEGA_RADPS);
            // Adjust angular velocity command
            desiredOmega = KP_HEADING * headingError;
            Serial.printf("heading error %.5f\n", headingError);            
        }

        float desiredOmega_dps = -(desiredOmega * RAD_TO_DEG);   // to deg/s
        Serial.printf("desired correction Omega in degs is %.3f\n", desiredOmega_dps);
        float ff_angular_volt_diff = getAngularFeedforwardVoltageDiff(desiredOmega_dps); 
        Serial.printf("ffOmegaCorrection is %.2f\n", ff_angular_volt_diff);
        int8_t sign = (ff_angular_volt_diff >= 0) ? 1 : -1;

        // Split voltage difference and add to motor commands (direct voltage feedforward)
        float left_angular_ff = sign * fabs(ff_angular_volt_diff) / 2.0f;
        float right_angular_ff = -sign * fabs(ff_angular_volt_diff) / 2.0f;
        Serial.printf("angular_ff voltage correction for left motor is %.3f\n", left_angular_ff);
        Serial.printf("angular_ff voltage correction for right motor is %.3f\n", right_angular_ff);
        // Inner angular velocity loop (PID on yaw rate)
        float omegaCorrection = _pidOmega.compute(desiredOmega, yawRate_radps);
        omegaCorrection= constrain(omegaCorrection, -MAX_OMEGA_RADPS,  MAX_OMEGA_RADPS); 
        Serial.printf("omega correction is %.5f\n", omegaCorrection);
        // 3. Desired side speeds from kinematics (m/s)
        float leftDesired_mps = _targetV - omegaCorrection * ROBOT_TRACK_WIDTH / 2.0f;
        float rightDesired_mps = _targetV + omegaCorrection * ROBOT_TRACK_WIDTH / 2.0f;
        Serial.printf("leftDesired_mps is %f\n", leftDesired_mps);
        Serial.printf("rightDesired_mps is %f\n", rightDesired_mps);

        // 4. Convert to desired ticks/s using encoder resolution
        float leftDesired = leftDesired_mps * TICKS_PER_METER;
        float rightDesired = rightDesired_mps * TICKS_PER_METER;



        Serial.printf("current left speed(ticks/sec) is %f\n", leftTickSpeed);
        Serial.printf("current right speed(ticks/sec) is %f\n", rightTickSpeed);
        Serial.printf("leftDesired ticks_per_meter is %f\n", leftDesired);
        Serial.printf("rightDesired ticks_per_meter is %f\n", rightDesired);

        float leftVelocity_mps = leftTickSpeedF / TICKS_PER_METER;
        float rightVelocity_mps = rightTickSpeedF / TICKS_PER_METER;
        float robotVelocity_mps = (leftVelocity_mps + rightVelocity_mps) / 2.0f;
        Serial.printf("current left velocity(m/s) is %f\n", leftVelocity_mps);
        Serial.printf("current right velocity(m/s) is %f\n", rightVelocity_mps);
        Serial.printf("current robot velocity(m/s) is %f\n", robotVelocity_mps);
        

        // 5. Compute PID outputs (volts)
        // float leftFF = getFeedforwardVoltage(leftDesired);
        // float rightFF = getFeedforwardVoltage(rightDesired);

        float leftFF = getFeedforwardVoltageLeft(leftDesired);
        float rightFF = getFeedforwardVoltageRight(rightDesired);

        // float leftFF  = getFeedforwardVoltageLeft(leftDesired);
        // float rightFF = getFeedforwardVoltageRight(rightDesired);

        Serial.printf("left FF voltage is %.2f\n", leftFF);
        Serial.printf("right FF voltage is %.2f\n", rightFF);

        float leftPIDout = _pidLeft.compute(leftDesired, leftTickSpeedF);
        float rightPIDout = _pidRight.compute(rightDesired, rightTickSpeedF);
        Serial.printf("leftPIDout voltage is %.2f\n", leftPIDout);
        Serial.printf("rightPIDout voltage is %.2f\n", rightPIDout);

        float leftCmd = leftFF + leftPIDout+ left_angular_ff;
        float rightCmd = rightFF + rightPIDout +  right_angular_ff;
        Serial.printf("leftCmd before constrain is %f\n", leftCmd);
        Serial.printf("rightCmd before constrain is %f\n", rightCmd);

        leftCmd = constrain(leftCmd, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE);
        rightCmd = constrain(rightCmd, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE);

        Serial.printf("leftCmd after constrain is %f\n", leftCmd);
        Serial.printf("rightCmd after constrain is %f\n", rightCmd);

        motorsVoltage[0] = leftCmd;
        motorsVoltage[1] = rightCmd;

        // setLeftMotorsVoltage(leftCmd);
        // setRightMotorsVoltage(rightCmd);
        // float leftCmd = _pidLeft.compute(leftDesired, leftTickSpeed);
        // float rightCmd = _pidRight.compute(rightDesired, rightTickSpeed);
        // Serial.printf("leftCmd is %f\n", leftCmd);
        // Serial.printf("rightCmd is%f\n", rightCmd);
        // // 6. Apply to motors (the motor driver will convert volts to PWM)
        // setLeftMotorsVoltage(leftCmd);
        // setRightMotorsVoltage(rightCmd);
    }

    void reset() {
        _pidLeft.reset();
        _pidRight.reset();
        _targetV = 0.0f;
        _targetOmega = 0.0f;
        _straightMode = false;
        _headingSetpointValid = false;
    }

private:
    PID _pidLeft, _pidRight, _pidOmega;
    float _targetV = 0.0f, _targetOmega = 0.0f;
    bool _straightMode = false;
    float _headingSetpoint = 0.0f;
    bool _headingSetpointValid = false;
    float prevLeft = 0.0f, prevRight = 0.0f;
};