// // MotionController.h
// #pragma once

// #include "PID.h"
// #include "motor_control.h"
// #include <cmath>

// class MotionController {
// public:
//     MotionController()
//         : _pidLeft(KP_VEL, KI_VEL, KD_VEL, CONTROL_DT, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE),
//           _pidOmega(KP_OMEGA, KI_OMEGA, KD_OMEGA, CONTROL_DT, -MAX_OMEGA_RADPS, MAX_OMEGA_RADPS),
//           _pidRight(KP_VEL, KI_VEL, KD_VEL, CONTROL_DT, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE) {}

//     // Set target velocities (linear m/s, angular rad/s)
//     void setTargetVelocity(float v, float omega) {
//         _targetV = v;
//         _targetOmega = omega;
//         _straightMode = false;   // disable heading correction
//     }

//     // Go straight at given speed (m/s), using current heading as reference
//     void setStraight(float speed) {
//         _targetV = speed;
//         _targetOmega = 0.0f;
//         _straightMode = true;
//         _headingSetpointValid = false;   // will capture on next update
//     }

//     // Main control update, to be called at fixed interval (dt seconds)
//     void update(float leftTicksAvg, float rightTicksAvg, float yaw, float yawRate, float dt) {
//         // 1. Compute wheel speeds (ticks/s)
//         static float prevLeft = 0.0f, prevRight = 0.0f;
//         float leftSpeed = (leftTicksAvg - prevLeft) / dt;
//         float rightSpeed = (rightTicksAvg - prevRight) / dt;
        
//         // Serial.printf("left speed is %f\n", leftSpeed);
//         // Serial.printf("right speed is %f\n", rightSpeed);
//         prevLeft = leftTicksAvg;
//         prevRight = rightTicksAvg;

//         float desiredOmega = _targetOmega; // from setTargetVelocity
//         // 2. Heading correction if in straight mode
//         if (_straightMode) {
//             if (!_headingSetpointValid) {
//                 _headingSetpoint = yaw;
//                 _headingSetpointValid = true;
//             }
//             float headingError = yaw - _headingSetpoint;
//             // Normalize to [-π, π] (assuming yaw in radians)
//             headingError = atan2f(sinf(headingError), cosf(headingError));
//             // Adjust angular velocity command
//             // _targetOmega = KP_HEADING * headingError;
//             desiredOmega = KP_HEADING * headingError;
//             // Serial.printf("desiredOmega: %f, headingError: %f\n", desiredOmega, headingError);
//         }
//         // // 3. Desired side speeds from kinematics (m/s)
//         // float leftDesired_mps = _targetV - _targetOmega * ROBOT_TRACK_WIDTH / 2.0f;
//         // float rightDesired_mps = _targetV + _targetOmega * ROBOT_TRACK_WIDTH / 2.0f;

//         // 3. Inner Angular velocity PID
//         float omegaCorrection = _pidOmega.compute(desiredOmega, yawRate);
//         float leftDesired_mps = _targetV - omegaCorrection * ROBOT_TRACK_WIDTH / 2.0f;
//         float rightDesired_mps = _targetV + omegaCorrection * ROBOT_TRACK_WIDTH / 2.0f;

//         // Serial.printf("leftDesired_mps is %f\n", leftDesired_mps);
//         // Serial.printf("rightDesired_mps is %f\n", rightDesired_mps);
//         // 4. Convert to desired ticks/s using encoder resolution
//         float leftDesired = leftDesired_mps * TICKS_PER_METER;
//         float rightDesired = rightDesired_mps * TICKS_PER_METER;

//         // Serial.printf("leftDesired ticks_per_meter is %f\n", leftDesired);
//         // Serial.printf("rightDesired ticks_per_meter is %f\n", rightDesired);
//         // 5. Compute PID outputs (volts)
//         float leftCmd = _pidLeft.compute(leftDesired, leftSpeed);
//         float rightCmd = _pidRight.compute(rightDesired, rightSpeed);
//         // Serial.printf("left voltage is %f\n", leftCmd);
//         // Serial.printf("right voltage is%f\n", rightCmd);
//         // 6. Apply to motors (the motor driver will convert volts to PWM)
//         setLeftMotorsVoltage(leftCmd);
//         setRightMotorsVoltage(rightCmd);
//     }

//     void reset() {
//         _pidLeft.reset();
//         _pidRight.reset();
//         _targetV = 0.0f;
//         _targetOmega = 0.0f;
//         _straightMode = false;
//         _headingSetpointValid = false;
//     }

// private:
//     PID _pidLeft, _pidRight, _pidOmega;
//     float _targetV = 0.0f, _targetOmega = 0.0f;
//     bool _straightMode = false;
//     float _headingSetpoint = 0.0f;
//     bool _headingSetpointValid = false;
// };


// MotionController.h
#pragma once

#include "PID.h"
#include "motor_control.h"
#include <cmath>

class MotionController {
public:
    MotionController()
        : _pidLeft(KP_VEL, KI_VEL, KD_VEL, CONTROL_DT, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE),
          _pidRight(KP_VEL, KI_VEL, KD_VEL, CONTROL_DT, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE) {}

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
    void update(float leftTicksAvg, float rightTicksAvg, float yaw, float dt) {
        // 1. Compute wheel speeds (ticks/s)
        static float prevLeft = 0.0f, prevRight = 0.0f;
        float leftSpeed = (leftTicksAvg - prevLeft) / dt;
        float rightSpeed = (rightTicksAvg - prevRight) / dt;
        Serial.printf("left speed is %f\n", leftSpeed);
        Serial.printf("right speed is %f\n", rightSpeed);
        prevLeft = leftTicksAvg;
        prevRight = rightTicksAvg;

        // 2. Heading correction if in straight mode
        if (_straightMode) {
            if (!_headingSetpointValid) {
                _headingSetpoint = yaw;
                _headingSetpointValid = true;
            }
            float headingError = yaw - _headingSetpoint;
            // Normalize to [-π, π] (assuming yaw in radians)
            headingError = atan2f(sinf(headingError), cosf(headingError));
            // Adjust angular velocity command
            _targetOmega = KP_HEADING * headingError;
        }

        // 3. Desired side speeds from kinematics (m/s)
        float leftDesired_mps = _targetV - _targetOmega * ROBOT_TRACK_WIDTH / 2.0f;
        float rightDesired_mps = _targetV + _targetOmega * ROBOT_TRACK_WIDTH / 2.0f;
        Serial.printf("leftDesired_mps is %f\n", leftDesired_mps);
        Serial.printf("rightDesired_mps is %f\n", rightDesired_mps);
        // 4. Convert to desired ticks/s using encoder resolution
        float leftDesired = leftDesired_mps * TICKS_PER_METER;
        float rightDesired = rightDesired_mps * TICKS_PER_METER;

        Serial.printf("leftDesired ticks_per_meter is %f\n", leftDesired);
        Serial.printf("rightDesired ticks_per_meter is %f\n", rightDesired);
        // 5. Compute PID outputs (volts)
        float leftCmd = _pidLeft.compute(leftDesired, leftSpeed);
        float rightCmd = _pidRight.compute(rightDesired, rightSpeed);
        Serial.printf("leftCmd is %f\n", leftCmd);
        Serial.printf("rightCmd is%f\n", rightCmd);
        // 6. Apply to motors (the motor driver will convert volts to PWM)
        setLeftMotorsVoltage(leftCmd);
        setRightMotorsVoltage(rightCmd);
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
    PID _pidLeft, _pidRight;
    float _targetV = 0.0f, _targetOmega = 0.0f;
    bool _straightMode = false;
    float _headingSetpoint = 0.0f;
    bool _headingSetpointValid = false;
};