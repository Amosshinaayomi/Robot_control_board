#pragma once

#include "PID.h"
#include "motor_control.h"
#include <cmath>

class MotionController {
public:
    MotionController()
        : _pidLeft(KP_VEL, KI_VEL, KD_VEL, CONTROL_DT, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE),
         prevLeft(0.0), prevRight(0.0),
          _pidRight(KP_VEL, KI_VEL, KD_VEL, CONTROL_DT, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE), 
          _pidOmega(KP_OMEGA, KI_OMEGA, KD_OMEGA, CONTROL_DT, -MAX_OMEGA_RADPS, MAX_OMEGA_RADPS) {}
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
    void update(float leftTicksAvg, float rightTicksAvg, float yaw, float yawRate_radps, float dt) {
        // 1. Compute wheel speeds (ticks/s)
        Serial.printf("absolute yaw is %.5f\n", yaw);
        Serial.printf("yawRate is %.5f\n", yawRate_radps);
        Serial.printf("prevleft speed is %f\n", prevLeft);
        Serial.printf("prev right speed is %f\n", prevRight);
        float leftSpeed = (leftTicksAvg - prevLeft) / dt;
        float rightSpeed = (rightTicksAvg - prevRight) / dt;
        Serial.printf("left speed delta is %f\n", leftSpeed);
        Serial.printf("right speed delta is %f\n", rightSpeed);
        prevLeft = leftTicksAvg;
        prevRight = rightTicksAvg;

        // 2. Heading correction if in straight mode
        float desiredOmega = _targetOmega;
        if (_straightMode) {
            if (!_headingSetpointValid) {
                _headingSetpoint = yaw;
                _headingSetpointValid = true;
            }
            float headingError = yaw - _headingSetpoint;
            Serial.printf("heading error in rads %.5f\n", headingError);
            // Normalize to [-π, π] (assuming yaw in radians)
            headingError = atan2f(sinf(headingError), cosf(headingError));
            Serial.printf("normalized heading error in rads %.5f\n", headingError);
            // Adjust angular velocity command
            desiredOmega = KP_HEADING * headingError;
        }

        // desiredOmega = constrain(desiredOmega, -MAX_OMEGA_RADPS, MAX_OMEGA_RADPS);
        float omegaCorrection = _pidOmega.compute(desiredOmega, yawRate_radps);

        Serial.printf("omega correction is %.5f\n", omegaCorrection);
        // 3. Desired side speeds from kinematics (m/s)
        float leftDesired_mps = _targetV - (omegaCorrection * ROBOT_TRACK_WIDTH / 2.0f);
        float rightDesired_mps = _targetV + (omegaCorrection  * ROBOT_TRACK_WIDTH / 2.0f);
        Serial.printf("leftDesired_mps is %f\n", leftDesired_mps);
        Serial.printf("rightDesired_mps is %f\n", rightDesired_mps);
        // 4. Convert to desired ticks/s using encoder resolution
        float leftDesired = leftDesired_mps * TICKS_PER_METER;
        float rightDesired = rightDesired_mps * TICKS_PER_METER;

        Serial.printf("leftDesired ticks_per_meter is %f\n", leftDesired);
        Serial.printf("rightDesired ticks_per_meter is %f\n", rightDesired);


        // 5. Compute PID outputs (volts)
        float leftFF = getFeedforwardVoltage(leftDesired);
        float rightFF = getFeedforwardVoltage(rightDesired);

        // float leftFF  = getFeedforwardVoltageLeft(leftDesired);
        // float rightFF = getFeedforwardVoltageRight(rightDesired);

        Serial.printf("left FF voltage is %.2f\n", leftFF);
        Serial.printf("right FF voltage is %.2f\n", rightFF);

        float leftPIDout = _pidLeft.compute(leftDesired, leftSpeed);
        float rightPIDout = _pidRight.compute(rightDesired, rightSpeed);
        Serial.printf("leftPIDout voltage is %.2f\n", leftPIDout);
        Serial.printf("rightPIDout voltage is %.2f\n", rightPIDout);

        float leftCmd = leftFF + leftPIDout;
        float rightCmd = rightFF + rightPIDout;
        Serial.printf("leftCmd before constrain is %f\n", leftCmd);
        Serial.printf("rightCmd before constrain is %f\n", rightCmd);

        leftCmd = constrain(leftCmd, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE);
        rightCmd = constrain(rightCmd, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE);
        Serial.printf("leftCmd after constrain is %f\n", leftCmd);
        Serial.printf("rightCmd after constrain is %f\n", rightCmd);
        
        Serial.println();
        Serial.println();
        Serial.println();
        setLeftMotorsVoltage(leftCmd);
        setRightMotorsVoltage(rightCmd);
        // float leftCmd = _pidLeft.compute(leftDesired, leftSpeed);
        // float rightCmd = _pidRight.compute(rightDesired, rightSpeed);
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
    PID _pidLeft, 17:03:45.510 -> control loop period 0.050000
17:03:45.510 -> lefTicksAvg is 281.00
17:03:45.510 -> rightTicksAvg is 584.50
17:03:45.510 -> dt is 0.05
17:03:45.510 -> absolute yaw is 1.08516
17:03:45.510 -> prevleft tick is 280.000000
17:03:45.542 -> prev right tick is 581.500000
17:03:45.542 -> leftTickSpeedF is 13
17:03:45.542 -> rightTickSpeedF is 67
17:03:45.542 -> leftDesired_mps is 0.058929
17:03:45.542 -> rightDesired_mps is 0.341071
17:03:45.542 -> current left speed(ticks/sec) is 20.000000
17:03:45.542 -> current right speed(ticks/sec) is 60.000000
17:03:45.542 -> Enc0: cur=0x0000 exp=0x0001 dir=1 ticks=277
17:03:45.542 -> leftDesired ticks_per_meter is 11.543215
17:03:45.542 -> rightDesired ticks_per_meter is 66.809990
17:03:45.542 -> current left velocity(m/s) is 0.068068
17:03:45.542 -> current right velocity(m/s) is 0.340339
17:03:45.542 -> current robot velocity(m/s) is 0.204204
17:03:45.542 -> left FF voltage is 2.00
17:03:45.542 -> right FF voltage is 5.22
17:03:45.542 -> leftPIDout voltage is -0.16
17:03:45.542 -> rightPIDout voltage is 0.01
17:03:45.542 -> leftCmd before constrain is 1.838889
17:03:45.542 -> rightCmd before constrain is 5.234482
17:03:45.542 -> leftCmd after constrain is 1.838889
17:03:45.542 -> rightCmd after constrain is 5.234482
17:03:45.542 -> left motor voltage is 1.839
17:03:45.542 -> right motor voltage is 5.234
17:03:45.542 -> left motor direction is 1
17:03:45.542 -> right motor direction is 1
17:03:45.542 -> 
17:03:45.542 -> 
17:03:45.542 -> 
17:03:45.578 -> Time stamp: 12122
17:03:45.578 -> Gyro (dps) X,Y,Z: 1.419, 0.753, 0.169
17:03:45.578 -> Accel(g) X,Y,Z: -0.023, 0.068, 1.075
17:03:45.578 -> PITCH:-0.32, ROLL:-0.82, YAW:62.18
17:03:45.578 -> YawRate: 0.17
17:03:45.578 -> Front Left side encoder tick is 278
17:03:45.578 -> Front right side encoder tick is 582
17:03:45.578 -> Back Left side encoder tick is 285
17:03:45.578 -> Back right side encoder tick is 592
17:03:45.578 -> 
17:03:45.578 -> control loop period 0.050000
17:03:45.578 -> lefTicksAvg is 282.00
17:03:45.578 -> rightTicksAvg is 588.50
17:03:45.578 -> dt is 0.05
17:03:45.578 -> absolute yaw is 1.08523
17:03:45.578 -> prevleft tick is 281.000000
17:03:45.578 -> prev right tick is 584.500000
17:03:45.578 -> leftTickSpeedF is 13
17:03:45.578 -> rightTickSpeedF is 67
17:03:45.578 -> leftDesired_mps is 0.058921
17:03:45.578 -> rightDesired_mps is 0.341079
17:03:45.578 -> current left speed(ticks/sec) is 20.000000
17:03:45.578 -> current right speed(ticks/sec) is 80.000000
17:03:45.578 -> leftDesired ticks_per_meter is 11.541546
17:03:45.578 -> rightDesired ticks_per_meter is 66.811661
17:03:45.578 -> current left velocity(m/s) is 0.068068
17:03:45.578 -> current right velocity(m/s) is 0.340339
17:03:45.578 -> current robot velocity(m/s) is 0.204204
17:03:45.578 -> left FF voltage is 2.00
17:03:45.578 -> right FF voltage is 5.22
17:03:45.578 -> leftPIDout voltage is -0.16
17:03:45.578 -> rightPIDout voltage is 0.01
17:03:45.578 -> leftCmd before constrain is 1.838739
17:03:45.578 -> rightCmd before constrain is 5.234739
17:03:45.578 -> leftCmd after constrain is 1.838739
17:03:45.578 -> rightCmd after constrain is 5.234739
17:03:45.578 -> left motor voltage is 1.839
17:03:45.578 -> right motor voltage is 5.235
17:03:45.578 -> left motor direction is 1
17:03:45.578 -> right motor direction is 1
17:03:45.578 -> 
17:03:45.578 -> 
17:03:45.578 -> 
17:03:45.610 -> control loop period 0.050000
17:03:45.610 -> lefTicksAvg is 282.50
17:03:45.610 -> rightTicksAvg is 592.00
17:03:45.610 -> dt is 0.05
17:03:45.610 -> absolute yaw is 1.08522
17:03:45.658 -> prevleft tick is 282.000000
17:03:45.658 -> prev right tick is 588.500000
17:03:45.658 -> leftTickSpeedF is 17
17:03:45.658 -> rightTickSpeedF is 70
17:03:45.658 -> leftDesired_mps is 0.058922
17:03:45.658 -> rightDesired_mps is 0.341078
17:03:45.658 -> current left speed(ticks/sec) is 10.000000
17:03:45.658 -> current right speed(ticks/sec) is 70.000000
17:03:45.658 -> leftDesired ticks_per_meter is 11.541808
17:03:45.658 -> rightDesired ticks_per_meter is 66.811394
17:03:45.658 -> current left velocity(m/s) is 0.085085
17:03:45.658 -> current right velocity(m/s) is 0.357356
17:03:45.658 -> current robot velocity(m/s) is 0.221220
17:03:45.658 -> left FF voltage is 2.00
17:03:45.658 -> right FF voltage is 5.22
17:03:45.658 -> leftPIDout voltage is -0.46
17:03:45.658 -> rightPIDout voltage is -0.29
17:03:45.658 -> leftCmd before constrain is 1.538763
17:03:45.658 -> rightCmd before constrain is 4.934698
17:03:45.658 -> leftCmd after constrain is 1.538763
17:03:45.658 -> rightCmd after constrain is 4.934698
17:03:45.658 -> left motor voltage is 1.539
17:03:45.658 -> right motor voltage is 4.935
17:03:45.658 -> left motor direction is 1
17:03:45.658 -> right motor direction is 1
17:03:45.658 -> 
    float _targetV = 0.0f, _targetOmega = 0.0f;
    bool _straightMode = false;
    float _headingSetpoint = 0.0f;
    bool _headingSetpointValid = false;
    float prevLeft = 0.0f, prevRight = 0.0f;
};