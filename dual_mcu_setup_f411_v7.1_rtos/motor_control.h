#pragma once
#include "STMPWMTimer.h"
#include "pins.h"
#include "RobotConfig.h"

// Motor control pins
uint8_t motorAcontrolpins[3] = {PA10, PA6, PA7};
uint8_t motorBcontrolpins[3] = {PA1, PA5, PA4};
uint8_t motorCcontrolpins[3] = {PA8, PB0, PB13};
uint8_t motorDcontrolpins[3] = {PA9, PB1, PB10};

// Create motor objects
STMPWMTimer motorA(motorAcontrolpins[0], 25000);   // TIM1_CH3
STMPWMTimer motorB(motorBcontrolpins[0], 25000);   // TIM1_CH2 (same timer, different channel!)
STMPWMTimer motorC(motorCcontrolpins[0], 25000);   // TIM1_CH1 (same timer, different channel!)
STMPWMTimer motorD(motorDcontrolpins[0], 25000);   // TIM2_CH2 (different timer)


bool initMotorDrivers()
{
  if(!motorA.attach())
  {
    return false;
  }
  if(!motorB.attach())
  {
    return false;
  }
  if(!motorC.attach())
  {
    return false;
  }
  if(!motorD.attach())
  {
    return false;
  }
  for(uint8_t i = 1; i < 3; i++)
  {
    pinMode(motorAcontrolpins[i], OUTPUT);
    pinMode(motorBcontrolpins[i], OUTPUT);
    pinMode(motorCcontrolpins[i], OUTPUT);
    pinMode(motorDcontrolpins[i], OUTPUT);
  }
  pinMode(MOTOR_STBY_PIN, OUTPUT);
  return true;
}


void move_f() {
  digitalWrite(MOTOR_STBY_PIN, HIGH);
  
  motorA.write(100.0);
  digitalWrite(motorAcontrolpins[1], HIGH);
  digitalWrite(motorAcontrolpins[2], LOW);

  motorB.write(100.0);
  digitalWrite(motorBcontrolpins[1], HIGH);
  digitalWrite(motorBcontrolpins[2], LOW);

  motorC.write(100.0);
  digitalWrite(motorCcontrolpins[1], HIGH);
  digitalWrite(motorCcontrolpins[2], LOW);

  motorD.write(100.0);
  digitalWrite(motorDcontrolpins[1], HIGH);
  digitalWrite(motorDcontrolpins[2], LOW);
}

void move_b() {
  digitalWrite(MOTOR_STBY_PIN, HIGH);
  
  motorA.write(100.0);
  digitalWrite(motorAcontrolpins[1], LOW);
  digitalWrite(motorAcontrolpins[2], HIGH);

  motorB.write(100.0);
  digitalWrite(motorBcontrolpins[1], LOW);
  digitalWrite(motorBcontrolpins[2], HIGH);

  motorC.write(100.0);
  digitalWrite(motorCcontrolpins[1], LOW);
  digitalWrite(motorCcontrolpins[2], HIGH);

  motorD.write(100.0);
  digitalWrite(motorDcontrolpins[1], LOW);
  digitalWrite(motorDcontrolpins[2], HIGH);
}

void analog_move_f(int dutyCycle)
{
  motorA.write(dutyCycle);
  digitalWrite(motorAcontrolpins[1], HIGH);
  digitalWrite(motorAcontrolpins[2], LOW);

  motorB.write(dutyCycle);
  digitalWrite(motorBcontrolpins[1], HIGH);
  digitalWrite(motorBcontrolpins[2], LOW);

  motorC.write(dutyCycle);
  digitalWrite(motorCcontrolpins[1], HIGH);
  digitalWrite(motorCcontrolpins[2], LOW);

  motorD.write(dutyCycle);
  digitalWrite(motorDcontrolpins[1], HIGH);
  digitalWrite(motorDcontrolpins[2], LOW);
}

void analog_move_b(int dutyCycle) {
  digitalWrite(MOTOR_STBY_PIN, HIGH);
  
  motorA.write(dutyCycle);
  digitalWrite(motorAcontrolpins[1], LOW);
  digitalWrite(motorAcontrolpins[2], HIGH);

  motorB.write(dutyCycle);
  digitalWrite(motorBcontrolpins[1], LOW);
  digitalWrite(motorBcontrolpins[2], HIGH);

  motorC.write(dutyCycle);
  digitalWrite(motorCcontrolpins[1], LOW);
  digitalWrite(motorCcontrolpins[2], HIGH);

  motorD.write(dutyCycle);
  digitalWrite(motorDcontrolpins[1], LOW);
  digitalWrite(motorDcontrolpins[2], HIGH);
}

void analog_turn_l(int dutyCycle)
{
  digitalWrite(MOTOR_STBY_PIN, HIGH);
  
  motorA.write(dutyCycle);
  digitalWrite(motorAcontrolpins[1], HIGH);
  digitalWrite(motorAcontrolpins[2], LOW);

  motorB.write(dutyCycle);
  digitalWrite(motorBcontrolpins[1], LOW);
  digitalWrite(motorBcontrolpins[2], HIGH);

  motorC.write(dutyCycle);
  digitalWrite(motorCcontrolpins[1], HIGH);
  digitalWrite(motorCcontrolpins[2], LOW);

  motorD.write(dutyCycle);
  digitalWrite(motorDcontrolpins[1], LOW);
  digitalWrite(motorDcontrolpins[2], HIGH);
}

void analog_turn_r(int dutyCycle)
{
  digitalWrite(MOTOR_STBY_PIN, HIGH);
  
  motorA.write(dutyCycle);
  digitalWrite(motorAcontrolpins[1], LOW);
  digitalWrite(motorAcontrolpins[2], HIGH);

  motorB.write(dutyCycle);
  digitalWrite(motorBcontrolpins[1], HIGH);
  digitalWrite(motorBcontrolpins[2], LOW);

  motorC.write(dutyCycle);
  digitalWrite(motorCcontrolpins[1], LOW);
  digitalWrite(motorCcontrolpins[2], HIGH);

  motorD.write(dutyCycle);
  digitalWrite(motorDcontrolpins[1], HIGH);
  digitalWrite(motorDcontrolpins[2], LOW);
}


void halt()
{
  digitalWrite(MOTOR_STBY_PIN, LOW);
  
  motorA.write(0);
  digitalWrite(motorAcontrolpins[1], LOW);
  digitalWrite(motorAcontrolpins[2], LOW);

  motorB.write(0);
  digitalWrite(motorBcontrolpins[1], LOW);
  digitalWrite(motorBcontrolpins[2], LOW);

  motorC.write(0);
  digitalWrite(motorCcontrolpins[1], LOW);
  digitalWrite(motorCcontrolpins[2], LOW);

  motorD.write(0);
  digitalWrite(motorDcontrolpins[1], LOW);
  digitalWrite(motorDcontrolpins[2], LOW);
}

void setLeftMotorsVoltage(float voltage) {
    // Clamp to maximum allowed voltage
    voltage = constrain(voltage, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE);

    // Convert voltage to duty cycle based on current battery voltage
    float duty = (voltage / BATTERY_VOLTAGE) * 100.0f;
    duty = constrain(duty, -100.0f, 100.0f);

    bool forward = duty >= 0;
    float absDuty = fabs(duty);

    digitalWrite(MOTOR_STBY_PIN, HIGH);
    motorA.write(absDuty);
    motorC.write(absDuty);
    digitalWrite(motorAcontrolpins[1], forward ? HIGH : LOW);
    digitalWrite(motorAcontrolpins[2], forward ? LOW : HIGH);
    digitalWrite(motorCcontrolpins[1], forward ? HIGH : LOW);
    digitalWrite(motorCcontrolpins[2], forward ? LOW : HIGH);
}

void setRightMotorsVoltage(float voltage) {
    voltage = constrain(voltage, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE);
    float duty = (voltage / BATTERY_VOLTAGE) * 100.0f;
    duty = constrain(duty, -100.0f, 100.0f);

    bool forward = duty >= 0;
    float absDuty = fabs(duty);

    digitalWrite(MOTOR_STBY_PIN, HIGH);
    motorB.write(absDuty);
    motorD.write(absDuty);
    digitalWrite(motorBcontrolpins[1], forward ? HIGH : LOW);
    digitalWrite(motorBcontrolpins[2], forward ? LOW : HIGH);
    digitalWrite(motorDcontrolpins[1], forward ? HIGH : LOW);
    digitalWrite(motorDcontrolpins[2], forward ? LOW : HIGH);
}


// LOGGED DATA ON GROUND
// 00:11:51.722 -> Sent command 0x13 with 0 params
// 00:11:52.703 -> Sent RUN command
// 00:11:52.772 -> SPEED_TEST: 2.00,0.00,21.67,0.00
// 00:11:55.894 -> SPEED_TEST: 3.00,0.00,35.67,0.00
// 00:11:58.977 -> SPEED_TEST: 4.00,0.00,50.00,0.00
// 00:12:02.072 -> SPEED_TEST: 5.00,0.00,65.00,0.00
// 00:14:11.462 -> left_voltage,right_voltage,left_speed,right_speed
// 00:14:11.462 -> 2.00,0.00,6.33,0.00
// 00:14:11.462 -> 3.00,0.00,17.33,0.67
// 00:14:11.462 -> 4.00,0.00,29.33,3.00
// 00:14:11.462 -> 5.00,0.00,43.67,4.00
// 00:14:11.462 -> 6.00,0.00,60.67,1.33
// 00:14:11.462 -> 7.00,0.00,78.67,1.67
// 00:14:11.462 -> 8.00,0.00,91.33,9.33
// 00:14:11.497 -> 9.00,0.00,105.33,6.33
// 00:14:11.497 -> 0.00,2.00,0.00,2.33
// 00:14:11.497 -> 0.00,3.00,0.00,18.67
// 00:14:11.497 -> 0.00,4.00,0.00,30.67
// 00:14:11.497 -> 0.00,5.00,0.00,50.00
// 00:14:11.497 -> 0.00,6.00,0.00,62.67
// 00:14:11.497 -> 0.00,7.00,0.33,78.33
// 00:14:11.497 -> 0.00,8.00,0.00,92.67
// 00:14:11.497 -> 0.00,9.00,0.00,107.00
// 00:14:11.497 -> 2.00,2.00,19.00,19.00
// 00:14:11.497 -> 3.00,3.00,31.67,33.00
// 00:14:11.497 -> 4.00,4.00,47.33,49.33
// 00:14:11.497 -> 5.00,5.00,62.67,64.00
// 00:14:11.497 -> 6.00,6.00,78.00,80.00
// 00:14:11.497 -> 7.00,7.00,93.33,94.67
// 00:14:11.497 -> 8.00,8.00,108.33,109.67
// 00:14:11.497 -> 9.00,9.00,123.00,123.67


// LOGGED DATA WITH FREE WHEELS
// 00:25:47.813 -> Sent command 0x07 with 0 params
// 00:25:47.813 -> Sent test start to F411
// 00:25:50.953 -> SPEED_TEST: 2.00,0.00,20.67,0.00
// 00:25:54.022 -> SPEED_TEST: 3.00,0.00,35.00,0.00
// 00:25:57.107 -> SPEED_TEST: 4.00,0.00,50.00,0.00
// 00:26:00.221 -> SPEED_TEST: 5.00,0.00,65.00,0.00
// 00:26:03.337 -> SPEED_TEST: 6.00,0.00,81.00,0.00
// 00:26:06.421 -> SPEED_TEST: 7.00,0.00,96.00,0.00
// 00:26:09.507 -> SPEED_TEST: 8.00,0.00,112.00,0.00
// 00:26:12.623 -> SPEED_TEST: 9.00,0.00,127.00,0.00
// 00:26:15.721 -> SPEED_TEST: 0.00,2.00,0.00,22.00
// 00:26:18.835 -> SPEED_TEST: 0.00,3.00,0.00,36.67
// 00:26:21.938 -> SPEED_TEST: 0.00,4.00,0.00,52.33
// 00:26:25.023 -> SPEED_TEST: 0.00,5.00,0.00,68.67
// 00:26:28.123 -> SPEED_TEST: 0.00,6.00,0.00,83.67
// 00:26:31.223 -> SPEED_TEST: 0.00,7.00,0.00,99.33
// 00:26:34.337 -> SPEED_TEST: 0.00,8.00,0.00,113.33
// 00:26:37.423 -> SPEED_TEST: 0.00,9.00,0.00,127.33
// 00:26:40.538 -> SPEED_TEST: 2.00,2.00,21.67,23.67
// 00:26:43.623 -> SPEED_TEST: 3.00,3.00,35.33,38.33
// 00:26:46.738 -> SPEED_TEST: 4.00,4.00,50.00,53.67
// 00:26:49.825 -> SPEED_TEST: 5.00,5.00,64.67,68.67
// 00:26:52.937 -> SPEED_TEST: 6.00,6.00,80.00,84.33
// 00:26:56.025 -> SPEED_TEST: 7.00,7.00,95.00,99.33
// 00:26:59.126 -> SPEED_TEST: 8.00,8.00,111.00,112.67
// 00:27:02.221 -> SPEED_TEST: 9.00,9.00,125.00,125.00
// 00:27:47.623 -> left_voltage,right_voltage,left_speed,right_speed
// 00:27:47.623 -> 2.00,0.00,20.67,0.00
// 00:27:47.656 -> 3.00,0.00,35.00,0.00
// 00:27:47.656 -> 4.00,0.00,50.00,0.00
// 00:27:47.656 -> 5.00,0.00,65.00,0.00
// 00:27:47.656 -> 6.00,0.00,81.00,0.00
// 00:27:47.656 -> 7.00,0.00,96.00,0.00
// 00:27:47.656 -> 8.00,0.00,112.00,0.00
// 00:27:47.656 -> 9.00,0.00,127.00,0.00
// 00:27:47.656 -> 0.00,2.00,0.00,22.00
// 00:27:47.656 -> 0.00,3.00,0.00,36.67
// 00:27:47.656 -> 0.00,4.00,0.00,52.33
// 00:27:47.656 -> 0.00,5.00,0.00,68.67
// 00:27:47.656 -> 0.00,6.00,0.00,83.67
// 00:27:47.656 -> 0.00,7.00,0.00,99.33
// 00:27:47.656 -> 0.00,8.00,0.00,113.33
// 00:27:47.656 -> 0.00,9.00,0.00,127.33
// 00:27:47.656 -> 2.00,2.00,21.67,23.67
// 00:27:47.656 -> 3.00,3.00,35.33,38.33
// 00:27:47.656 -> 4.00,4.00,50.00,53.67
// 00:27:47.688 -> 5.00,5.00,64.67,68.67
// 00:27:47.688 -> 6.00,6.00,80.00,84.33
// 00:27:47.688 -> 7.00,7.00,95.00,99.33
// 00:27:47.688 -> 8.00,8.00,111.00,112.67
// 00:27:47.688 -> 9.00,9.00,125.00,125.00



// 2.00,2.00,6.86,6.86,0.00
// 3.00,3.00,28.00,29.00,-0.44
// 4.00,4.00,41.00,42.00,-0.77
// 5.00,5.00,54.00,55.00,-1.11
// 6.00,6.00,67.00,68.00,-1.15
// 7.00,7.00,76.00,79.00,-1.15
// 8.00,8.00,88.00,88.00,-0.74
// 9.00,9.00,96.00,96.00,-0.71
// 2.00,-2.00,-2.00,-4.00,10.25
// 3.00,-3.00,-2.00,2.00,22.06
// 4.00,-4.00,2.00,0.00,35.39
// 5.00,-5.00,-4.00,0.00,45.27
// 6.00,-6.00,0.00,-4.00,58.27
// 7.00,-7.00,2.00,-4.00,61.77
// 8.00,-8.00,0.00,0.00,67.26
// 9.00,-9.00,0.00,0.00,69.20



// 2.00,2.00,7.35,7.84,0.00
// 3.00,3.00,28.00,29.00,-0.56
// 4.00,4.00,40.00,41.00,-0.64
// 5.00,5.00,53.00,54.00,-1.49
// 6.00,6.00,66.00,68.00,-1.02
// 7.00,7.00,75.00,77.00,-1.27
// 8.00,8.00,87.00,89.00,-1.60
// 9.00,9.00,95.00,95.00,-1.42
// 2.00,-2.00,2.00,0.00,11.00
// 3.00,-3.00,-2.00,2.00,22.58
// 4.00,-4.00,0.00,-2.00,33.63
// 5.00,-5.00,0.00,-4.00,46.45
// 6.00,-6.00,0.00,-4.00,54.92
// 7.00,-7.00,-6.00,2.00,61.29
// 8.00,-8.00,-6.00,2.00,66.93
// 9.00,-9.00,0.00,-2.00,71.77




// left_voltage,right_voltage,left_speed,right_speed, voltageDiff, yawRate_radps, yawRate_dps
// 2.00,2.00,7.84,8.33,0.00
// 3.00,3.00,29.00,30.00,-0.42
// 4.00,4.00,41.00,43.00,-0.36
// 5.00,5.00,54.00,54.00,-0.41
// 6.00,6.00,67.00,67.00,-0.08
// 7.00,7.00,77.00,78.00,-0.15
// 8.00,8.00,88.00,90.00,-0.13
// 9.00,9.00,96.00,97.00,0.18
// 2.00,-2.00,2.00,-2.00,11.64
// 3.00,-3.00,-4.00,2.00,22.02
// 4.00,-4.00,0.00,0.00,35.63
// 5.00,-5.00,2.00,-2.00,47.63
// 6.00,-6.00,0.00,-2.00,58.00
// 7.00,-7.00,-6.00,0.00,61.10
// 8.00,-8.00,-6.00,2.00,68.83
// 9.00,-9.00,-2.00,0.00,73.86




// left_voltage,right_voltage,left_speed,right_speed, voltageDiff, yawRate_radps, yawRate_dps
// 2.00,2.00,7.84,7.84,0.00
// 3.00,3.00,29.00,30.00,-0.73
// 4.00,4.00,41.00,43.00,-0.56
// 5.00,5.00,55.00,56.00,-0.62
// 6.00,6.00,68.00,69.00,-0.56
// 7.00,7.00,78.00,80.00,-0.72
// 8.00,8.00,89.00,90.00,-0.79
// 9.00,9.00,98.00,97.00,-0.49
// 2.00,-2.00,0.00,-2.00,10.86
// 3.00,-3.00,4.00,-2.00,21.95
// 4.00,-4.00,-2.00,0.00,35.89
// 5.00,-5.00,-2.00,2.00,46.87
// 6.00,-6.00,-2.00,2.00,58.66
// 7.00,-7.00,0.00,-2.00,64.01
// 8.00,-8.00,0.00,-2.00,68.51
// 9.00,-9.00,-4.00,0.00,72.83



// left_voltage,right_voltage,left_speed,right_speed, voltageDiff, yawRate_radps, yawRate_dps
// 2.00,2.00,7.84,8.33,-0.00
// 3.00,3.00,28.00,30.00,-0.55
// 4.00,4.00,41.00,43.00,-0.34
// 5.00,5.00,55.00,55.00,-0.29
// 6.00,6.00,67.00,69.00,0.04
// 7.00,7.00,77.00,78.00,-0.33
// 8.00,8.00,89.00,91.00,0.07
// 9.00,9.00,98.00,98.00,0.55
// 2.00,-2.00,0.00,-2.00,11.48
// 3.00,-3.00,2.00,-2.00,23.71
// 4.00,-4.00,0.00,0.00,35.67
// 5.00,-5.00,2.00,-2.00,47.44
// 6.00,-6.00,-2.00,2.00,58.15
// 7.00,-7.00,0.00,0.00,63.76
// 8.00,-8.00,0.00,0.00,71.10
// 9.00,-9.00,0.00,0.00,74.44


// 2.00,2.00,7.84,8.33,-0.00
// 3.00,3.00,28.00,29.00,0.03
// 4.00,4.00,42.00,42.00,0.22
// 5.00,5.00,55.00,56.00,0.07
// 6.00,6.00,68.00,68.00,0.05
// 7.00,7.00,77.00,78.00,0.07
// 8.00,8.00,89.00,90.00,0.02
// 9.00,9.00,98.00,98.00,0.93
// 2.00,-2.00,2.00,0.00,12.28
// 3.00,-3.00,2.00,0.00,24.46
// 4.00,-4.00,2.00,-2.00,36.81
// 5.00,-5.00,0.00,0.00,47.83
// 6.00,-6.00,2.00,-2.00,58.83
// 7.00,-7.00,2.00,-2.00,64.14
// 8.00,-8.00,0.00,-2.00,71.45
// 9.00,-9.00,0.00,-4.00,74.84
