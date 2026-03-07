#pragma once
#include "STMPWMTimer.h"

// Motor control pins
uint8_t motorAcontrolpins[3] = {PA10, PA6, PA7};
uint8_t motorBcontrolpins[3] = {PA1, PA5, PA4};
uint8_t motorCcontrolpins[3] = {PA8, PB0, PB13};
uint8_t motorDcontrolpins[3] = {PA9, PB1, PB10};

// Create motor objects
STMPWMTimer motorA(motorAcontrolpins[0], 20000);   // TIM1_CH3
STMPWMTimer motorB(motorBcontrolpins[0], 20000);   // TIM1_CH2 (same timer, different channel!)
STMPWMTimer motorC(motorCcontrolpins[0], 20000);   // TIM1_CH1 (same timer, different channel!)
STMPWMTimer motorD(motorDcontrolpins[0], 20000);   // TIM2_CH2 (different timer)


bool initMotorDrivers()
{
  if(!motorA.attach())
  {
    return false;
    // Serial.printf("attaching pwm for motorA failed");
  }
  if(!motorB.attach())
  {
    return false;
    // Serial.printf("attaching pwm for motorB failed");
  }
  if(!motorC.attach())
  {
    return false;
    // Serial.printf("attaching pwm for motorC failed");
  }
  if(!motorD.attach())
  {
    return false;
    // Serial.printf("attaching pwm for motorD failed");
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


void halt() {
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

void analog_move_f(int dutyCycle)
{
  digitalWrite(MOTOR_STBY_PIN, HIGH);
  
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



void setFrontLeftMotorsPWM(float dutyCycle, bool forward) {
    // motorA
    digitalWrite(MOTOR_STBY_PIN, HIGH);
    motorA.write(dutyCycle);
    digitalWrite(motorAcontrolpins[1], forward ? HIGH : LOW);
    digitalWrite(motorAcontrolpins[2], forward ? LOW : HIGH);
}

void setFrontRightMotorsPWM(float dutyCycle, bool forward) {
    // motorB
    digitalWrite(MOTOR_STBY_PIN, HIGH);
    motorB.write(dutyCycle);
    digitalWrite(motorBcontrolpins[1], forward ? HIGH : LOW);
    digitalWrite(motorBcontrolpins[2], forward ? LOW : HIGH);
}

void setBackLeftMotorsPWM(float dutyCycle, bool forward) {
    // motorC
    digitalWrite(MOTOR_STBY_PIN, HIGH);
    motorC.write(dutyCycle);
    digitalWrite(motorCcontrolpins[1], forward ? HIGH : LOW);
    digitalWrite(motorCcontrolpins[2], forward ? LOW : HIGH);
}

void setBaclRightMotorsPWM(float dutyCycle, bool forward) {
    // motorD
    digitalWrite(MOTOR_STBY_PIN, HIGH);
    motorD.write(dutyCycle);
    digitalWrite(motorDcontrolpins[1], forward ? HIGH : LOW);
    digitalWrite(motorDcontrolpins[2], forward ? LOW : HIGH);
}