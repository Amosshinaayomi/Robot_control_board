#include "STMPWMTimer.h"
#include <EncoderManager.h>
#include <BMI160.h>
// // --- 1. Configuration ---
#define ENCODER_PIN_1 PB0 
#define ENCODER_PIN_2 PC15 // PC15(F411)
#define ENCODER_PIN_3 PB5 // PB5
#define ENCODER_PIN_4 PA15 // PA15
#define POLL_DELAY_MICROS (30UL) // 30µs between polls
#define MOTOR_FREQ 20000

#define TOTAL_ENCODERS 4
#define MOTOR_STBY_PIN PB12

#define UART_TX PA2
#define UART_RX PA3

#define SCL PB8
#define SDA PB9

#define LED_PIN PC13

uint8_t motorAcontrolpins[3] = {PA10, PA6, PA7};
uint8_t motorBcontrolpins[3] = {PA1, PA5, PA4};
uint8_t motorCcontrolpins[3] = {PA8, PB1, PB10};
uint8_t motorDcontrolpins[3] = {PA9, PB0, PB13};
// Create motor objects
STMPWMTimer motorA(motorAcontrolpins[0], 20000);   // TIM1_CH3
STMPWMTimer motorB(motorBcontrolpins[0], 20000);   // TIM1_CH2 (same timer, different channel!)
STMPWMTimer motorC(motorCcontrolpins[0], 20000);   // TIM1_CH1 (same timer, different channel!)
STMPWMTimer motorD(motorDcontrolpins[0], 20000);   // TIM2_CH2 (different timer)
// Create encoder manager instance
EncoderManager encoderManager;

void setup() {
    Serial.begin(115200);

    // Attach all motors (automatically manages timer sharing!)
    initMotorDrivers();
    // Debug info
    STMPWMTimer::debug();
    // Setup encoders
    encoderManager.addEncoder(ENCODER_PIN_1);
    encoderManager.addEncoder(ENCODER_PIN_2);
    encoderManager.addEncoder(ENCODER_PIN_3);
    encoderManager.addEncoder(ENCODER_PIN_3);

    // Start polling at 100kHz (10µs intervals)
    if(encoderManager.begin(100000, TIM2)) {
      Serial.println("Encoder manager started successfully");
    } else {
      Serial.println("Failed to start encoder manager");
    }

    encoderManager.printDebugInfo();    
}
uint8_t directionState = 1;

void loop() {
    // Motor control logic

    for(uint8_t i = 30; i <= 70; i+=10)
    {
      if(directionState == 1)
      {
        analog_move_f(i);
        Serial.println("move_f");
      } 
      else if (directionState == 2){
          analog_move_b(i);
          Serial.println("move_b");
      }
      Serial.printf("speed: %i\n", i);
      delay(500);
    }

    for(uint8_t i = 70; i > 30; i-=10)
    {
      if(directionState == 1)
      {
        analog_move_f(i);
        Serial.println("move_f");
      } 
      else if (directionState == 2){
          analog_move_b(i);
          Serial.println("move_b");
      }
      Serial.printf("speed: %i\n", i);
      delay(500);
    }
    directionState++;
    if(directionState > 2) directionState = 1;
    Serial.print("direction State: "); Serial.println(directionState);
    Serial.println("Hello New World");
}


void initMotorDrivers()
{
  if(!motorA.attach())
  {
    Serial.printf("attaching pwm for motorA failed");
  }
  if(!motorB.attach())
  {
    Serial.printf("attaching pwm for motorB failed");
  }
  if(!motorC.attach())
  {
    Serial.printf("attaching pwm for motorC failed");
  }
  if(!motorD.attach())
  {
    Serial.printf("attaching pwm for motorD failed");
  }
  for(uint8_t i = 1; i < 3; i++)
  {
    pinMode(motorAcontrolpins[i], OUTPUT);
    pinMode(motorBcontrolpins[i], OUTPUT);
    pinMode(motorCcontrolpins[i], OUTPUT);
    pinMode(motorDcontrolpins[i], OUTPUT);
  }
  pinMode(MOTOR_STBY_PIN, OUTPUT);
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

