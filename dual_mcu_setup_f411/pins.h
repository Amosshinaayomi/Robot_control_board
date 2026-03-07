#pragma once
#include <Arduino.h>

// // --- 1. Configuration ---
#define ENCODER_PIN_1 PB2
#define ENCODER_PIN_2 PC15 // PC15(F411)
#define ENCODER_PIN_3 PB5  // PB5
#define ENCODER_PIN_4 PA15 // PA15
#define MOTOR_FREQ 20000

#define TOTAL_ENCODERS 4
#define MOTOR_STBY_PIN PB12

#define SDA_PIN PB7
#define SCL_PIN PB6

#define UART_TX PA2
#define UART_RX PA3

#define LED_PIN PC13

// Motor control pins
uint8_t  motorAcontrolpins[3] = {PA10, PA6, PA7};
uint8_t motorBcontrolpins[3] = {PA1, PA5, PA4};
uint8_t motorCcontrolpins[3] = {PA8, PB0, PB13};
uint8_t motorDcontrolpins[3] = {PA9, PB1, PB10};