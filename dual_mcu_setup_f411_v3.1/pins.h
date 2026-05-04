#pragma once
#include <Arduino.h>

// // --- 1. Configuration ---
#define ENCODER_PIN_1 PC15 // Front left
#define ENCODER_PIN_2  PB2 // Front right
#define ENCODER_PIN_3 PA15 // Back left
#define ENCODER_PIN_4 PB5 // Back right
#define MOTOR_FREQ 2000

#define TOTAL_ENCODERS 4
#define MOTOR_STBY_PIN PB12

#define SDA_PIN PB7
#define SCL_PIN PB6

#define UART_TX PA2
#define UART_RX PA3

#define LED_PIN PC13
