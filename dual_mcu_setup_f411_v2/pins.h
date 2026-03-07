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
