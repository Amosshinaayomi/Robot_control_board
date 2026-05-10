# EncoderManager – Direction‑Aware Quadrature Emulation for Optical/Mechanical Encoders

This library polls up to 4 digital encoders (e.g., optical slot sensors) at a fixed frequency using a single hardware timer. It implements **debounce** and **direction detection** by tracking 16‑bit sample patterns.

## Features
- Direction‑aware counting (increments/decrements)
- Configurable debounce bits (4–8 recommended)
- 32‑bit signed tick counter (no overflow)
- Works with any simple digital encoder (one channel)
- Lightweight ISR – minimal CPU load

## Hardware Requirements
- Encoder output connected to a digital pin (with optional pull‑up)
- One hardware timer (e.g., TIM2, TIM3, TIM4) not used elsewhere
- STM32 with Arduino core (or compatible)

## Wiring
- Encoder signal pin → GPIO (e.g., PA0)
- VCC → 3.3V/5V (depending on encoder)
- GND → common ground

## Usage

```cpp
#include "EncoderManager.h"

EncoderManager encManager;

void setup() {
    Serial.begin(115200);
    encManager.addEncoder(PA0);  // left front
    encManager.addEncoder(PA1);  // right front
    encManager.addEncoder(PA2);  // left rear
    encManager.addEncoder(PA3);  // right rear

    // Optional: set debounce bits (default 6)
    encManager.setDebounceBits(6);

    // Start polling at 5 kHz
    if (!encManager.begin(5000, TIM2)) {
        Serial.println("Encoder manager init failed");
        while(1);
    }
}

void loop() {
    // Read ticks (non‑blocking, no interrupts disabled)
    int32_t left = encManager.getTicks(0);
    int32_t right = encManager.getTicks(1);
    Serial.printf("Left: %ld, Right: %ld\n", left, right);
    delay(100);
}