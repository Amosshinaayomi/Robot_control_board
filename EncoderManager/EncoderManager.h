#ifndef ENCODER_MANAGER_H
#define ENCODER_MANAGER_H

#include <Arduino.h>
#include <HardwareTimer.h>

class EncoderManager {
private:
  struct Encoder {
    int pin;
    volatile long ticks;
    uint16_t readings;
    uint16_t transition;
    bool initialized;
  };

  static const uint8_t MAX_ENCODERS = 8; // Allow more flexibility
  Encoder encoders_[MAX_ENCODERS];
  HardwareTimer* timer_;
  uint8_t encoderCount_;

public:
  EncoderManager();
  ~EncoderManager();
  
  bool addEncoder(int pin);
  bool begin(uint32_t pollingFrequency = 100000, TIM_TypeDef* timerInstance = TIM2);
  void pollAll();
  
  long getTicks(uint8_t encoderIndex) const;
  void resetTicks(uint8_t encoderIndex);
  void resetAllTicks();
  
  uint8_t getEncoderCount() const;
  int getPin(uint8_t encoderIndex) const;
  bool isInitialized(uint8_t encoderIndex) const;
  
  void printAllTicks();
  void printDebugInfo();
};

#endif