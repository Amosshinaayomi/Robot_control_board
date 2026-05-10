// #ifndef ENCODER_MANAGER_H
// #define ENCODER_MANAGER_H

// #include <Arduino.h>
// #include <HardwareTimer.h>

// class EncoderManager {
// private:
//   struct Encoder {
//     int pin;
//     volatile long ticks;
//     uint16_t readings;
//     uint16_t transition;
//     bool initialized;
//   };

//   static const uint8_t MAX_ENCODERS = 8; // Allow more flexibility
//   Encoder encoders_[MAX_ENCODERS];
//   HardwareTimer* timer_;
//   uint8_t encoderCount_;

// public:
//   EncoderManager();
//   ~EncoderManager();
  
//   bool addEncoder(int pin);
//   bool begin(uint32_t pollingFrequency = 100000, TIM_TypeDef* timerInstance = TIM2);
//   void pollAll();
  
//   long getTicks(uint8_t encoderIndex) const;
//   void resetTicks(uint8_t encoderIndex);
//   void resetAllTicks();
  
//   uint8_t getEncoderCount() const;
//   int getPin(uint8_t encoderIndex) const;
//   bool isInitialized(uint8_t encoderIndex) const;
  
//   void printAllTicks();
//   void printDebugInfo();
// };

// #endif


#ifndef ENCODER_MANAGER_H
#define ENCODER_MANAGER_H

#include <Arduino.h>
#include <HardwareTimer.h>

class EncoderManager {
private:
  struct Encoder {
    int pin;
    volatile long ticks;
    uint16_t readings;      // 16-bit shift register of recent samples
    uint16_t transition;    // expected edge pattern (masked to debounceBits_)
    bool initialized;
    bool direction;         // true = forward (increment), false = backward (decrement)
  };

  static const uint8_t MAX_ENCODERS = 8;
  Encoder encoders_[MAX_ENCODERS];
  HardwareTimer* timer_;
  uint8_t encoderCount_;

  uint8_t debounceBits_;    // number of bits in debounce pattern (2–16)
  uint16_t transitionMask_; // bitmask for debounceBits_ LSBs

public:
  EncoderManager();
  ~EncoderManager();

  bool addEncoder(int pin);
  bool begin(uint32_t pollingFrequency = 100000, TIM_TypeDef* timerInstance = TIM2);
  void pollAll();

  // Debounce configuration
  void setDebounceBits(uint8_t bits);

  long getTicks(uint8_t encoderIndex) const;
  void resetTicks(uint8_t encoderIndex);
  void resetAllTicks();

  uint8_t getEncoderCount() const;
  int getPin(uint8_t encoderIndex) const;
  bool isInitialized(uint8_t encoderIndex) const;

  void setDirection(uint8_t encoderIndex, bool forward);
  bool getCommandedDirection(uint8_t encoderIndex) const;
  void printAllTicks();
  void printDebugInfo();
};

#endif