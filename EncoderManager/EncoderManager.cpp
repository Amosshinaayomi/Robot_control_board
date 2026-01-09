#include "EncoderManager.h"

EncoderManager::EncoderManager() : encoderCount_(0), timer_(nullptr) {
  // Initialize all encoders with safe defaults
  for(int i = 0; i < MAX_ENCODERS; i++) {
    encoders_[i].pin = -1;
    encoders_[i].ticks = 0;
    encoders_[i].readings = 0;
    encoders_[i].transition = 0;
    encoders_[i].initialized = false;
  }
}

EncoderManager::~EncoderManager() {
  if(timer_) {
    timer_->pause();
    delete timer_;
  }
}

bool EncoderManager::addEncoder(int pin) {
  if(encoderCount_ >= MAX_ENCODERS) {
    return false;
  }

  encoders_[encoderCount_].pin = pin;
  encoders_[encoderCount_].ticks = 0;
  encoders_[encoderCount_].readings = 0;
  encoders_[encoderCount_].transition = 0;
  encoders_[encoderCount_].initialized = true;

  pinMode(pin, INPUT); // Using pullup for more stable readings

  // Initialize based on current state
  int initialState = digitalRead(pin);
  Serial.printf("Encoder Initial State is %i\n", initialState);
  if(initialState == HIGH) {
    encoders_[encoderCount_].readings = 0xFFFF;
    encoders_[encoderCount_].transition = 0xFFFE;
  } else {
    encoders_[encoderCount_].readings = 0x0000;
    encoders_[encoderCount_].transition = 0x0001;
  }
  
  encoderCount_++;
  return true;
}

bool EncoderManager::begin(uint32_t pollingFrequency, TIM_TypeDef* timerInstance) {
  if(encoderCount_ == 0) {
    return false; // No encoders added
  }

  timer_ = new HardwareTimer(timerInstance);
  
  if(timer_ == nullptr) {
    return false;
  }
  
  timer_->setOverflow(pollingFrequency, HERTZ_FORMAT);
  timer_->attachInterrupt([this]() { this->pollAll(); });
  timer_->resume();
  
  return true;
}

void EncoderManager::pollAll() {
  for(uint8_t i = 0; i < encoderCount_; i++) {
    if(!encoders_[i].initialized) continue;
    
    encoders_[i].readings = (encoders_[i].readings << 1) | digitalRead(encoders_[i].pin);

    if (encoders_[i].readings == encoders_[i].transition) {
      encoders_[i].ticks++;
      encoders_[i].transition = ~encoders_[i].transition;
    }
  }
}

long EncoderManager::getTicks(uint8_t encoderIndex) const {
  if(encoderIndex >= encoderCount_ || !encoders_[encoderIndex].initialized) {
    return 0;
  }
  return encoders_[encoderIndex].ticks;
}

void EncoderManager::resetTicks(uint8_t encoderIndex) {
  if(encoderIndex < encoderCount_) {
    encoders_[encoderIndex].ticks = 0;
  } 
}

void EncoderManager::resetAllTicks() {
  for(uint8_t i = 0; i < encoderCount_; i++) {
    encoders_[i].ticks = 0;
  }
}

uint8_t EncoderManager::getEncoderCount() const {
  return encoderCount_;
}

int EncoderManager::getPin(uint8_t encoderIndex) const {
  if(encoderIndex >= encoderCount_) return -1;
  return encoders_[encoderIndex].pin;
}

bool EncoderManager::isInitialized(uint8_t encoderIndex) const {
  if(encoderIndex >= encoderCount_) return false;
  return encoders_[encoderIndex].initialized;
}

void EncoderManager::printAllTicks() {
  for(uint8_t i = 0; i < encoderCount_; i++) {
    if(encoders_[i].initialized) {
      Serial.printf("Encoder %i (pin %i) ticks: %ld\n", 
                    i + 1, encoders_[i].pin, encoders_[i].ticks);
    }
  }
}

void EncoderManager::printDebugInfo() {
  Serial.println("=== Encoder Manager Debug Info ===");
  Serial.printf("Total encoders: %i/%i\n", encoderCount_, MAX_ENCODERS);
  Serial.printf("Timer: %s\n", timer_ ? "Active" : "Inactive");
  
  for(uint8_t i = 0; i < encoderCount_; i++) {
    Serial.printf("Encoder %i: pin=%i, ticks=%ld, initialized=%s\n",
                  i + 1, encoders_[i].pin, encoders_[i].ticks,
                  encoders_[i].initialized ? "Yes" : "No");
  }
  Serial.println("==================================");
}
