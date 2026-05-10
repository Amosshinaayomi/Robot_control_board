#include <EncoderManager.h>

// Create encoder manager instance
EncoderManager encoderManager;

// Encoder pins
#define ENCODER_PIN_1 PB_2
#define ENCODER_PIN_2 PC_15  
#define ENCODER_PIN_3 PB_5
#define ENCODER_PIN_4 PA_15

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("Encoder Manager Library Test");
  
  // Add encoders
  encoderManager.addEncoder(ENCODER_PIN_1);
  encoderManager.addEncoder(ENCODER_PIN_2);
  encoderManager.addEncoder(ENCODER_PIN_3);
  encoderManager.addEncoder(ENCODER_PIN_4);
  
  // Start polling at 100kHz (10µs intervals)
  if(encoderManager.begin(100000, TIM2)) {
    Serial.println("Encoder manager started successfully");
  } else {
    Serial.println("Failed to start encoder manager");
  }
  
  encoderManager.printDebugInfo();
}

void loop() {
  // Print encoder ticks every second
  static unsigned long lastPrint = 0;
  if(millis() - lastPrint > 1000) {
    encoderManager.printAllTicks();
    lastPrint = millis();
  }
  
  // Example: reset encoder 0 when it reaches 1000 ticks
  if(encoderManager.getTicks(0) > 1000) {
    encoderManager.resetTicks(0);
    Serial.println("Reset encoder 0");
  }
}