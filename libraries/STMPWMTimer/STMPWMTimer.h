// #ifndef STMPWMTimer_H_
// #define STMPWMTimer_H_

// #include "HardwareTimer.h"


// class STMPWMTimer {
//   private: 
//     HardwareTimer* timer;
//     uint32_t channel;
//     uint16_t pin;
//     uint32_t frequency;
//     uint32_t maxPeriod;
//   public:

//   STMPWMTimer(uint16_t pwmPin, uint32_t freq);
//   TIM_TypeDef *TIM_Instance;

//   /**
//    * Set motor speed (duty cycle)
//    * @param percent: 0 to 100 (0% = off, 100% = full speed)
//    */    
//   bool begin();

//   void writeDutyCycle(int percent);
//   /**
//    * Set motor speed with finer control (0.0 to 1.0)
//    * @param speed: 0.0 to 1.0 (0.0 = off, 1.0 = full speed)
//    */
//   void writeDutyCycleNormalized(float speed);

//   uint16_t readDutyCycle();
//   /**
//    * Stop the motor (0% duty cycle)
//    */
//   void stop();
//     /*
//    * Get the actual PWM frequency
//    * @return frequency in Hz
//    */
//    uint32_t getFrequency();

//   /**
//    * Get the timer resolution (number of steps from 0% to 100%)
//    * @return resolution in steps
//    */
//   uint32_t getResolution();
//   /**
//    * Get the HardwareTimer TIM_instance for advanced configuration
//    * @return pointer to HardwareTimer
//    */
//   HardwareTimer* getTimer();
// };

// #endif


#ifndef STMPWMTimer_H
#define STMPWMTimer_H

#include "HardwareTimer.h"
#include <Arduino.h>
#include <map>
#include <set>

class STMPWMTimer {
private:
    // Static registry for shared timer management (like ESP32's LEDC)
    struct TimerData {
        HardwareTimer* timer;
        uint32_t frequency;
        uint32_t resolution;  // ARR + 1
        std::set<uint32_t> usedChannels;  // Which channels are occupied
        bool isInitialized;
    };
    
    static std::map<TIM_TypeDef*, TimerData> timerRegistry;
    static std::map<uint16_t, STMPWMTimer*> pinToInstanceMap;
    
    // Per-instance data
    uint16_t pin;
    uint32_t desiredFrequency;
    TIM_TypeDef* timerInstance;
    uint32_t channel;
    HardwareTimer* sharedTimer;
    bool isAttached;
    
    // Internal methods
    bool findAndAllocateTimerChannel();
    void configureChannel();
    
public:
    // ========== PUBLIC API ==========
    
    /**
     * Constructor - doesn't attach to hardware yet
     * @param pwmPin: Arduino pin with PWM capability (e.g., PA10)
     * @param freq: Desired PWM frequency in Hz (default 20000 for motors, 50 for servos)
     */
    STMPWMTimer(uint16_t pwmPin, uint32_t freq = 20000);
    
    /**
     * Attach PWM to the pin with configured frequency
     * @return true if successful, false if pin doesn't support PWM or channel busy
     */
    bool attach();
    
    /**
     * Detach PWM from pin (free the channel for other use)
     */
    void detach();
    
    /**
     * Check if PWM is attached to pin
     * @return true if attached
     */
    bool attached();
    
    /**
     * Set duty cycle (0-100%)
     * @param percent: 0.0 to 100.0
     */
    void write(float percent);
    
    /**
     * Set duty cycle with normalized value (0.0-1.0)
     * @param value: 0.0 to 1.0
     */
    void writeNormalized(float value);
    
    /**
     * Set raw compare value (0 to resolution-1)
     * @param value: raw compare register value
     */
    void writeMicroseconds(uint32_t microseconds);
    
    /**
     * Read current duty cycle (0-100%)
     * @return duty cycle in percent
     */
    float read();
    
    /**
     * Read current normalized value (0.0-1.0)
     * @return normalized value
     */
    float readNormalized();
    
    /**
     * Read current raw compare value
     * @return raw compare register value
     */
    uint32_t readRaw();
    
    /**
     * Stop output (0% duty cycle)
     */
    void stop();
    
    /**
     * Get actual PWM frequency (may differ slightly from requested)
     * @return actual frequency in Hz
     */
    uint32_t getFrequency();
    
    /**
     * Get timer resolution (max compare value + 1)
     * @return resolution in steps
     */
    uint32_t getResolution();
    
    /**
     * Get maximum microseconds for servo control
     * @return max microseconds in one period
     */
    uint32_t getPeriodMicroseconds();
    
    /**
     * Change frequency (will reconfigure timer if no other channels in use)
     * @param freq: new frequency in Hz
     * @return true if frequency changed successfully
     */
    bool setFrequency(uint32_t freq);
    
    // ========== STATIC MANAGEMENT API ==========
    
    /**
     * Get number of currently allocated timers
     * @return count of active timers
     */
    static uint8_t getTimerCount();
    
    /**
     * Get number of currently attached pins
     * @return count of attached pins
     */
    static uint8_t getAttachedPinCount();
    
    /**
     * Print debug information about all timers and channels
     */
    static void debug();
    
    /**
     * Detach all pins and free all timers
     */
    static void detachAll();
    
    /**
     * Check if a pin supports hardware PWM
     * @param pin: Arduino pin to check
     * @return true if pin supports hardware PWM
     */
    static bool pinSupportsPWM(uint16_t pin);
    
    /**
     * Get recommended frequency for servo control
     * @return 50 Hz (standard for servos)
     */
    static constexpr uint32_t servoFrequency() { return 50; }
    
    /**
     * Get recommended frequency for motor control
     * @return 20000 Hz (standard for motors)
     */
    static constexpr uint32_t motorFrequency() { return 20000; }
    
    /**
     * Get timer instance and channel for a pin (without attaching)
     * @param pin: Arduino pin
     * @param timerOut: will contain timer instance if found
     * @param channelOut: will contain channel number if found
     * @return true if pin has PWM capability
     */
    static bool getTimerInfo(uint16_t pin, TIM_TypeDef* &timerOut, uint32_t &channelOut);
};

#endif