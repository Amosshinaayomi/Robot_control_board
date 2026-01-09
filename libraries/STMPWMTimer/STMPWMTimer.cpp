#include "STMPWMTimer.h"

// Initialize static members
std::map<TIM_TypeDef*, STMPWMTimer::TimerData> STMPWMTimer::timerRegistry;
std::map<uint16_t, STMPWMTimer*> STMPWMTimer::pinToInstanceMap;

// ========== CONSTRUCTOR ==========
STMPWMTimer::STMPWMTimer(uint16_t pwmPin, uint32_t freq) : 
    pin(pwmPin), 
    desiredFrequency(freq),
    timerInstance(nullptr),
    channel(0),
    sharedTimer(nullptr),
    isAttached(false) {
    // Constructor doesn't allocate hardware
}

// ========== PRIVATE METHODS ==========
bool STMPWMTimer::findAndAllocateTimerChannel() {
    // Get timer and channel for this pin
    PinName pinName = digitalPinToPinName(pin);
    uint32_t function = pinmap_function(pinName, PinMap_PWM);
    
    if(function == NC) {
        Serial.print("[STMPWMTimer] Pin ");
        Serial.print(pin);
        Serial.println(" doesn't support hardware PWM!");
        return false;
    }
    
    timerInstance = (TIM_TypeDef*)pinmap_peripheral(pinName, PinMap_PWM);
    channel = STM_PIN_CHANNEL(function);
    
    // Check if timer already exists in registry
    if(timerRegistry.find(timerInstance) == timerRegistry.end()) {
        // First time using this timer - create it
        TimerData newTimer;
        newTimer.timer = new HardwareTimer(timerInstance);
        newTimer.frequency = desiredFrequency;
        newTimer.isInitialized = false;
        
        timerRegistry[timerInstance] = newTimer;
        
        Serial.print("[STMPWMTimer] Created new timer at 0x");
        Serial.print((uint32_t)timerInstance, HEX);
        
        // Print timer name if we recognize it
        if(timerInstance == TIM1) Serial.print(" (TIM1)");
        else if(timerInstance == TIM2) Serial.print(" (TIM2)");
        else if(timerInstance == TIM3) Serial.print(" (TIM3)");
        else if(timerInstance == TIM4) Serial.print(" (TIM4)");
        else if(timerInstance == TIM5) Serial.print(" (TIM5)");
        #ifdef TIM8
        else if(timerInstance == TIM8) Serial.print(" (TIM8)");
        #endif
        #ifdef TIM9
        else if(timerInstance == TIM9) Serial.print(" (TIM9)");
        #endif
        #ifdef TIM10
        else if(timerInstance == TIM10) Serial.print(" (TIM10)");
        #endif
        #ifdef TIM11
        else if(timerInstance == TIM11) Serial.print(" (TIM11)");
        #endif
        
        Serial.println();
    } else {
        // Timer exists - check if frequency matches
        TimerData& existingTimer = timerRegistry[timerInstance];
        if(existingTimer.frequency != desiredFrequency) {
            Serial.print("[STMPWMTimer] WARNING: Frequency mismatch! Requested ");
            Serial.print(desiredFrequency);
            Serial.print("Hz, but timer is already at ");
            Serial.print(existingTimer.frequency);
            Serial.println("Hz. Using existing frequency.");
            desiredFrequency = existingTimer.frequency;
        }
    }
    
    // Check if channel is already used
    TimerData& timerData = timerRegistry[timerInstance];
    if(timerData.usedChannels.find(channel) != timerData.usedChannels.end()) {
        Serial.print("[STMPWMTimer] ERROR: Channel ");
        Serial.print(channel);
        Serial.print(" on timer 0x");
        Serial.print((uint32_t)timerInstance, HEX);
        Serial.println(" is already in use!");
        return false;
    }
    
    // Mark channel as used
    timerData.usedChannels.insert(channel);
    
    // Store pointer to shared timer
    sharedTimer = timerData.timer;
    
    return true;
}

void STMPWMTimer::configureChannel() {
    if(!sharedTimer || !timerInstance) return;
    
    TimerData& timerData = timerRegistry[timerInstance];
    
    // If timer not initialized yet, configure it
    if(!timerData.isInitialized) {
        Serial.print("[STMPWMTimer] Configuring timer to ");
        Serial.print(desiredFrequency);
        Serial.println("Hz");
        
        sharedTimer->pause();

        sharedTimer->setOverflow(desiredFrequency, HERTZ_FORMAT);
        timerData.resolution = sharedTimer->getOverflow(TICK_FORMAT) + 1;
        sharedTimer->setPreloadEnable(true);
        
        timerData.isInitialized = true;
    }
    
    // Configure our channel
    sharedTimer->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, pin);
    sharedTimer->setCaptureCompare(channel, 0, PERCENT_COMPARE_FORMAT); // Start at 0%
    sharedTimer->resume();
    Serial.print("[STMPWMTimer] Attached pin ");
    Serial.print(pin);
    Serial.print(" to timer 0x");
    Serial.print((uint32_t)timerInstance, HEX);
    Serial.print(" CH");
    Serial.print(channel);
    Serial.print(" (Resolution: 1/");
    Serial.print(timerData.resolution);
    Serial.println(")");
}

// ========== PUBLIC METHODS ==========
bool STMPWMTimer::attach() {
    if(isAttached) {
        Serial.print("[STMPWMTimer] Pin ");
        Serial.print(pin);
        Serial.println(" is already attached!");
        return true;
    }
    
    if(pinToInstanceMap.find(pin) != pinToInstanceMap.end()) {
        Serial.print("[STMPWMTimer] Pin ");
        Serial.print(pin);
        Serial.println(" is already attached by another instance!");
        return false;
    }
    
    if(!findAndAllocateTimerChannel()) {
        return false;
    }
    
    configureChannel();
    
    // Mark as attached
    isAttached = true;
    pinToInstanceMap[pin] = this;
    
    return true;
}

void STMPWMTimer::detach() {
    if(!isAttached) return;
    
    // Stop output
    stop();
    
    // Free channel
    if(sharedTimer && timerInstance && timerRegistry.find(timerInstance) != timerRegistry.end()) {
        TimerData& timerData = timerRegistry[timerInstance];
        timerData.usedChannels.erase(channel);
        
        // If no channels left, we could free the timer, but keep it for potential reuse
        if(timerData.usedChannels.empty()) {
            Serial.print("[STMPWMTimer] No more channels on timer 0x");
            Serial.print((uint32_t)timerInstance, HEX);
            Serial.println(", keeping timer allocated for reuse.");
        }
    }
    
    // Remove from pin map
    pinToInstanceMap.erase(pin);
    
    isAttached = false;
    sharedTimer = nullptr;
    timerInstance = nullptr;
    
    Serial.print("[STMPWMTimer] Detached pin ");
    Serial.println(pin);
}

bool STMPWMTimer::attached() {
    return isAttached;
}

void STMPWMTimer::write(float percent) {
    if(!isAttached || !sharedTimer) return;
    
    percent = constrain(percent, 0.0f, 100.0f);

    // Read ARR directly from hardware
    uint32_t arr = timerInstance->ARR;
    uint32_t compare;
    if(percent >= 100.0f) {
        compare = arr;  // For 100% use ARR
    } else {
        compare = (uint32_t)((percent / 100.0f) * arr);
    }

    switch(channel) {
        case 1: timerInstance->CCR1 = compare; break;
        case 2: timerInstance->CCR2 = compare; break;
        case 3: timerInstance->CCR3 = compare; break;
        case 4: timerInstance->CCR4 = compare; break;
    }

    Serial.print("Set "); Serial.print(percent);
    Serial.print("%: ARR="); Serial.print(arr);
    Serial.print(", CCR="); Serial.println(compare);


    // uint32_t arr = sharedTimer->getOverflow(TICK_FORMAT); 
    // uint32_t compareFor100 = (100 * (arr + 1)) / 100;
    // Serial.print("ARR: "); Serial.print(arr);
    // Serial.print(", 100% compare should be: "); Serial.print(compareFor100);
    // sharedTimer->setCaptureCompare(channel, percent, PERCENT_COMPARE_FORMAT);


    // uint32_t actualCompare = sharedTimer->getCaptureCompare(channel, TICK_COMPARE_FORMAT);
    // Serial.print(", Actual compare: "); Serial.println(actualCompare);
}

void STMPWMTimer::writeNormalized(float value) {
    write(value * 100.0f);
}

void STMPWMTimer::writeMicroseconds(uint32_t microseconds) {
    if(!isAttached || !sharedTimer) return;
    
    uint32_t periodMicroseconds = 1000000 / getFrequency();
    if(microseconds > periodMicroseconds) microseconds = periodMicroseconds;
    
    float percent = (microseconds * 100.0f) / periodMicroseconds;
    write(percent);
}

float STMPWMTimer::read() {
    if(!isAttached || !sharedTimer) return 0.0f;
    return sharedTimer->getCaptureCompare(channel, PERCENT_COMPARE_FORMAT);
}

float STMPWMTimer::readNormalized() {
    return read() / 100.0f;
}

uint32_t STMPWMTimer::readRaw() {
    if(!isAttached || !sharedTimer) return 0;
    return sharedTimer->getCaptureCompare(channel, TICK_COMPARE_FORMAT);
}

void STMPWMTimer::stop() {
    write(0.0f);
}

uint32_t STMPWMTimer::getFrequency() {
    if(!isAttached || !timerInstance || timerRegistry.find(timerInstance) == timerRegistry.end()) {
        return 0;
    }
    return timerRegistry[timerInstance].frequency;
}

uint32_t STMPWMTimer::getResolution() {
    if(!isAttached || !timerInstance || timerRegistry.find(timerInstance) == timerRegistry.end()) {
        return 0;
    }
    return timerRegistry[timerInstance].resolution;
}

uint32_t STMPWMTimer::getPeriodMicroseconds() {
    uint32_t freq = getFrequency();
    if(freq == 0) return 0;
    return 1000000 / freq;
}

bool STMPWMTimer::setFrequency(uint32_t freq) {
    if(!isAttached || !sharedTimer || !timerInstance) return false;
    
    TimerData& timerData = timerRegistry[timerInstance];
    
    // Check if other channels are using this timer
    if(timerData.usedChannels.size() > 1) {
        Serial.println("[STMPWMTimer] Cannot change frequency - other channels are using this timer!");
        return false;
    }
    
    // Change frequency
    sharedTimer->pause();
    sharedTimer->setOverflow(freq, HERTZ_FORMAT);
    timerData.frequency = freq;
    timerData.resolution = sharedTimer->getOverflow(TICK_FORMAT) + 1;
    sharedTimer->resume();
    
    desiredFrequency = freq;
    
    Serial.print("[STMPWMTimer] Changed frequency to ");
    Serial.print(freq);
    Serial.println("Hz");
    
    return true;
}

// ========== STATIC METHODS ==========
uint8_t STMPWMTimer::getTimerCount() {
    return timerRegistry.size();
}

uint8_t STMPWMTimer::getAttachedPinCount() {
    return pinToInstanceMap.size();
}

void STMPWMTimer::debug() {
    Serial.println("\n=== STMPWMTimer Debug Information ===");
    Serial.print("Active timers: ");
    Serial.println(getTimerCount());
    Serial.print("Attached pins: ");
    Serial.println(getAttachedPinCount());
    
    for(auto& entry : timerRegistry) {
        TIM_TypeDef* timer = entry.first;
        TimerData& data = entry.second;
        
        Serial.print("\nTimer @ 0x");
        Serial.print((uint32_t)timer, HEX);
        
        // Try to identify the timer
        if(timer == TIM1) Serial.print(" (TIM1)");
        else if(timer == TIM2) Serial.print(" (TIM2)");
        else if(timer == TIM3) Serial.print(" (TIM3)");
        else if(timer == TIM4) Serial.print(" (TIM4)");
        else if(timer == TIM5) Serial.print(" (TIM5)");
        #ifdef TIM8
        else if(timer == TIM8) Serial.print(" (TIM8)");
        #endif
        #ifdef TIM9
        else if(timer == TIM9) Serial.print(" (TIM9)");
        #endif
        #ifdef TIM10
        else if(timer == TIM10) Serial.print(" (TIM10)");
        #endif
        #ifdef TIM11
        else if(timer == TIM11) Serial.print(" (TIM11)");
        #endif
        
        Serial.print(": ");
        Serial.print(data.frequency);
        Serial.print("Hz, Resolution: 1/");
        Serial.print(data.resolution);
        Serial.print(", Channels used: ");
        
        for(uint32_t ch : data.usedChannels) {
            Serial.print("CH");
            Serial.print(ch);
            Serial.print(" ");
        }
        
        // Find which pins are using these channels
        Serial.print("\n  Pins: ");
        for(auto& pinEntry : pinToInstanceMap) {
            if(pinEntry.second->timerInstance == timer) {
                Serial.print("PA");
                Serial.print(pinEntry.first);
                Serial.print("(CH");
                Serial.print(pinEntry.second->channel);
                Serial.print(") ");
            }
        }
    }
    
    Serial.println("\n====================================\n");
}

void STMPWMTimer::detachAll() {
    Serial.println("[STMPWMTimer] Detaching all pins...");
    
    // Create a copy since detach() modifies the map
    auto pinMapCopy = pinToInstanceMap;
    
    for(auto& entry : pinMapCopy) {
        if(entry.second) {
            entry.second->detach();
        }
    }
    
    // Clear registry
    timerRegistry.clear();
    pinToInstanceMap.clear();
    
    Serial.println("[STMPWMTimer] All pins detached and timers freed.");
}

bool STMPWMTimer::pinSupportsPWM(uint16_t pin) {
    PinName pinName = digitalPinToPinName(pin);
    uint32_t function = pinmap_function(pinName, PinMap_PWM);
    return function != NC;
}

bool STMPWMTimer::getTimerInfo(uint16_t pin, TIM_TypeDef* &timerOut, uint32_t &channelOut) {
    PinName pinName = digitalPinToPinName(pin);
    uint32_t function = pinmap_function(pinName, PinMap_PWM);
    
    if(function == NC) return false;
    
    timerOut = (TIM_TypeDef*)pinmap_peripheral(pinName, PinMap_PWM);
    channelOut = STM_PIN_CHANNEL(function);
    
    return true;
}
