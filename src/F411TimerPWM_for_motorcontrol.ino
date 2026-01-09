class F411PWMTimer {
  private: 
    HardwareTimer* timer;
    uint32_t channel;
    uint16_t pin;
    uint32_t frequency;
    uint32_t maxPeriod;
  public:
    /**
   * Constructor
   * @param pwmPin: Arduino pin with PWM capability (e.g., PA8)
   * @param freq: PWM frequency in Hz (default 20000 for motors)
   */
    TIM_TypeDef *TIM_Instance;
   F411PWMTimer(uint16_t pwmPin, uint32_t freq = 20000) : pin(pwmPin), frequency(freq) {
    timer = nullptr;
    channel = 0;
    maxPeriod = 0;
   }

  /**
    *Intitalize the PWM controller
    @return true if successful, false if pin doesn't support PWM
  **/
  bool begin()
  {
    // Get timer information for the pin
    PinName pinName = digitalPinToPinName(pin);
    uint32_t function = pinmap_function(pinName, PinMap_PWM);

    if(function == NC)
    {
      return false; //Pin doesn't support PWM
    }

    TIM_Instance = (TIM_TypeDef *)pinmap_peripheral(pinName, PinMap_PWM);
    channel = STM_PIN_CHANNEL(function);

    // create timer instance
    timer = new HardwareTimer(TIM_Instance);
    // Configure HardwareTimer TIM_instance
    timer->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, pin);
    timer->setOverflow(frequency, HERTZ_FORMAT);

    // Get the actual period for duty cycle calculations
    maxPeriod = timer->getOverflow(TICK_FORMAT) + 1;

    // Start with 0% duty cycle
    writeDutyCycle(0);

    // Enable preload for smooth updates
    timer->setPreloadEnable(true);
    // Start the timer
    timer->resume();
    return true;
  }

  /**
   * Set motor speed (duty cycle)
   * @param percent: 0 to 100 (0% = off, 100% = full speed)
   */
  void writeDutyCycle(float percent) {
    if(percent < 0) percent = 0;
    if(percent > 100) percent = 100;

    // Use PERCENT_COMPARE_FORMAT for simplicity
    timer->setCaptureCompare(channel, percent, PERCENT_COMPARE_FORMAT);
  }

  /**
   * Set motor speed with finer control (0.0 to 1.0)
   * @param speed: 0.0 to 1.0 (0.0 = off, 1.0 = full speed)
   */
  void writeDutyCycleNormalized(float speed) {
    writeDutyCycle(speed * 100.0f);
  }


  uint16_t readDutyCycle()
  {
    return timer->getCaptureCompare(channel, PERCENT_COMPARE_FORMAT);
  }
  /**
   * Stop the motor (0% duty cycle)
   */
  void stop() {
    writeDutyCycle(0);
  }

  /**
   * Get the actual PWM frequency
   * @return frequency in Hz
   */
   uint32_t getFrequency() {
    uint32_t timerClock = timer->getTimerClkFreq();
    uint32_t prescaler =  TIM_Instance->PSC;
    uint32_t arr = TIM_Instance->ARR;
    return timerClock / ((prescaler + 1) * (arr + 1));
   }  

  /**
   * Get the timer resolution (number of steps from 0% to 100%)
   * @return resolution in steps
   */
  uint32_t getResolution() {
    return timer->getOverflow(TICK_FORMAT) + 1;
  }
  
  /**
   * Get the HardwareTimer TIM_instance for advanced configuration
   * @return pointer to HardwareTimer
   */
  HardwareTimer* getTimer() {
    return timer;
  }
};


F411PWMTimer motor1(PA10, 20000);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial)
  {
    delay(10);
  }
  if(!motor1.begin()) {
    Serial.println("Failed to initialize Hardware PWM for motor");
    while(1);
  }

  Serial.print("Motor PWM Frequency: ");
  Serial.print(motor1.getFrequency());
  Serial.println(" Hz");
  
  Serial.print("Resolution: 1/");
  Serial.println(motor1.getResolution());


}

void loop() {
  // put your main code here, to run repeatedly:

    // Ramp up
  for (int i = 0; i <= 100; i += 10) {
    motor1.writeDutyCycle(i);
    int measured_pwm = motor1.readDutyCycle();
    Serial.print("raw compare value: "); Serial.println(measured_pwm);
    float predicted_voltage = measured_pwm / 100.00 * 3.3;
    Serial.print("predicted voltage: "); Serial.print(measured_pwm / 100.00 * 3.3); Serial.println("v");
    delay(500);
  }
  
  delay(1000);
  
  // Ramp down
  for (int i = 100; i >= 0; i -= 10) {
    motor1.writeDutyCycle(i);
    int measured_pwm = motor1.readDutyCycle();
    Serial.print("raw compare value: "); Serial.println(measured_pwm);
    float predicted_voltage = measured_pwm / 100.00 * 3.3;
    Serial.print("predicted voltage"); Serial.print(measured_pwm / 100.00 * 3.3); Serial.println("v");
    delay(500);
  }
}
