#include <Arduino.h>

// --- 1. Configuration ---
#define ENCODER_PIN_1 25 
#define ENCODER_PIN_2 34 
#define ENCODER_PIN_3 35 
#define ENCODER_PIN_4 32 

// This is our "knob" for the filter speed.
// 16 bits * 50µs = 800µs total filter time.
#define POLLING_INTERVAL_MICROS (50UL) 

#define TOTAL_ENCODERS 4
struct EncoderState {
  const int pin;
  volatile long ticks;
  uint16_t readings;   // 16-bit history
  uint16_t transition; // The 16-bit pattern we're looking for
  // No 'pollAtMicros' needed! The timer handles the timing.
};

static EncoderState encoders[TOTAL_ENCODERS] = 
{
  {ENCODER_PIN_1, 0L, 0, 0},
  {ENCODER_PIN_2, 0L, 0, 0},
  {ENCODER_PIN_3, 0L, 0, 0},
  {ENCODER_PIN_4, 0L, 0, 0}
};

hw_timer_t *encoderPollingTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR pollAllEncodersISR() {
  for(uint8_t i = 0; i < TOTAL_ENCODERS; i++)
  {
    encoders[i].readings = (encoders[i].readings << 1) | digitalRead(encoders[i].pin);

    // Check for a stable match
    if (encoders[i].readings == encoders[i].transition) {
      portENTER_CRITICAL_ISR(&timerMux);
      encoders[i].ticks += 1; // Count the tick
      portEXIT_CRITICAL_ISR(&timerMux);
      encoders[i].transition = ~encoders[i].transition; // Invert the target
    }
    
    // Set the time for the next poll
  }    
}

void initAllEncoders()
{
  for(uint8_t i = 0; i < TOTAL_ENCODERS; i++)
  {
    pinMode(encoders[i].pin, INPUT);

    int initialState = digitalRead(encoders[i].pin);
    if(initialState == HIGH)
    {
      Serial.printf("Initial state for encoder[%i]: HIGH\n", i+1);
      encoders[i].readings = 0xFFFF;
      encoders[i].transition = 0xFFFE;
    } else {
      Serial.printf("Initial state for encoder[%i]: LOW\n", i+1);
      encoders[i].readings = 0x0000;
      encoders[i].transition = 0x0001;
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Timer-based Encoder poller");
  initAllEncoders();

// --- Timer Configuration ---
  
  // 1. Initialize timer 0.
  // The prescaler 80 divides the 80MHz base clock to 1MHz.
  // This means our timer ticks once every 1 microsecond.
  encoderPollingTimer = timerBegin(80); // (Timer num 0, prescaler 80, countUp true)
  encoderPollingTimer = timerBegin(80);

  timerAttachInterrupt(encoderPollingTimer, &pollAllEncodersISR);// (timer, ISR, edge-triggered=true)
  
  // 3. Set the timer to "alarm" (fire the ISR) every 50 ticks (50µs)
  // The 'true' means it will auto-reload and repeat forever.
  timerAlarm(encoderPollingTimer, POLLING_INTERVAL_MICROS, true, 0);

  timerStart(encoderPollingTimer);

}

void loop() {
  // put your main code here, to run repeatedly:
  handleSerialCommands();
}


void printEncoderTicks(EncoderState sensor)
{
  // Serial.print("Encoder Ticks: ");
  Serial.println(sensor.ticks);
}

void printAllEncoderTicks()
{
  for(uint8_t i = 0; i < TOTAL_ENCODERS; i++)
  {
    long currentTicks;
    portENTER_CRITICAL(&timerMux);
    currentTicks = encoders[i].ticks;
    portEXIT_CRITICAL(&timerMux);
    Serial.printf("Encoder %i ticks is ", i+1, currentTicks);
  }
}

void resetEncoder(EncoderState *sensor)
{
  sensor->ticks = 0;
  Serial.println("Encoder Ticks Reset.");
}

void resetAllEncoder()
{
  for(uint8_t i = 0; i < TOTAL_ENCODERS; i++)
  {
    portENTER_CRITICAL(&timerMux);
    encoders[i].ticks = 0;
    portEXIT_CRITICAL(&timerMux);
  Serial.println("All Encoder Ticks Reset.");
  }
}

void handleSerialCommands()
{
  if(Serial.available() > 0)
  {
    char cmd = Serial.read();

    if(cmd == 'p')
    {
      printAllEncoderTicks();
    }
    if (cmd == 'r')
    {
      resetAllEncoder();
    }
  }
}

