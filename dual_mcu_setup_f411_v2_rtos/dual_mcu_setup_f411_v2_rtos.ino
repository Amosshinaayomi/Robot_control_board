// ==== 1. INCLUDES AND GLOBAL OBJECTS ====
#include "AHRS.h"
#include <task.h>
#include <EncoderManager.h>
#include <motor_control.h>

// Stack overflow hook
extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    Serial.print("STACK OVERFLOW in ");
    Serial.println(pcTaskName);
    while (1);
}

// Create encoder manager instance
EncoderManager encoderManager;
AHRS ahrs(SDA_PIN, SCL_PIN, 4000000UL);
ahrsPacket_t ahrsData;

// Task handles for debugging
TaskHandle_t ahrsHandle = NULL;
TaskHandle_t motorHandle = NULL;
TaskHandle_t motionHandle = NULL;
TaskHandle_t printHandle = NULL;


long currentMillis;
int switchInterval = 1000;
uint8_t currentState = 0;
uint8_t motionStates = 3;


uint8_t directionState;
uint8_t motor_speed = 20;
long motorChangeMillis;
int ahrsTaskHertCount  = 0;
int motorTaskHertCount = 0;




void ahrsTask(void *pvParameters) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(2.5); 
  for(;;) {
    digitalWrite(LED_BUILTIN, HIGH);
    ahrs.update();
    ahrsTaskHertCount++;
    vTaskDelayUntil(&lastWake, period);  
     
  }

}

void motorTask(void *pvParameters)
{
  Serial.println("Motor task started");
  static unsigned long lastPrint = 0; 
  currentMillis = millis();
  motorChangeMillis = millis();

  TickType_t lastWake = xTaskGetTickCount();

  for(;;) {
    if(millis() - lastPrint >= 30) {

      if (directionState == 0) analog_move_f(motor_speed);
      else if (directionState == 1) analog_move_b(motor_speed);
      else if (directionState == 2) analog_turn_l(motor_speed);
      else if (directionState == 3) analog_turn_r(motor_speed);
      else if (directionState == 4) analog_move_f(20);
      motor_speed = (motor_speed + 1) % 110;
      if (motor_speed < 20) motor_speed = 20;
      lastPrint = millis();
    }

    if(millis() - motorChangeMillis >= 2700)
    {
      directionState++;
      directionState = (directionState + 1) % 5;
      motorChangeMillis = millis();
    }
    
    motorTaskHertCount++;
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(50));
  }
}

void motionSensorTask(void *pvParameters) 
{
  Serial.println("Motion sensor task started");
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(5); 
  for(;;)
  {
    // Serial.println("sensor task");
    // time stamp
    ahrsData.timestamp_ms = millis();
    // orientation
    ahrsData.roll = ahrs.getRoll();
    // Serial.println("  motion: getting roll");
    ahrsData.roll = ahrs.getRoll();
    // Serial.println("  motion: got roll, getting pitch");
    ahrsData.pitch = ahrs.getPitch();
    // Serial.println("  motion: got pitch, getting yaw");
    ahrsData.yaw = ahrs.getYaw();

    // Encoder ticks (read atomically)
    noInterrupts();
    ahrsData.encoder_ticks[0] = encoderManager.getTicks(0);
    ahrsData.encoder_ticks[1] = encoderManager.getTicks(1);
    ahrsData.encoder_ticks[2] = encoderManager.getTicks(2);
    ahrsData.encoder_ticks[3] = encoderManager.getTicks(3);
    interrupts();
    // Accelerometer and gyro
    ahrs.getAccel(ahrsData.accel_g); 
    ahrs.getGyro(ahrsData.gyro_dps);    
    vTaskDelayUntil(&lastWake, period);  
  }
}

void printTask(void *pvParameters)
{
  
  for(;;)
  {
    // Serial.printf("AHRS task runs %i per second\n", ahrsTaskHertCount);
    // Serial.printf("Motor task runs %i per second\n", motorTaskHertCount);
    printIMUPacket(ahrsData);
    ahrsTaskHertCount = motorTaskHertCount = 0;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);

    while(!Serial){}

    // Initialize AHRS (IMU + Magnetometer) 
    while(!ahrs.begin()) {
      Serial.println("AHRS initialization failed! Halt.");
      while (true);
    }
    Serial.println("IMU and Fusion Library Initialized.");

    // Attach all motors (automatically manages timer sharing!)
    bool initMotors = initMotorDrivers();
    if(!initMotors)
    {
      Serial.println("Motor driver initialization failed");
      while(1){}
    }

    // Setup encoders
    encoderManager.addEncoder(ENCODER_PIN_1);
    encoderManager.addEncoder(ENCODER_PIN_2);
    encoderManager.addEncoder(ENCODER_PIN_3);
    encoderManager.addEncoder(ENCODER_PIN_4);
    // Debug info    
    STMPWMTimer::debug();

    // Start polling at 100kHz (100µs intervals)
    if(encoderManager.begin(5000, TIM2)) {
      Serial.println("Encoder manager started successfully");
    } else {
      Serial.println("Failed to start encoder manager");
    }
    encoderManager.printDebugInfo();  

  xTaskCreate(ahrsTask, "AHRS", 1024, NULL, 3, &ahrsHandle);
  xTaskCreate(motionSensorTask, "Motion", 512, NULL, 2, &motionHandle); // priority 2
  xTaskCreate(motorTask, "Motor", 512, NULL, 2, &motorHandle);
  xTaskCreate(printTask, "Print", 256, NULL, 1, &printHandle);
  pinMode(LED_BUILTIN, OUTPUT);
  if (ahrsHandle == NULL) {
      Serial.println("AHRS task creation failed!");
      while(1);
  }

  Serial.println("All tasks created, starting scheduler...");
  vTaskStartScheduler();

}


void loop() {
}


void printIMUPacket(ahrsPacket_t data)
{
    Serial.print("Time stamp: "); Serial.println(data.timestamp_ms);
    Serial.print("Gyro (dps) X,Y,Z: ");
    Serial.print(data.gyro_dps[0], 3); Serial.print(", ");
    Serial.print(data.gyro_dps[1], 3); Serial.print(", ");
    Serial.println(data.gyro_dps[2], 3);
    Serial.print("Accel(g) X,Y,Z: ");
    Serial.print(data.accel_g[0], 3); Serial.print(", ");
    Serial.print(data.accel_g[1], 3); Serial.print(", ");
    Serial.println(data.accel_g[2], 3);

    Serial.print("PITCH:"); Serial.print(data.pitch);
    Serial.print(", ROLL:"); Serial.print(data.roll);
    Serial.print(", YAW:"); Serial.println(data.yaw); 

    Serial.printf("Front Left side encoder tick is %i\nFront right side encoder tick is %i\nBack Left side encoder tick is %i\nBack right side encoder tick is %i\n",data.encoder_ticks[0], data.encoder_ticks[1], data.encoder_ticks[2], data.encoder_ticks[3]);  
    Serial.println();    
}

void readModeFromSerial()
{
  if(Serial.available() > 0)
  {
    String serialInput = Serial.readStringUntil('\n');
    char CHAR = serialInput[0];
    CHAR = toupper(CHAR);
    if(CHAR == 'F' || CHAR == 'B' || CHAR == 'L' || CHAR == 'R' || CHAR == 'N')
    {
      // controlMode = CHAR;   
      uint8_t speed = (serialInput.substring(1)).toInt();
      Serial.printf("speed: %i\n", speed);
      return;
    }
    Serial.println("Please Enter the Character F or B or L or R to control the motors.");
  }
}


void sendVisualizationData(ahrsPacket_t data)
{
    Serial.print("PITCH:"); Serial.print(data.pitch);
    Serial.print(",ROLL:"); Serial.print(data.roll);
    Serial.print(",YAW:"); Serial.println(data.yaw);       
}