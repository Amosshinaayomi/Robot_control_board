// ==== 1. INCLUDES AND GLOBAL OBJECTS ====
#include "AHRS.h"
#include <task.h>
#include <EncoderManager.h>
#include "MotionController.h"


// Stack overflow hook
extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    Serial.print("STACK OVERFLOW in ");
    Serial.println(pcTaskName);
    while (1);
}

// Create encoder manager instance
EncoderManager encoderManager;
AHRS ahrs(SDA_PIN, SCL_PIN, 4000000UL);
MotionController motionController;

ahrsPacket_t ahrsData;
SemaphoreHandle_t ahrsMutex;   // protects ahrsData


// Task handles for debugging
TaskHandle_t ahrsHandle = NULL;
TaskHandle_t motorHandle = NULL;
TaskHandle_t motionHandle = NULL;
TaskHandle_t printHandle = NULL;
TaskHandle_t motionControlHandle = NULL;

long currentMillis;
int switchInterval = 1000;
uint8_t currentState = 0;
uint8_t motionStates = 3;


uint8_t directionState;
float motor_speed = 20;
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

// Motion Control Task: runs at 50 Hz (20 ms)
void motionControlTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20); // 50 Hz

    // Example: start going straight at 0.5 m/s after 2 seconds
    vTaskDelay(pdMS_TO_TICKS(2000));
    motionController.setStraight(0.5f);


    // Moving forward at 0.5m/s linear velocity and 3 angular vel

    // motionController.setTargetVelocity(0.5f, 3.0f); 
    for (;;) {
        // Serial.println("motion controller is running");
        // Grab latest sensor data
        ahrsPacket_t localData;
        xSemaphoreTake(ahrsMutex, portMAX_DELAY);
        localData = ahrsData;
        xSemaphoreGive(ahrsMutex);

        // Compute average ticks for left and right sides
        // encoderManager.printAllTicks();
        printAHRSPacket(localData);
        float leftTicksAvg = (localData.encoder_ticks[0] + localData.encoder_ticks[2]) / 2.0f;
        float rightTicksAvg = (localData.encoder_ticks[1] + localData.encoder_ticks[3]) / 2.0f;
        Serial.printf("lefTicksAvg is %.f\n", leftTicksAvg);
        Serial.printf("rightTicksAvg is %.f\n", rightTicksAvg);
        // Update motion controller
        motionController.update(leftTicksAvg, rightTicksAvg, localData.yaw, 0.02f);

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
    if (motor_speed < 20) motor_speed = 20;
    
    if(millis() - lastPrint >= 28) {

      if (directionState == 0) analog_move_f(motor_speed);
      else if (directionState == 1) analog_move_b(motor_speed);
      else if (directionState == 2) analog_turn_l(motor_speed);
      else if (directionState == 3) analog_turn_r(motor_speed);
      else if (directionState == 4) analog_move_f(MAX_MOTOR_VOLTAGE/batteryVoltage * 100);

      motor_speed = fmodf((motor_speed + 1), (MAX_MOTOR_VOLTAGE/batteryVoltage * 100));
      Serial.println(motorA.read());
      lastPrint = millis();
    }

    if(millis() - motorChangeMillis >= 2000)
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
    ahrsPacket_t localData;
    localData.timestamp_ms = millis();
    // orientation
    localData.roll = ahrs.getRoll();
    localData.roll = ahrs.getRoll();
    localData.pitch = ahrs.getPitch();
    localData.yaw = ahrs.getYaw();

    // Encoder ticks (read atomically)
    noInterrupts();
    localData.encoder_ticks[0] = encoderManager.getTicks(0);
    // Serial.printf("encoder 0 is %i\n", localData.encoder_ticks[0]);
    localData.encoder_ticks[1] = encoderManager.getTicks(1);
    // Serial.printf("encoder 1 is %i\n", localData.encoder_ticks[1]);
    localData.encoder_ticks[2] = encoderManager.getTicks(2);
    // Serial.printf("encoder 2 is %i\n", localData.encoder_ticks[2]);
    localData.encoder_ticks[3] = encoderManager.getTicks(3);
    // Serial.printf("encoder 3 is %i\n", localData.encoder_ticks[3]);
    interrupts();
    // Accelerometer and gyro
    ahrs.getAccel(localData.accel_g); 
    ahrs.getGyro(localData.gyro_dps);   

    // Publish to shared variable with mutex
    xSemaphoreTake(ahrsMutex, portMAX_DELAY);
    ahrsData = localData;
    xSemaphoreGive(ahrsMutex);
    vTaskDelayUntil(&lastWake, period);  
  }
}

void printTask(void *pvParameters)
{
  
  for(;;)
  {
    // Serial.printf("AHRS task runs %i per second\n", ahrsTaskHertCount);
    // Serial.printf("Motor task runs %i per second\n", motorTaskHertCount);
    ahrsPacket_t localData;
    xSemaphoreTake(ahrsMutex, portMAX_DELAY);
    localData = ahrsData;
    xSemaphoreGive(ahrsMutex);
    printAHRSPacket(localData);
    // Serial.print("Yaw: "); Serial.print(localData.yaw);
    // Serial.print("  Left ticks: "); Serial.print(localData.encoder_ticks[0] + localData.encoder_ticks[2]);
    // Serial.print("  Right ticks: "); Serial.println(localData.encoder_ticks[1] + localData.encoder_ticks[3]);  
    ahrsTaskHertCount = motorTaskHertCount = 0;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    ahrsMutex = xSemaphoreCreateMutex();
    if (ahrsMutex == NULL) {
        Serial.println("Failed to create mutex");
        while (1);
    }

    // while(!Serial){}

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
    if(encoderManager.begin(7000, TIM2)) {
      Serial.println("Encoder manager started successfully");
    } else {
      Serial.println("Failed to start encoder manager");
    }
    encoderManager.printDebugInfo();  

  xTaskCreate(ahrsTask, "AHRS", 1024, NULL, 3, &ahrsHandle);
  xTaskCreate(motionSensorTask, "Motion", 512, NULL, 2, &motionHandle); // priority 2
  // xTaskCreate(motorTask, "Motor", 512, NULL, 2, &motorHandle);
  xTaskCreate(motionControlTask, "MotionControl", 512, NULL, 2, &motionControlHandle);
  xTaskCreate(printTask, "Print", 256, NULL, 1, &printHandle);
  pinMode(LED_BUILTIN, OUTPUT);
  if (ahrsHandle == NULL) {
      Serial.println("AHRS task creation failed!");
      while(1);
  }

  delay(3000);

  Serial.println("All tasks created, starting scheduler...");
  vTaskStartScheduler();

}


void loop() {
  
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