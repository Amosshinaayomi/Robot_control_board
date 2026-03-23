// ==== 1. INCLUDES AND GLOBAL OBJECTS ====
#include "AHRS.h"
#include <task.h>
#include <EncoderManager.h>
#include "MotionController.h"
#include "communications.h"
#include "comm_protocol.h"
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

bool systemRunning = false;              // controls main tasks
SemaphoreHandle_t startSemaphore = NULL; // optionally, for tasks to wait

static bool hardwareInitialized = false;

// Task handles for debugging
TaskHandle_t ahrsHandle = NULL;
TaskHandle_t motorHandle = NULL;
TaskHandle_t motionHandle = NULL;
TaskHandle_t printHandle = NULL;
TaskHandle_t motionControlHandle = NULL;
TaskHandle_t commsHandle = NULL;


long currentMillis;
int switchInterval = 1000;
uint8_t currentState = 0;
uint8_t motionStates = 3;


uint8_t directionState;
float motor_speed = 20;
long motorChangeMillis;
int ahrsTaskHertCount  = 0;
int motorTaskHertCount = 0;

uint8_t hardwareSensorStatus = 0;

bool initAllHardware(uint8_t* errorCode);

void commsTask(void *parameter) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(20); // 50Hz

  commState rxState = WAIT_START;
  uint8_t rxBuffer[64];
  uint8_t rxIndex = 0;
  uint8_t rxLen = 0;
  uint8_t rxType = 0;


  for(;;)
  {
    while (commSerial.available()) {  
      byte b = commSerial.read();
      switch(rxState) {
      case WAIT_START: 
          if (b == PKT_START_BYTE) rxState = WAIT_TYPE;
          // Serial.println("packet start");
          break;
      case WAIT_TYPE: 
          rxType = b;
          Serial.printf("packet type is 0x%X\n", rxType);
          rxState = WAIT_LEN;
          break;

      case WAIT_LEN: 
          rxLen = b;
          rxIndex = 0;
          if(rxLen == 0)
          {
          rxState = WAIT_CHECKSUM;
          } else if(rxLen <= sizeof(rxBuffer)) {
          rxState = WAIT_PAYLOAD;
          } else {
          rxState = WAIT_START;
          }
          Serial.printf("packet length is %i\n", rxLen);
          break;
      case WAIT_PAYLOAD: 
          rxBuffer[rxIndex++] = b;
          if(rxIndex >= rxLen)
          {
              rxState = WAIT_CHECKSUM;
          }
          // Serial.println("packet payload is added to buffer");
          break;
      case WAIT_CHECKSUM: {
          byte header[3] = {PKT_START_BYTE, rxType, rxLen};
          byte calculated = compute_checksum(header, 3) ^ compute_checksum(rxBuffer, rxLen);

          if(calculated == b) {
            // Check packet type and parse data
            // check if pack is a command packet
            if(rxType == PKT_TYPE_SENSOR) {

            } 
            else if(rxType == PKT_TYPE_COMMAND) {
              // At least one command byte must be present
              if(rxLen >= 1) {
                uint8_t cmd = rxBuffer[0];
                uint8_t paramLen = rxLen - 1;
                uint8_t *params = (paramLen > 0) ? &rxBuffer[1] : NULL;


                // Handle startup and run velocity commands
                switch (cmd) {
                  case CMD_STARTUP_REQ: {
                      Serial.println("Recieved STARTUP_REQ");                    
                      if(!hardwareInitialized) {
                        sendStartupNack(hardwareSensorStatus);
                        Serial.println("hardware initialization failed  sending Nack ....");
                      } else {
                          // Already initialized – just ACK again
                          Serial.println("Hardware already initialized");   
                          sendStartupAck();
                      }
                    break;                      
                  }
                  case CMD_RUN:
                    Serial.println("Received RUN command – starting main tasks");
                    systemRunning = true;
                    break;
                  default:
                    Serial.printf("Unknown command: 0x%02X\n", cmd);
                    break;
                }
              }
            }
          } else {
              Serial.println("Checksum error – packet corrupted");
          }
          rxState = WAIT_START;
          break;
      }
      default:
          rxState = WAIT_START;
      }
    }
    if(systemRunning)
    {
      ahrsPacket_t localData;
      xSemaphoreTake(ahrsMutex, portMAX_DELAY);
      localData = ahrsData;
      xSemaphoreGive(ahrsMutex);
      // Serial.println("NORMAL AHRS data");
      // printAHRSPacket(localData);
      // Copy the local data content into the packed_struct
      ahrsPacketPacked_t txData;
      txData.timestamp_ms = localData.timestamp_ms;
      txData.roll = localData.roll;
      txData.pitch = localData.pitch;
      txData.yaw = localData.yaw;
      txData.yawRate = localData.yawRate;
      memcpy(txData.accel_g, localData.accel_g, sizeof(float)*3);
      memcpy(txData.gyro_dps, localData.gyro_dps, sizeof(float)*3);
      memcpy(txData.encoder_ticks, localData.encoder_ticks, sizeof(int32_t)*4);
      // Serial.println("SENT AHRS data");
      // printAHRSPacket(txData);
      // Build packet
      uint8_t packet[sizeof(ahrsPacketPacked_t) + 3];
      uint8_t idx = 0;
      // Packet header, Start byte
      packet[idx++] = PKT_START_BYTE;
      // Sensor packet type
      packet[idx++] = PKT_TYPE_SENSOR;
      // packet size length
      packet[idx++] = sizeof(ahrsPacketPacked_t);
      // copy stored data into packet
      memcpy(&packet[idx], &txData, sizeof(ahrsPacketPacked_t));
      // increase packet size to contain sensor data struct
      idx += sizeof(ahrsPacketPacked_t);

      // Store checksum
      packet[idx++] = compute_checksum(packet, idx);
      // Publish data
      commSerial.write(packet, idx);
      Serial.println("AHRS Data sent succesfully");
    }

    vTaskDelayUntil(&lastWake, period);
  }
}


void ahrsTask(void *pvParameters) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(2.5); 
  for(;;) {
    if (!systemRunning) {
        vTaskDelay(pdMS_TO_TICKS(100));
        continue;
    }
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
    while (!systemRunning) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    motionController.setStraight(0.5f);


    // Moving forward at 0.5m/s linear velocity and 3 angular vel
    // motionController.setTargetVelocity(0.5f, 3.0f); 
    for (;;) {
       if (!systemRunning) {
          vTaskDelay(pdMS_TO_TICKS(100));
          continue;
        }
        // Serial.println("motion controller is running");
        // Grab latest sensor data
        ahrsPacket_t localData;
        xSemaphoreTake(ahrsMutex, portMAX_DELAY);
        localData = ahrsData;
        xSemaphoreGive(ahrsMutex);

        // Compute average ticks for left and right sides
        // encoderManager.printAllTicks();
        // printAHRSPacket(localData);
        float leftTicksAvg = (localData.encoder_ticks[0] + localData.encoder_ticks[2]) / 2.0f;
        float rightTicksAvg = (localData.encoder_ticks[1] + localData.encoder_ticks[3]) / 2.0f;
        // Serial.printf("lefTicksAvg is %.f\n", leftTicksAvg);
        // Serial.printf("rightTicksAvg is %.f\n", rightTicksAvg);
        // Update motion controller
        motionController.update(leftTicksAvg, rightTicksAvg, localData.yaw, localData.yawRate, 0.02f);

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
    if(!systemRunning) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
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
    if(!systemRunning) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
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
    localData.yawRate = ahrs.getYawRate();
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
    if(!systemRunning) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    ahrsPacket_t localData;
    xSemaphoreTake(ahrsMutex, portMAX_DELAY);
    localData = ahrsData;
    xSemaphoreGive(ahrsMutex);
    // printAHRSPacket(localData);
    // Serial.print("Yaw: "); Serial.print(localData.yaw);
    // Serial.print("  Left ticks: "); Serial.print(localData.encoder_ticks[0] + localData.encoder_ticks[2]);
    // Serial.print("  Right ticks: "); Serial.println(localData.encoder_ticks[1] + localData.encoder_ticks[3]);  
    ahrsTaskHertCount = motorTaskHertCount = 0;

    // byte data[] = {0x55, 0x01, 0x02, 0x03};   
    // byte payloadHeader[3] = {PKT_START_BYTE, PKT_TYPE_SENSOR, sizeof(data)};
    // send_message(PKT_TYPE_SENSOR, sizeof(data), data);    

    
    // commSerial.write(PKT_START_BYTE);
    // commSerial.write(PKT_TYPE_SENSOR);    
    // commSerial.write(len);
    // commSerial.write(data, len);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

  }
}


void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    // Begin serial communication with the second mcu
    // while(!Serial) {delay(50);}
    commSerial.begin(115200);
    ahrsMutex = xSemaphoreCreateMutex();
    if (ahrsMutex == NULL) {
        Serial.println("Failed to create mutex");
        while (1);
    }
    // Serial.printf("hardware start millis %i\n", millis());
    if(initAllHardware(&hardwareSensorStatus)) {
      Serial.println("hardware initialized  sending startupack ....");
      hardwareInitialized = true;
    } else {
      Serial.println("hardware initialization failed  sending Nack ....");
      hardwareInitialized = false;
  }
    // Serial.printf("hardware end millis %i\n", millis());
    xTaskCreate(commsTask, "comms", 1024, NULL, 2, &commsHandle);
    Serial.println("Waiting rtup command from esp32s3");

    xTaskCreate(ahrsTask, "AHRS", 1024, NULL, 3, &ahrsHandle);
    xTaskCreate(motionSensorTask, "Motion", 512, NULL, 2, &motionHandle); // priority 2
    // xTaskCreate(motorTask, "Motor", 512, NULL, 2, &motorHandle);
    xTaskCreate(motionControlTask, "MotionControl", 1024, NULL, 2, &motionControlHandle);
    xTaskCreate(printTask, "Print", 256, NULL, 1, &printHandle);
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.println("All tasks created, starting scheduler...");
   vTaskStartScheduler();

}


void loop() {
  
}

bool initAllHardware(uint8_t* errorCode)
{
    if(!ahrs.begin()) {
      Serial.println("AHRS initialization failed! Halt.");
      *errorCode = 1;
      return false;
    }
    Serial.println("IMU and Fusion Library Initialized.");

    // Attach all motors (automatically manages timer sharing!)
    bool initMotors = initMotorDrivers();
    if(!initMotors)
    {
      Serial.println("Motor driver initialization failed");
      *errorCode = 2;
      return false;
    }
    STMPWMTimer::debug();
    // Setup encoders
    if(!encoderManager.addEncoder(ENCODER_PIN_1)) 
    {
      *errorCode = 3;
      Serial.println("encoder 1 initialization failed");
      return false;
    }
    if(!encoderManager.addEncoder(ENCODER_PIN_2)) 
    {
      *errorCode = 3;
      Serial.println("encoder  initialization failed");
      return false;
    }
    if(!encoderManager.addEncoder(ENCODER_PIN_3)) 
    {
      *errorCode = 3;
      Serial.println("encoder 3 initialization failed");
      return false;
    }
    if(!encoderManager.addEncoder(ENCODER_PIN_4)) 
    {
      *errorCode = 3;
      Serial.println("encoder 4 initialization failed");
      return false;
    }
    // Debug info    


    // Start polling at 100kHz (100µs intervals)
    if(!encoderManager.begin(7000, TIM2)) {
      Serial.println("Encoder manager failed");
      *errorCode = 3;
      return false;
    }
    encoderManager.printDebugInfo();  
    return true;
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