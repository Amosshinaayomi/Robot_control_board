// ==== 1. INCLUDES AND GLOBAL OBJECTS ====
#include "AHRS.h"
#include <task.h>
#include <EncoderManager.h>
#include "motor_control.h"
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
AHRS ahrs(SDA_PIN, SCL_PIN, 1000000UL);
MotionController motionController;



motionSensorPacket_t ahrsData;
SemaphoreHandle_t ahrsMutex;   // protects ahrsData

pwrStatus_t lastestPwrStatus;
SemaphoreHandle_t sysPwrMutex;

// Hardware system flag tasks
volatile bool systemRunning = false;              // controls main tasks
SemaphoreHandle_t startSemaphore = NULL; // optionally, for tasks to wait
static bool hardwareInitialized = false;
uint8_t hardwareSensorStatus = 0;
volatile bool calibrated = false; 
volatile bool calibrationRequested = true;
// Task handles for debugging
TaskHandle_t ahrsHandle = NULL;
TaskHandle_t motorHandle = NULL;
TaskHandle_t motionSensorHandle = NULL;
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
int motorControllerTaskHertCount = 0;
// Test mode control
volatile bool speedTestMode = false;
volatile bool speedTestRunning = false;
volatile float speedTestLeftVoltage = 0;
volatile float speedTestRightVoltage = 0;
SemaphoreHandle_t speedTestSemaphore;



bool initAllHardware(uint8_t* errorCode);
void calibrateMagnetometer2D(int duration);

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
    // Read and parse incoming packets from s3
    if (commSerial.available()) {  
      byte b = commSerial.read();
      switch(rxState) {
      case WAIT_START: 
          if (b == PKT_START_BYTE) rxState = WAIT_TYPE;
          // Serial.println("packet start");
          break;
      case WAIT_TYPE: 
          rxType = b;
          // Serial.printf("packet type is 0x%X\n", rxType);
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
          // Serial.printf("packet length is %i\n", rxLen);
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
                  case CMD_START_SPEED_TEST:
                      if (!speedTestRunning) {
                          speedTestMode = true;
                          xSemaphoreGive(speedTestSemaphore);   // trigger the test task
                          Serial.println("Test start command received");
                      }
                      break;
                  case CMD_STOP_SPEED_TEST:
                      speedTestMode = false;
                      speedTestRunning = false;
                      speedTestLeftVoltage = 0;
                      speedTestRightVoltage = 0;
                      Serial.println("Test stop command received");
                      break;
                  default:
                    Serial.printf("Unknown command: 0x%02X\n", cmd);
                    break;
                }
              }
            }
            else if (rxType == PWR_STATUS) {
              if(rxLen == sizeof(pwrStatus_t)) {
                pwrStatus_t pwr;
                memcpy(&pwr, rxBuffer, sizeof(pwrStatus_t));
               
                // Optionally store in a global with mutex
                xSemaphoreTake(sysPwrMutex, portMAX_DELAY);
                lastestPwrStatus = pwr;
                BATTERY_VOLTAGE = lastestPwrStatus.batteryVoltage;                
                xSemaphoreGive(sysPwrMutex);
              // Now you can print or use the data
              // Serial.printf("Power: timestamp=%lu, voltage=%.2f\n", lastestPwrStatus.timestamp_ms, BATTERY_VOLTAGE);
              }  else {
                  Serial.printf("PWR_STATUS payload size mismatch: got %d, expected %d\n", rxLen, sizeof(pwrStatus_t));
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
    
    // Build and send sensor data packets to s3
    if(systemRunning)
    {
      motionSensorPacket_t localData;
      xSemaphoreTake(ahrsMutex, portMAX_DELAY);
      localData = ahrsData;
      xSemaphoreGive(ahrsMutex);
      ahrsPacketPacked_t txData;
      txData.timestamp_ms = localData.timestamp_ms;
      txData.roll = localData.roll;
      txData.pitch = localData.pitch;
      txData.yaw = localData.yaw;
      txData.yawRate = localData.yawRate;
      memcpy(txData.accel_g, localData.accel_g, sizeof(float)*3);
      memcpy(txData.gyro_dps, localData.gyro_dps, sizeof(float)*3);
      memcpy(txData.encoder_ticks, localData.encoder_ticks, sizeof(int32_t)*4);

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
      // Serial.println("AHRS Data sent succesfully");
    }

    vTaskDelayUntil(&lastWake, period);
  }
}


void ahrsTask(void *pvParameters) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(2.5); 
  for(;;) {
    if (!systemRunning) {
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
    }
    if (calibrationRequested) {
        Serial.println("calibration is on going");
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
    }
    ahrs.update();
    ahrsTaskHertCount++;
    vTaskDelayUntil(&lastWake, period);  
  
  }
}

void ahrsCalibrationTask(void *pvParameters) {
    Serial.println("Calibration task started. Waiting for battery voltage...");
    // Wait for battery voltage to be valid (optional)
    calibrationRequested = true;
    vTaskDelay(10 / portTICK_PERIOD_MS);    
    while (BATTERY_VOLTAGE < 9.0f) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }


    Serial.println("Battery OK. Starting IMU calibration...");
    ahrs.calibrateIMU();           // your existing IMU calibration    
    Serial.println("IMU calibration done. Starting 2D mag calibration...");
    calibrateMagnetometer2D(13000); // your existing function
    Serial.println("Magnetometer calibration done.");
    // calibrated = true;
    Serial.println("Calibration complete. Other tasks will now start.");
    calibrationRequested = false;
    calibrated = true;
    Serial.printf("calibrationRequest is %i\n", calibrationRequested);
    Serial.printf("calibrated is %i\n", calibrated);
    vTaskDelay(10 / portTICK_PERIOD_MS);   
    // Suspend or delete itself
    vTaskSuspend(NULL);
}

// Motion Control Task: runs at 50 Hz (20 ms)
// quantization issue
void motionControllerTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(50); // 50 Hz
    CONTROL_DT = period / 1000.0;
    long prevTimestamp_ms = 0;
    float dt = 0.0f;

    motionController.setStraight(0.3f);
    // motionController.setTargetVelocity(0.3, 3);

    // Moving forward at 0.5m/s linear velocity and 3 angular vel
    // motionController.setTargetVelocity(0.4f, 3.0f); 

    for (;;) {
       if (!systemRunning) 
       {
          Serial.printf("system flag is %i, calibrated flag is %i\n", systemRunning, calibrated);
          Serial.println("waiting for calibration or system flag motioncontrollertask task");
          vTaskDelay(pdMS_TO_TICKS(10));
          continue;
       }

        // Serial.println("motion controller is running");
        // Grab latest sensor data
        motionSensorPacket_t localData;
        if (calibrationRequested) {
            Serial.println("calibration is on going");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (xSemaphoreTake(ahrsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            localData = ahrsData;
            xSemaphoreGive(ahrsMutex);
        } else {
            Serial.println("AHRS mutex timeout!");
            continue;
        }
        Serial.printf("control loop period %f\n", CONTROL_DT);
        // Compute average ticks for left and right sides
        float leftTicksAvg = (localData.encoder_ticks[0] + localData.encoder_ticks[2]) / 2.0f;
        float rightTicksAvg = (localData.encoder_ticks[1] + localData.encoder_ticks[3]) / 2.0f;
        Serial.printf("lefTicksAvg is %.2f\n", leftTicksAvg);
        Serial.printf("rightTicksAvg is %.2f\n", rightTicksAvg);
        dt = (localData.timestamp_ms - prevTimestamp_ms) /1000.0;
        
        if (dt < 0.001f) 
          dt = 0.001; // fallback to nominal
          Serial.printf("dt is lower than 0.001, dt is %.6f\n", dt);
        Serial.printf("dt is %.6f\n", dt);        
        
        float yawRad = localData.yaw * DEG_TO_RAD;
        float yawRate_dps = localData.yawRate * DEG_TO_RAD;
        prevTimestamp_ms = localData.timestamp_ms;
        // Use an Exponential Moving average filter to increase quantization steps
        static float leftTicksAvgF = 0;
        static float rightTicksAvgF = 0;


        // Update motion controller
        float motorsVolt[2];
        motionController.update(leftTicksAvg, rightTicksAvg, yawRad, yawRate_dps, dt, motorsVolt);

        Serial.printf("left motor voltage is %.3f\nright motor voltage is %.3f\n", motorsVolt[0], motorsVolt[1]);
 
        setLeftMotorsVoltage(motorsVolt[0]);
        setRightMotorsVoltage(motorsVolt[1]); 
        bool leftMotorDir = (motorsVolt[0] >= 0);   
        bool rightMotorDir = (motorsVolt[1] >= 0);           
        // Serial.printf("left motor direction is %i\nright motor direction is %i\n", leftMotorDir, rightMotorDir);   
        encoderManager.setDirection(0, leftMotorDir);
        encoderManager.setDirection(2, leftMotorDir);
        encoderManager.setDirection(1, rightMotorDir);
        encoderManager.setDirection(3, rightMotorDir);

        motorControllerTaskHertCount++;

        Serial.println();
        Serial.println();
        Serial.println();
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
        Serial.printf("system flag is %i, calibrated flag is %i\n", systemRunning, calibrated);
        Serial.println("waiting for calibration or system flag motorTask task");
      continue;
    }
    if (calibrationRequested) {
        Serial.println("calibration is on going");
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
    }
    // setLeftMotorsVoltage(6);
    // setRightMotorsVoltage(6); 

    if(BATTERY_VOLTAGE >= 9.0)
    {
      if (speedTestMode) {
          // Apply test voltages (open loop)
        bool leftMotorDir = (speedTestLeftVoltage >= 0);   
        bool rightMotorDir = (speedTestRightVoltage >= 0);           
        // Serial.printf("left motor direction is %i\nright motor direction is %i\n", leftMotorDir, rightMotorDir);   
        setLeftMotorsVoltage(speedTestLeftVoltage);
        setRightMotorsVoltage(speedTestRightVoltage);        
        encoderManager.setDirection(0, leftMotorDir);
        encoderManager.setDirection(1, leftMotorDir);
        encoderManager.setDirection(2, rightMotorDir);
        encoderManager.setDirection(3, rightMotorDir);
      } else {
          // Normal operation: PID outputs or stop
          // (You can put your normal control code here, or just set to 0)
        float voltage = 0;
        bool leftMotorDir = (voltage >= 0);   
        bool rightMotorDir = (-voltage >= 0);           
        // Serial.printf("left motor direction is %i\nright motor direction is %i\n", leftMotorDir, rightMotorDir);   
        setLeftMotorsVoltage(voltage);
        setRightMotorsVoltage(-voltage);        
        encoderManager.setDirection(0, leftMotorDir);
        encoderManager.setDirection(2, leftMotorDir);
        encoderManager.setDirection(1, rightMotorDir);
        encoderManager.setDirection(3, rightMotorDir);
      }     
    }
    motorTaskHertCount++;
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(50));
  }
}

void speedTestTask(void *pvParameters) {
    const float voltageSteps[] = {2, 3, 4, 5, 6, 7, 8, 9};
    const int settleTimeMs = 2000;
    const int numSteps = sizeof(voltageSteps)/sizeof(voltageSteps[0]);
    // Example: start going straight at 0.5 m/s after 2 seconds

    for (;;) {
        if (!systemRunning) {
          vTaskDelay(pdMS_TO_TICKS(10));
            Serial.printf("system flag is %i, calibrated flag is %i\n", systemRunning, calibrated);
            Serial.println("waiting for calibration or system flag speed task");
          continue;
        }    
        if (calibrationRequested) {
            Serial.println("calibration is on going");
          vTaskDelay(pdMS_TO_TICKS(10));
          continue;
        }  
        // Wait for start command
        xSemaphoreTake(speedTestSemaphore, portMAX_DELAY);

        if (speedTestMode) {
            speedTestRunning = true;
            Serial.println("TEST_START");

            speed_test_log_t log;

            // ----- Left motor only -----
            // for (uint8_t i = 0; i < numSteps; i++) {
            //     speedTestLeftVoltage = voltageSteps[i];
            //     speedTestRightVoltage = 0;
            //     Serial.printf("SET,LEFT,%.2f\n", speedTestLeftVoltage);
                

                
            //     int32_t initLeft = (encoderManager.getTicks(0) + encoderManager.getTicks(2)) / 2;
            //     int32_t initRight = (encoderManager.getTicks(1) + encoderManager.getTicks(3)) / 2;
            //     interrupts();

            //     vTaskDelay(pdMS_TO_TICKS(settleTimeMs));

                
            //     int32_t finalLeft = (encoderManager.getTicks(0) + encoderManager.getTicks(2)) / 2;
            //     int32_t finalRight = (encoderManager.getTicks(1) + encoderManager.getTicks(3)) / 2;
            //     interrupts();

            //     float leftSpeed = (finalLeft - initLeft) / (settleTimeMs / 1000.0f);
            //     float rightSpeed = (finalRight - initRight) / (settleTimeMs / 1000.0f);


            //     log.timestamp_ms = millis();
            //     log.leftVoltage = speedTestLeftVoltage;
            //     log.rightVoltage = speedTestRightVoltage;
            //     log.leftSpeed = leftSpeed;
            //     log.rightSpeed = rightSpeed;


            //     // Serial.printf("RESULT,%.2f,%.2f,%.2f,%.2f\n",
            //     //     speedTestLeftVoltage, speedTestRightVoltage,
            //     //     leftSpeed, rightSpeed);

            //     Serial.printf("RESULT,%.2f,%.2f,%.2f,%.2f\n", 
            //     log.leftVoltage, log.rightVoltage,
            //     log.leftSpeed, log.rightSpeed);   

            //     send_log_packet(LOG_TYPE_SPEED_TEST, &log, sizeof(log));

            // }

            // // ----- Right motor only -----
            // for (uint8_t i = 0; i < numSteps; i++) {
            //     speedTestLeftVoltage = 0;
            //     speedTestRightVoltage = voltageSteps[i];
            //     Serial.printf("SET,RIGHT,%.2f\n", speedTestRightVoltage);

            //     vTaskDelay(pdMS_TO_TICKS(100));

                
            //     int32_t initLeft = (encoderManager.getTicks(0) + encoderManager.getTicks(2)) / 2;
            //     int32_t initRight = (encoderManager.getTicks(1) + encoderManager.getTicks(3)) / 2;
            //     interrupts();

            //     vTaskDelay(pdMS_TO_TICKS(settleTimeMs));

                
            //     int32_t finalLeft = (encoderManager.getTicks(0) + encoderManager.getTicks(2)) / 2;
            //     int32_t finalRight = (encoderManager.getTicks(1) + encoderManager.getTicks(3)) / 2;
            //     interrupts();

            //     float leftSpeed = (finalLeft - initLeft) / (settleTimeMs / 1000.0f);
            //     float rightSpeed = (finalRight - initRight) / (settleTimeMs / 1000.0f);

            //     log.timestamp_ms = millis();
            //     log.leftVoltage = speedTestLeftVoltage;
            //     log.rightVoltage = speedTestRightVoltage;
            //     log.leftSpeed = leftSpeed;
            //     log.rightSpeed = rightSpeed;


            //     // Serial.printf("RESULT,%.2f,%.2f,%.2f,%.2f\n",
            //     //     speedTestLeftVoltage, speedTestRightVoltage,
            //     //     leftSpeed, rightSpeed);

            //     Serial.printf("RESULT,%.2f,%.2f,%.2f,%.2f\n", 
            //     log.leftVoltage, log.rightVoltage,
            //     log.leftSpeed, log.rightSpeed);   

            //     send_log_packet(LOG_TYPE_SPEED_TEST, &log, sizeof(log));
            // }

            // ----- Both motors -----
            for (uint8_t i = 0; i < numSteps; i++) {
                speedTestLeftVoltage = voltageSteps[i];
                speedTestRightVoltage = voltageSteps[i];
                Serial.printf("SET,BOTH,%.2f\n", speedTestLeftVoltage);
                // Data capture the beginning of the steady‑state interval
                float yaw_start = 0;
                unsigned long start_millis = 0;
                xSemaphoreTake(ahrsMutex, portMAX_DELAY);
                yaw_start = ahrsData.yaw;   // in degrees or radians (be consistent)
                start_millis = ahrsData.timestamp_ms;
                xSemaphoreGive(ahrsMutex);
                noInterrupts();
                int32_t initLeft = (encoderManager.getTicks(0) + encoderManager.getTicks(2)) / 2;
                int32_t initRight = (encoderManager.getTicks(1) + encoderManager.getTicks(3)) / 2;
                interrupts();

                vTaskDelay(pdMS_TO_TICKS(settleTimeMs / 2));

                noInterrupts();
                int32_t finalLeft = (encoderManager.getTicks(0) + encoderManager.getTicks(2)) / 2;
                int32_t finalRight = (encoderManager.getTicks(1) + encoderManager.getTicks(3)) / 2;
                interrupts();

                // Data capture the beginning of the steady‑state interval
                float yaw_end = 0;
                unsigned long end_millis = 0;
                xSemaphoreTake(ahrsMutex, portMAX_DELAY);
                yaw_end = ahrsData.yaw;   // in degrees or radians (be consistent)
                end_millis = ahrsData.timestamp_ms;
                xSemaphoreGive(ahrsMutex);
                int dt = end_millis - start_millis;
                Serial.printf("dt is %i\nsettle time is %i\n", dt, settleTimeMs);
                float leftSpeed = (finalLeft - initLeft) / (dt / 1000.0f);
                float rightSpeed = (finalRight - initRight) / (dt / 1000.0f);
                float yawRate_dps = (yaw_end - yaw_start) / (dt / 1000.0f);
                log.timestamp_ms = millis();
                log.leftVoltage = speedTestLeftVoltage;
                log.rightVoltage = speedTestRightVoltage;
                log.leftSpeed = leftSpeed;
                log.rightSpeed = rightSpeed;
                log.yawRate = yawRate_dps;

                // Serial.printf("RESULT,%.2f,%.2f,%.2f,%.2f\n",
                //     speedTestLeftVoltage, speedTestRightVoltage,
                //     leftSpeed, rightSpeed);

                Serial.printf("RESULT,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
                log.leftVoltage, log.rightVoltage,
                log.leftSpeed, log.rightSpeed, log.yawRate);   

                send_log_packet(LOG_TYPE_SPEED_TEST, &log, sizeof(log));
                speedTestLeftVoltage = 0;
                speedTestRightVoltage = 0;
                vTaskDelay(pdMS_TO_TICKS(settleTimeMs));
            }
            
            for (uint8_t i = 0; i < numSteps; i++) {
                speedTestLeftVoltage = voltageSteps[i];
                speedTestRightVoltage = -voltageSteps[i];
                Serial.printf("SET,LEFT %.2f, RIGHT %.2f\n", speedTestLeftVoltage, speedTestRightVoltage);

                // Data capture the beginning of the steady‑state interval
                float yaw_start = 0;
                unsigned long start_millis = 0;
                xSemaphoreTake(ahrsMutex, portMAX_DELAY);
                yaw_start = ahrsData.yaw;   // in degrees or radians (be consistent)
                start_millis = ahrsData.timestamp_ms;
                xSemaphoreGive(ahrsMutex);

                noInterrupts();
                int32_t initLeft = (encoderManager.getTicks(0) + encoderManager.getTicks(2)) / 2;
                int32_t initRight = (encoderManager.getTicks(1) + encoderManager.getTicks(3)) / 2;
                interrupts();

                // Test duration
                vTaskDelay(pdMS_TO_TICKS(settleTimeMs / 4));

                noInterrupts();
                int32_t finalLeft = (encoderManager.getTicks(0) + encoderManager.getTicks(2)) / 2;
                int32_t finalRight = (encoderManager.getTicks(1) + encoderManager.getTicks(3)) / 2;
                interrupts();


                // Data capture the beginning of the steady‑state interval
                float yaw_end = 0;
                unsigned long end_millis = 0;
                xSemaphoreTake(ahrsMutex, portMAX_DELAY);
                yaw_end = ahrsData.yaw;   // in degrees or radians (be consistent)
                end_millis = ahrsData.timestamp_ms;
                xSemaphoreGive(ahrsMutex);
                int dt = end_millis - start_millis;
                Serial.printf("dt is %i\nsettle time is %i\n", dt, settleTimeMs);
                float leftSpeed = (finalLeft - initLeft) / (dt / 1000.0f);
                float rightSpeed = (finalRight - initRight) / (dt / 1000.0f);
                float yawRate_dps = (yaw_end - yaw_start) / (dt / 1000.0f);

                log.timestamp_ms = millis();
                log.leftVoltage = speedTestLeftVoltage;
                log.rightVoltage = speedTestRightVoltage;
                log.leftSpeed = leftSpeed;
                log.rightSpeed = rightSpeed;
                log.yawRate = yawRate_dps;

                Serial.printf("RESULT,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
                log.leftVoltage, log.rightVoltage,
                log.leftSpeed, log.rightSpeed, log.yawRate);  

                send_log_packet(LOG_TYPE_SPEED_TEST, &log, sizeof(log));
                speedTestLeftVoltage = 0.0;
                speedTestRightVoltage = 0.0;
                vTaskDelay(pdMS_TO_TICKS(settleTimeMs));// Wait to eliminate any angular velocity before the next test.

            }
            // End test
            speedTestMode = false;
            speedTestRunning = false;
            speedTestLeftVoltage = 0;
            speedTestRightVoltage = 0;
            Serial.println("TEST_END");
        }
    }
}

void motionSensorTask(void *pvParameters) 
{

  Serial.println("Motion sensor task started");
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(5); 
  for(;;)
  {
    if (!systemRunning) 
    {
        Serial.printf("system flag is %i, calibrated flag is %i\n", systemRunning, calibrated);
        Serial.println("waiting for calibration or system flag motionsensor task");
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
    }

    if (calibrationRequested) {
        Serial.println("calibration is on going");
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
    }
    Serial.println("sensor task");
    // time stamp
    motionSensorPacket_t localData;
    localData.timestamp_ms = millis();
    // orientation
    localData.roll = ahrs.getRoll();
    localData.roll = ahrs.getRoll();
    localData.pitch = ahrs.getPitch();
    localData.yaw = ahrs.getYaw();

    localData.encoder_ticks[0] = encoderManager.getTicks(0);
    // Serial.printf("encoder 0 is %i\n", localData.encoder_ticks[0]);
    localData.encoder_ticks[1] = encoderManager.getTicks(1);
    // Serial.printf("encoder 1 is %i\n", localData.encoder_ticks[1]);
    localData.encoder_ticks[2] = encoderManager.getTicks(2);
    // Serial.printf("encoder 2 is %i\n", localData.encoder_ticks[2]);
    localData.encoder_ticks[3] = encoderManager.getTicks(3);
    // Serial.printf("encoder 3 is %i\n", localData.encoder_ticks[3]);
    localData.yawRate = ahrs.getYawRate();
    // Accelerometer and gyro
    ahrs.getAccel(localData.accel_g); 
    ahrs.getGyro(localData.gyro_dps);   

    // Publish to shared variable with mutex
    if (xSemaphoreTake(ahrsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        ahrsData = localData;
        xSemaphoreGive(ahrsMutex);
    } else {
        Serial.println("AHRS mutex timeout!");
        continue;
    }


    vTaskDelayUntil(&lastWake, period);  
  }
}

void printTask(void *pvParameters)
{
  
  for(;;)
  {
    if (!systemRunning) 
    {
        Serial.printf("system flag is %i, calibrated flag is %i\n", systemRunning, calibrated);
        Serial.println("waiting for calibration or system flag print task");
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
    }
    if (calibrationRequested) {
        Serial.println("calibration is on going");
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
    }
    motionSensorPacket_t localData;
    xSemaphoreTake(ahrsMutex, portMAX_DELAY);
    localData = ahrsData;
    xSemaphoreGive(ahrsMutex);
    printAHRSPacket(localData);
    // Serial.print("Yaw: "); Serial.print(localData.yaw);
    // Serial.print("  Left ticks: "); Serial.print(localData.encoder_ticks[0] + localData.encoder_ticks[2]);
    // Serial.print("  Right ticks: "); Serial.println(localData.encoder_ticks[1] + localData.encoder_ticks[3]);  
    // Serial.printf("motorcontrollertask hertz is %i\n", motorControllerTaskHertCount);    
    ahrsTaskHertCount = motorControllerTaskHertCount = motorTaskHertCount = 0;

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
  delay(10);
  // Begin serial communication with the second mcu
  // while(!Serial) {}
  commSerial.begin(115200);
  ahrsMutex = xSemaphoreCreateMutex();
  sysPwrMutex = xSemaphoreCreateMutex();
  speedTestSemaphore = xSemaphoreCreateBinary();

  if (ahrsMutex == NULL) {
      Serial.println("Failed to create ahrsmutex");
      while (1);
  }
  if (sysPwrMutex == NULL) {
      Serial.println("Failed to create sysPwrmutex");
      while (1);
  }
  if (speedTestSemaphore == NULL) {
      Serial.println("Failed to create speed test semaphore");
      while (1);
  }
  // Serial.printf("hardware start millis %i\n", millis());
  if(initAllHardware(&hardwareSensorStatus)) {
    Serial.println("hardware initialized  sending startupack ....");
    hardwareInitialized = true;
  } else {
      Serial.println("hardware initialization failed  sending Nack ....");
      Serial.printf("hardware init failed errorcode %i\n", hardwareSensorStatus);  
      hardwareInitialized = false;          
      while(1) {
        digitalWrite(LED_BUILTIN, HIGH);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        digitalWrite(LED_BUILTIN, LOW);
        vTaskDelay(500 / portTICK_PERIOD_MS);
      }

  }
    
  // Serial.printf("hardware end millis %i\n", millis());
    xTaskCreate(ahrsTask, "AHRS", 1024, NULL, 4, &ahrsHandle);  
    xTaskCreate(commsTask, "comms", 1024, NULL, 3, &commsHandle);
    xTaskCreate(motionSensorTask, "motionSensor", 512, NULL, 3, &motionSensorHandle); // priority 
    // xTaskCreate(motorTask, "Motor", 512, NULL, 2, &motorHandle);
    xTaskCreate(motionControllerTask, "MotionControl", 1024, NULL, 2, &motionControlHandle);
    xTaskCreate(printTask, "Print", 256, NULL, 2, &printHandle);
    xTaskCreate(speedTestTask, "speedTestTask", 2048, NULL, 2, NULL);
    xTaskCreate(ahrsCalibrationTask, "Calibration", 512, NULL, 5, NULL); // highest priority
    Serial.println("All tasks created, starting scheduler...");
    vTaskStartScheduler();

}


void loop() {
  
}

bool initAllHardware(uint8_t* errorCode)
{
    pinMode(LED_BUILTIN, OUTPUT);
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
    encoderManager.setDebounceBits(6);
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


    encoderManager.setDebounceBits(6);
    // Start polling at 10kHz (10µs intervals)
    if(!encoderManager.begin(5000, TIM2)) {
      Serial.println("Encoder manager failed");
      *errorCode = 3;
      return false;
    }
    encoderManager.printDebugInfo();  
    return true;
}


void calibrateMagnetometer2D(int duration) {
    Serial.println("2D Mag Cal: Spin robot 360° slowly in place...");
    ahrs.mag.startCalibration();
    unsigned long start = millis();
    while(millis() - start < (duration / 2.0)) {
        ahrs.mag.updateCalibration();
        analog_turn_l(35);
        Serial.println("Running mag calibration");
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    halt();
    start = millis();
    while(millis() - start < (duration / 2.0)) {
        ahrs.mag.updateCalibration();
        analog_turn_r(35);
        Serial.println("Running mag calibration");
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    halt();
    ahrs.mag.endCalibration();
    Serial.println("Mag Calibration done."); 
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


void sendVisualizationData(motionSensorPacket_t data)
{
    Serial.print("PITCH:"); Serial.print(data.pitch);
    Serial.print(",ROLL:"); Serial.print(data.roll);
    Serial.print(",YAW:"); Serial.println(data.yaw);       
}