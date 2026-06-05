// ==== 1. INCLUDES AND GLOBAL OBJECTS ====

#include "AHRS.h"
#include <task.h>
#include <EncoderManager.h>
#include "motor_control.h"
#include "MotionController.h"
#include "communications.h"
#include "comm_protocol.h"
#include <event_groups.h>

// Stack overflow hook
extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    Serial.print("STACK OVERFLOW in ");
    Serial.println(pcTaskName);
    while (1);
}

// Event bits and system flags
#define EVT_RUNNING         (1 << 0)   // system enabled (from CMD_RUN)
#define EVT_CALIB_REQUEST   (1 << 1)   // calibration in progress (tasks must halt)
#define EVT_CALIBRATED      (1 << 2)   // calibration completed successfully

EventGroupHandle_t sysEventGroup;


// Create encoder manager instance
EncoderManager encoderManager;
AHRS ahrs(SDA_PIN, SCL_PIN, 1000000UL);
MotionController motionController;



motionSensorPacket_t ahrsData;
SemaphoreHandle_t ahrsMutex;   // protects ahrsData

pwrStatus_t lastestPwrStatus;
SemaphoreHandle_t sysPwrMutex;

// Hardware system flag tasks
// volatile bool systemRunning = false;              // controls main tasks
SemaphoreHandle_t startSemaphore = NULL; // optionally, for tasks to wait
static bool hardwareInitialized = false;
uint8_t hardwareSensorStatus = 0;
// volatile bool calibrated = false; 
// volatile bool calibrationRequested = true;
// Task handles for debugging
TaskHandle_t ahrsHandle = NULL;
TaskHandle_t motorHandle = NULL;
TaskHandle_t motionSensorHandle = NULL;
TaskHandle_t printHandle = NULL;
TaskHandle_t motionControlHandle = NULL;
TaskHandle_t commsHandle = NULL;
TaskHandle_t poseHandle = NULL;

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


pose_packet_t  robotPose;
SemaphoreHandle_t poseMutex;


bool initAllHardware(uint8_t* errorCode);
void calibrateMagnetometer2D(int duration);

bool isSystemReady() {
    EventBits_t bits = xEventGroupGetBits(sysEventGroup);
    return (bits & EVT_RUNNING) && (bits & EVT_CALIBRATED) && !(bits & EVT_CALIB_REQUEST);
}

void commsTask(void *parameter) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(20); // 50Hz

  commState rxState = WAIT_START;
  uint8_t bufLen = 128;
  uint8_t rxBuffer[bufLen];
  uint8_t rxIndex = 0;
  uint8_t rxLen = 0;
  uint8_t rxType = 0;


  for(;;)
  {
    // Read and parse incoming packets from s3
    while(commSerial.available()) {  
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
            uint8_t combined[3 + rxLen];
            memcpy(combined, header, 3);
            memcpy(combined + 3, rxBuffer, rxLen);
            uint8_t calculated = compute_checksum(combined, 3 + rxLen);

            if(rxType == PKT_TYPE_ACK)
            {
              // Serial.printf("payload len is: %i\n", rxLen);
              // Serial.printf("checksum for received ack is %i\n", b); 
              // Serial.printf("calculated checksum is %i\n", calculated); 
              // Serial.print("ACK bytes: ");
              // for (int i = 0; i < rxIndex; i++) Serial.printf("0x%02X(HEX), %i(DEC)\n", rxBuffer[i], rxBuffer[i]);                     
            }
            if(calculated == b) {
              // Check packet type and parse data
              // check if pack is a command packet
              if(rxType == PKT_TYPE_SENSOR) {
              } 
              else if(rxType == PKT_TYPE_POSE_DATA) {
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
                      // systemRunning = true;
                      xEventGroupSetBits(sysEventGroup, EVT_RUNNING);
                      sendAck(CMD_RUN, 0);
                      break;
                    case CMD_START_SPEED_TEST:
                        if (!speedTestRunning) {
                            speedTestMode = true;
                            xSemaphoreGive(speedTestSemaphore);   // trigger the test task
                            Serial.println("Test start command received");
                            sendAck(CMD_START_SPEED_TEST, 0);
                        }
                        break;
                    case CMD_STOP_SPEED_TEST:
                        speedTestMode = false;
                        speedTestRunning = false;
                        speedTestLeftVoltage = 0;
                        speedTestRightVoltage = 0;
                        Serial.println("Test stop command received");
                        sendAck(CMD_STOP_SPEED_TEST, 0);
                        break;
                    default:
                      Serial.printf("Unknown command: 0x%02X\n", cmd);
                      break;
                  }
                }
              }
              else if (rxType == PWR_STATUS) {
                if(rxLen == sizeof(pwrStatus_t)) {
                  sendAck(PWR_STATUS, 0);
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
              else if(rxType == PKT_TYPE_ACK) {
                uint8_t packetType = rxBuffer[0];    
                uint8_t status = rxBuffer[1];            
                Serial.println("acknowledgement received");
                Serial.printf("acknowledgement is from packet 0X%X\n", packetType);
                Serial.printf("acknowledgement status is %i\n", status);

              }
              else if(rxType == PKT_TYPE_NACK) {
                uint8_t packetType = rxBuffer[0];    
                uint8_t status = rxBuffer[1];            
                Serial.println("no acknowledgement received");
                Serial.printf("acknowledgement is from packet 0X%X\n", packetType);
                Serial.printf("no acknowledgement status is %i\n", status);
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
    // EventBits_t bits = xEventGroupGetBits(sysEventGroup);
    // return (bits & EVT_RUNNING) && (bits & EVT_CALIBRATED) && !(bits & EVT_CALIB_REQUEST);
    if(isSystemReady())
    {
      uint8_t idx = 0;
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

      uint8_t ahrsPacket[sizeof(ahrsPacketPacked_t) + 3 + 1];
      // Packet header, Start byte
      ahrsPacket[idx++] = PKT_START_BYTE;
      // Sensor packet type
      ahrsPacket[idx++] = PKT_TYPE_SENSOR;
      // packet size length
      ahrsPacket[idx++] = sizeof(ahrsPacketPacked_t);
      // copy stored data into packet
      memcpy(&ahrsPacket[idx], &txData, sizeof(ahrsPacketPacked_t));
      // increase packet size to contain sensor data struct
      idx += sizeof(ahrsPacketPacked_t);

      // Store checksum
      ahrsPacket[idx++] = compute_checksum(ahrsPacket, idx);
      // Publish data
      commSerial.write(ahrsPacket, idx);

      // Serial.println("AHRS Data sent succesfully");


      pose_packet_t currentPose;
      xSemaphoreTake(poseMutex, portMAX_DELAY);
      currentPose = robotPose;
      xSemaphoreGive(poseMutex);

      pose_packet_packed_t txPose;
      txPose.timestamp_ms = currentPose.timestamp_ms;
      txPose.roll = currentPose.roll;
      txPose.pitch = currentPose.pitch;
      txPose.yaw = currentPose.yaw;
      txPose.x = currentPose.x;
      txPose.y = currentPose.y;
      txPose.theta = currentPose.theta;
      txPose.v_linear = currentPose.v_linear;
      txPose.v_angular = currentPose.v_angular;

      uint8_t posePacket[sizeof(pose_packet_packed_t) + 3 + 1];
      idx = 0;
      // Packet header, Start byte
      posePacket[idx++] = PKT_START_BYTE;
      // Sensor packet type
      posePacket[idx++] = PKT_TYPE_POSE_DATA;
      // packet size length
      posePacket[idx++] = sizeof(pose_packet_packed_t);
      // copy stored data into packet
      memcpy(&posePacket[idx], &txPose, sizeof(pose_packet_packed_t));
      // increase packet size to contain sensor data struct
      idx += sizeof(pose_packet_packed_t);

      // Store checksum
      posePacket[idx++] = compute_checksum(posePacket, idx);
      // Publish data
      commSerial.write(posePacket, idx);
      // Serial.println("Pose Data sent succesfully");
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    // vTaskDelay(period);
     vTaskDelayUntil(&lastWake, period);
  }
}


void ahrsTask(void *pvParameters) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(5); 
  for(;;) {
    if (!isSystemReady()) {
        lastWake = xTaskGetTickCount();
        vTaskDelay(pdMS_TO_TICKS(1));
        continue;
    }
    // uint32_t start = micros();
    ahrs.update();
    // uint32_t elapsed = micros() - start;
    // if (elapsed > 2000) Serial.printf("AHRS update took %lu us\n", elapsed);
    vTaskDelayUntil(&lastWake, period);  
    // Serial.println("ahrs update ran");
  }
}

void ahrsCalibrationTask(void *pvParameters) {
    // Wait for system to be marked as RUNNING (CMD_RUN received)
    xEventGroupWaitBits(sysEventGroup, EVT_RUNNING, pdFALSE, pdFALSE, portMAX_DELAY);

    Serial.println("Calibration task started. Waiting for battery voltage...");
    // Signal that calibration is starting – other tasks will block
    xEventGroupSetBits(sysEventGroup, EVT_CALIB_REQUEST);
    // Clear CALIBRATED bit until calibration is done
    xEventGroupClearBits(sysEventGroup, EVT_CALIBRATED);

    Serial.println("Got here");
    while (BATTERY_VOLTAGE < 9.0f) {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    Serial.println("Battery OK. Starting IMU calibration...");
    ahrs.calibrateIMU();           // your existing IMU calibration   

    Serial.println("IMU calibration done. Starting 2D mag calibration...");
    calibrateMagnetometer2D(13000); // your existing function
    Serial.println("Magnetometer calibration done.");
    Serial.println("Calibration complete. Other tasks will now start.");

    // ----- Dynamic settling: wait for yaw to stabilise -----
    Serial.println("Waiting for AHRS to settle...");
    const float STABILITY_THRESHOLD_DEG = 0.1f;   // 0.1 degrees change
    const int REQUIRED_STABLE_COUNT = 30;        // 200 consecutive stable updates (0.4 sec at 500 Hz)
    int stableCount = 0;
    float lastYaw = ahrs.getYaw();

    while (stableCount < REQUIRED_STABLE_COUNT) {
      ahrs.update();
      float currentYaw = ahrs.getYaw();
      Serial.printf("current yaw is %.3f\n", currentYaw);
      float delta = fabs(currentYaw - lastYaw);
      lastYaw = currentYaw;

      if (delta < STABILITY_THRESHOLD_DEG) {
          stableCount++;
          Serial.printf("count is %i\n", stableCount);
      } else {
          stableCount = 0;   // reset if a larger jump occurs
      }
      vTaskDelay(pdMS_TO_TICKS(2));   // 2 ms = 500 Hz (matches AHRS update rate)
    }

    Serial.printf("MotionCtrl stack free: %u\n", uxTaskGetStackHighWaterMark(motionControlHandle));
    Serial.printf("Print stack free: %u\n", uxTaskGetStackHighWaterMark(printHandle));
    Serial.printf("AHRS stack free: %u\n", uxTaskGetStackHighWaterMark(ahrsHandle));
    Serial.printf("Pose stack free: %u\n", uxTaskGetStackHighWaterMark(poseHandle));
    Serial.printf("motionsensor stack free: %u\n", uxTaskGetStackHighWaterMark(motionSensorHandle));
    
    // Calibration done – clear request and set calibrated
    xEventGroupClearBits(sysEventGroup, EVT_CALIB_REQUEST);
    xEventGroupSetBits(sysEventGroup, EVT_CALIBRATED);
    Serial.println("Calibration complete.");
    // Suspend or delete itself
    vTaskSuspend(NULL);
}


void motionSensorTask(void *pvParameters) 
{
  Serial.println("Motion sensor task started");
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(5); 
  for(;;)
  {
    if (!isSystemReady()) 
    {
        // Serial.printf("system not ready\n");
        lastWake = xTaskGetTickCount();
        vTaskDelay(pdMS_TO_TICKS(1));
        continue;
    }

    // Serial.println("motion sensor task");
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
    static float filteredYawRate = 0.0f;
    static bool first = true;
    const float alpha = 0.15f;   

    float raw = localData.yawRate;   // deg/s (or rad/s)
    if (first) {
        filteredYawRate = raw;
        first = false;
    } else {
        filteredYawRate = alpha * raw + (1.0f - alpha) * filteredYawRate;
    }
    localData.yawRateFiltered = filteredYawRate;
    // Accelerometer and gyro
    ahrs.getAccel(localData.accel_g); 
    ahrs.getGyro(localData.gyro_dps); 

    xSemaphoreTake(ahrsMutex, portMAX_DELAY);
    ahrsData = localData;
    xSemaphoreGive(ahrsMutex);

    // Publish to shared variable with mutex
    // if (xSemaphoreTake(ahrsMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    //     ahrsData = localData;
    //     xSemaphoreGive(ahrsMutex);
    // } else {
    //     Serial.println("AHRS mutex timeout!");
    //     continue;
    // }


    // vTaskDelay(period);
     vTaskDelayUntil(&lastWake, period);  
  }
}

// Motion Control Task: runs at 20 Hz (20 ms)
// quantization issue
void motionControllerTask(void *pvParameters) {
    uint8_t task_period = 50;
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(task_period); // 20 Hz
    CONTROL_DT = task_period / 1000.0f;
    static long prevTimestamp_ms = 0;
    static float dt = 0.0f;


    // ... existing declarations ...
    static uint32_t lastModeChange = 0;
    static uint8_t mode = 0;        // 0 = straight, 1 = pure rotation, 2 = curved
    const uint32_t interval = 1000;  // 5 seconds

    float linear_vel = 0.3;
    float angular_vel = 0.8;


    // Deadband with hysteresis
    static float last_yawRate = 0;
    const float HYST_IN = (DEG_TO_RAD * 0.5);   // 0.5 deg/s – enter dead zone
    const float HYST_OUT = DEG_TO_RAD;  // 1.0 deg/s – exit dead zone
    motionController.setStraight(linear_vel);
    // motionController.setTargetVelocity(linear_vel, angular_vel);
    for (;;) {
        if (!isSystemReady()) 
        {
            // Serial.printf("system not ready\n");
            lastWake = xTaskGetTickCount();
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        // ---- Mode cycling logic ----
        // uint32_t now = millis();
        // if (now - lastModeChange >= interval) {
        //     lastModeChange = now;
        //     mode = (mode + 1) % 13;

        //     switch (mode) {
        //         case 0:
        //             motionController.setStraight(linear_vel);
        //             break;
        //         case 1:
        //             motionController.setStraight(-linear_vel);

        //             break;
        //         case 2:
        //             motionController.setTargetVelocity(0.0f, angular_vel);
        //             break;
        //         case 3:
        //             motionController.setTargetVelocity(0.0f, -angular_vel);
        //             break;
        //        case 4:
        //             motionController.setTargetVelocity(linear_vel, angular_vel);
        //             break;
        //         case 5:
        //             motionController.setTargetVelocity(-linear_vel, -angular_vel);
        //             break;
        //         case 6:
        //             motionController.setStraight(0.0);
        //             break;
        //         case 7:
        //           // motionController.setStraight(0.0);
        //           break;
        //         case 8:
        //             // motionController.setStraight(0.0);
        //             break;
        //         case 9:
        //           // motionController.setStraight(0.0);
        //           break;
        //         case 10:
        //           // motionController.setStraight(0.0);
        //           break;
        //         case 11:
        //             // motionController.setStraight(0.0);
        //             break;
        //         case 12:
        //           // motionController.setStraight(0.0);
        //           break;
        //     }
        // }

        // Serial.println("motion controller is running");
        // Grab latest sensor data
      
        motionSensorPacket_t localData;
        xSemaphoreTake(ahrsMutex, portMAX_DELAY);
        localData = ahrsData;
        xSemaphoreGive(ahrsMutex);
        // if (xSemaphoreTake(ahrsMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        //     localData = ahrsData;
        //     xSemaphoreGive(ahrsMutex);
        // } else {
        //     Serial.println("AHRS mutex timeout!");
        //     continue;
        // }

        // Compute average ticks for left and right sides
        float leftTicksAvg = (localData.encoder_ticks[0] + localData.encoder_ticks[2]) / 2.0f;
        float rightTicksAvg = (localData.encoder_ticks[1] + localData.encoder_ticks[3]) / 2.0f;
        // Serial.printf("lefTicksAvg is %.2f\n", leftTicksAvg);
        // Serial.printf("rightTicksAvg is %.2f\n", rightTicksAvg);

        if (prevTimestamp_ms == 0) {
            dt = CONTROL_DT;   // first cycle use nominal
        } else {
            dt = (localData.timestamp_ms - prevTimestamp_ms) / 1000.0f;
            // if (dt > 0.1f) dt = 0.05f;   // cap to prevent spikes
        }
        prevTimestamp_ms = localData.timestamp_ms;

        // Serial.printf("dt is %.6f\n", dt);     

        Serial.printf("yawRate degs is %.3f\n", localData.yawRateFiltered);
        float yawRad = localData.yaw * DEG_TO_RAD;

        float yawRateFiltered_rad = localData.yawRateFiltered * DEG_TO_RAD;
        // Deadband
        if(fabs(yawRateFiltered_rad) < HYST_IN) {
          yawRateFiltered_rad = 0;
        } else if(fabs(yawRateFiltered_rad) < HYST_OUT && fabs(last_yawRate) < HYST_IN) {
          yawRateFiltered_rad = 0;
        }
        last_yawRate = yawRateFiltered_rad;
        // Serial.printf("filtered yawRate degs is %.3f\n", yawRateFiltered_rad);
        float motorsVolt[2];
        motionController.update(leftTicksAvg, rightTicksAvg, yawRad, yawRateFiltered_rad, dt, motorsVolt);

        // Serial.printf("left motor voltage is %.3f\nright motor voltage is %.3f\n", motorsVolt[0], motorsVolt[1]);
        setLeftMotorsVoltage(motorsVolt[0]);
        setRightMotorsVoltage(motorsVolt[1]); 
        bool leftMotorDir = (motorsVolt[0] >= 0);   
        bool rightMotorDir = (motorsVolt[1] >= 0);           
        // Serial.printf("left motor direction is %i\nright motor direction is %i\n", leftMotorDir, rightMotorDir);   
        encoderManager.setDirection(0, leftMotorDir);
        encoderManager.setDirection(2, leftMotorDir);
        encoderManager.setDirection(1, rightMotorDir);
        encoderManager.setDirection(3, rightMotorDir);
        // Serial.println("after motion control update");
        motorControllerTaskHertCount++;

        Serial.println();
        Serial.println();
        Serial.println();
        // vTaskDelay(period);
         vTaskDelayUntil(&lastWake, period);
    }
}


void poseTask(void *pvParameters) {

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(50); // 20 Hz

    static float x = 0.0f, y = 0.0f, theta = 0.0f;
    static float prevLeftTicks = 0.0f, prevRightTicks = 0.0f;
    static long prevTimestamp_ms = 0;
    static float dt = 0.0f;
    for (;;) {
        if (!isSystemReady()) 
        {
            // Serial.printf("system not ready\n");
            lastWake = xTaskGetTickCount();
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        motionSensorPacket_t localData;
        // Serial.println("pose task");
        // Get latest sensor data (from ahrsData)
        xSemaphoreTake(ahrsMutex, portMAX_DELAY);
        localData = ahrsData;
        xSemaphoreGive(ahrsMutex);
        // if (xSemaphoreTake(ahrsMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        //     localData = ahrsData;
        //     xSemaphoreGive(ahrsMutex);
        // } else {
        //     Serial.println("AHRS mutex timeout!");
        //     continue;
        // }
        dt = (localData.timestamp_ms - prevTimestamp_ms) /1000.0f;     
        prevTimestamp_ms = localData.timestamp_ms;

        // Average left and right ticks (using 4 encoders)
        float leftTicksAvg = (localData.encoder_ticks[0] + localData.encoder_ticks[2]) / 2.0f;
        float rightTicksAvg = (localData.encoder_ticks[1] + localData.encoder_ticks[3]) / 2.0f;

        // Compute delta distance and heading change from encoders
        float deltaLeft = (leftTicksAvg - prevLeftTicks) / TICKS_PER_METER;
        float deltaRight = (rightTicksAvg - prevRightTicks) / TICKS_PER_METER;
        prevLeftTicks = leftTicksAvg;
        prevRightTicks = rightTicksAvg;

        float deltaDist = (deltaLeft + deltaRight) / 2.0f;
        // Not using encoder heading because IMU is more accurate
        // float deltaThetaEnc = (deltaRight - deltaLeft) / ROBOT_TRACK_WIDTH;

        // Use IMU yaw for absolute heading
        theta = localData.yaw * DEG_TO_RAD;

        // Update x, y using forward displacement projected by heading
        x += deltaDist * cosf(theta);
        y += deltaDist * sinf(theta);

        // Compute linear and angular velocities 
        float v_linear = deltaDist / dt;            // m/s
        float v_angular = (deltaRight - deltaLeft) / ROBOT_TRACK_WIDTH / dt; // rad/s (from encoders)

        // Update global pose structure
        pose_packet_t  pose;
        pose.timestamp_ms = millis();
        pose.x = x;
        pose.y = y;
        pose.theta = theta;
        pose.v_linear = v_linear;
        pose.v_angular = v_angular;
        pose.roll = localData.roll;
        pose.pitch = localData.pitch;
        pose.yaw = theta;

        xSemaphoreTake(poseMutex, portMAX_DELAY); 
        robotPose = pose;
        xSemaphoreGive(poseMutex);

      // if (xSemaphoreTake(poseMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      //     robotPose = pose;
      //     xSemaphoreGive(poseMutex);
      // } else {
      //     Serial.println("AHRS mutex timeout!");
      //     continue;
      // }

        // Optional: send pose to ESP32 via UART (can be added later)
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
    if (!isSystemReady()) 
    {
        // Serial.printf("system not ready\n");
        lastWake = xTaskGetTickCount();
        vTaskDelay(pdMS_TO_TICKS(1));
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
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void speedTestTask(void *pvParameters) {
    const float voltageSteps[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 11.5};
    const int settleTimeMs = 2000;
    const int numSteps = sizeof(voltageSteps)/sizeof(voltageSteps[0]);
    // Example: start going straight at 0.5 m/s after 2 seconds

    for (;;) {
        if (!isSystemReady()) 
        {
            // Serial.printf("system not ready\n");
            vTaskDelay(pdMS_TO_TICKS(1));
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

            //     vTaskDelay(pdMS_TO_TICKS(50));

                
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
                // noInterrupts();
                int32_t initLeft = (encoderManager.getTicks(0) + encoderManager.getTicks(2)) / 2;
                int32_t initRight = (encoderManager.getTicks(1) + encoderManager.getTicks(3)) / 2;
                // interrupts();

                vTaskDelay(pdMS_TO_TICKS(settleTimeMs / 2));

                // noInterrupts();
                int32_t finalLeft = (encoderManager.getTicks(0) + encoderManager.getTicks(2)) / 2;
                int32_t finalRight = (encoderManager.getTicks(1) + encoderManager.getTicks(3)) / 2;
                // interrupts();

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

                // noInterrupts();
                int32_t initLeft = (encoderManager.getTicks(0) + encoderManager.getTicks(2)) / 2;
                int32_t initRight = (encoderManager.getTicks(1) + encoderManager.getTicks(3)) / 2;
                // interrupts();

                // Test duration
                vTaskDelay(pdMS_TO_TICKS(settleTimeMs / 4));

                // noInterrupts();
                int32_t finalLeft = (encoderManager.getTicks(0) + encoderManager.getTicks(2)) / 2;
                int32_t finalRight = (encoderManager.getTicks(1) + encoderManager.getTicks(3)) / 2;
                // interrupts();


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



void printTask(void *pvParameters)
{
  
  for(;;)
  {
    if (!isSystemReady()) 
    {
        // Serial.printf("system not ready\n");
        vTaskDelay(pdMS_TO_TICKS(1));
        continue;
    }
    motionSensorPacket_t localData;
    xSemaphoreTake(ahrsMutex, portMAX_DELAY);
    localData = ahrsData;
    xSemaphoreGive(ahrsMutex);
    // printAHRSPacket(localData);
    // sendVisualizationData(localData);

    pose_packet_t pose;
    if (xSemaphoreTake(poseMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        pose = robotPose;
        xSemaphoreGive(poseMutex);
    } else {
        Serial.println("AHRS mutex timeout!");
        continue;
    }
    // printPose(pose);  
    Serial.printf("Linear velocity: %.2f\n", pose.v_linear);
    Serial.printf("angular velocity: %.2f\n", pose.v_angular);
    Serial.printf("X : %.2f\n", pose.x);
    Serial.printf("Y : %.2f\n", pose.y);
    Serial.printf("theta (degs): %.2f\n", pose.theta * RAD_TO_DEG);

    Serial.printf("%f,%f,%f,%f,%f\n", pose.v_linear, pose.v_angular, pose.x, pose.y, (pose.theta * RAD_TO_DEG));

    ahrsTaskHertCount = motorControllerTaskHertCount = motorTaskHertCount = 0;
    static unsigned long lastToggle = 0;
    if(millis() - lastToggle >= 500)
    {
      digitalToggle(LED_PIN);
      lastToggle = millis();
    }
    digitalToggle(LED_PIN);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(921600);
  // Begin serial communication with the second mcu
  // while(!Serial) {}
  commSerial.begin(115200);
  ahrsMutex = xSemaphoreCreateMutex();
  sysPwrMutex = xSemaphoreCreateMutex();
  speedTestSemaphore = xSemaphoreCreateBinary();
  poseMutex = xSemaphoreCreateMutex();


  sysEventGroup = xEventGroupCreate();


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
  if (poseMutex == NULL) {
      Serial.println("Failed to create poseMutex");
      while (1);
  }

  if (sysEventGroup == NULL) {
      Serial.println("Failed to create event group");
      while(1);
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
    xTaskCreate(motionSensorTask, "motionSensor", 1024, NULL, 3, &motionSensorHandle); // priority 
    // xTaskCreate(motorTask, "Motor", 512, NULL, 2, &motorHandle);
    xTaskCreate(motionControllerTask, "MotionControl", 1024, NULL, 2, &motionControlHandle);
    xTaskCreate(printTask, "Print", 1024, NULL, 2, &printHandle);
    xTaskCreate(speedTestTask, "speedTestTask", 2048, NULL, 2, NULL);
    xTaskCreate(ahrsCalibrationTask, "Calibration", 1024, NULL, 5, NULL); // highest priority
    xTaskCreate(poseTask, "Pose", 1024, NULL, 2, &poseHandle);
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
    int voltage = 6.3;
    while(millis() - start < (duration / 2.0)) {
        ahrs.mag.updateCalibration();

        int leftVoltage = -voltage;
        int rightVoltage = voltage;
        bool leftMotorDir = (leftVoltage >= 0);   
        bool rightMotorDir = (rightVoltage >= 0);           
        // Serial.printf("left motor direction is %i\nright motor direction is %i\n", leftMotorDir, rightMotorDir);   
        setLeftMotorsVoltage(leftVoltage);
        setRightMotorsVoltage(rightVoltage);        
        encoderManager.setDirection(0, leftMotorDir);
        encoderManager.setDirection(1, leftMotorDir);
        encoderManager.setDirection(2, rightMotorDir);
        encoderManager.setDirection(3, rightMotorDir);
        setLeftMotorsVoltage(leftVoltage);
        setRightMotorsVoltage(rightVoltage);
        // Serial.println("Running mag calibration");
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    halt();
    encoderManager.printAllTicks();
    start = millis();
    while(millis() - start < (duration / 2.0)) {
        ahrs.mag.updateCalibration();
        int leftVoltage = voltage;
        int rightVoltage = -voltage;
        bool leftMotorDir = (leftVoltage >= 0);   
        bool rightMotorDir = (rightVoltage >= 0);           
        // Serial.printf("left motor direction is %i\nright motor direction is %i\n", leftMotorDir, rightMotorDir);   
        setLeftMotorsVoltage(leftVoltage);
        setRightMotorsVoltage(rightVoltage);        
        encoderManager.setDirection(0, leftMotorDir);
        encoderManager.setDirection(1, leftMotorDir);
        encoderManager.setDirection(2, rightMotorDir);
        encoderManager.setDirection(3, rightMotorDir);
        setLeftMotorsVoltage(leftVoltage);
        setRightMotorsVoltage(rightVoltage);
        // Serial.println("Running mag calibration");
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    halt();
    ahrs.mag.endCalibration();
    encoderManager.printAllTicks();
    encoderManager.resetAllTicks();
    encoderManager.setDirection(0, 1);
    encoderManager.setDirection(2, 1);
    encoderManager.setDirection(1, 1);
    encoderManager.setDirection(3, 1);
    Serial.println("Mag Calibration done."); 
    encoderManager.printDebugInfo();

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
