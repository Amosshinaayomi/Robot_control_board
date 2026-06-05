#include "pitches.h"
#include "pins.h"
#include "communications.h"
#include "pwr_mgmt.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "tof_sensors.h"

// communication flags
bool commsEstablished;
bool commsTimeoutExceeded;

ahrsPacketPacked_t lastestAhrsPacket;
SemaphoreHandle_t dataMutex;
volatile bool systemIsReady = false;
pose_packet_packed_t lastestPosePacket;

TaskHandle_t commsHandle = NULL;
TaskHandle_t powerHandle = NULL;
TaskHandle_t distSensorHandle = NULL;

// NVS storage
nvs_handle_t nvsHandle;
String speedTestResult;



void distSensorTask(void *parameters)
{
  // if (!pcf.begin(0x20, &Wire)) {
  //   Serial.println("Couldn't find PCF8574");
  //   while (1);
  // }
  while(!systemIsReady){
    vTaskDelay(pdMS_TO_TICKS(50));
  }


  bool ioInit = ioExpander.begin(0);
  if(ioInit){
    for(int i = 0; i < DIST_SEN_NUMB; i++) {
      ioExpander.write(XSHUT_PINS[i], LOW);  // Hold in reset/shutdown
    }
    for(int i = 0; i < DIST_SEN_NUMB-1; i++)
    {
      Serial.print("\nInitializing sensor ");
      Serial.println(i);

      ioExpander.write(XSHUT_PINS[i], HIGH);
      vTaskDelay(5 / portTICK_PERIOD_MS);

      if(!tof_sensors[i].begin(SENSOR_ADDRESSES[i]))
      {
        Serial.println(" FAILED!");
        Serial.print("  Could not initialize sensor at address 0x");
        Serial.println(SENSOR_ADDRESSES[i], HEX);
      }
    
      Serial.println("Success");
      tof_sensors[i].configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED);
      // Start continuous ranging mode
      tof_sensors[i].startRangeContinuous(UPDATE_INTERVAL_MS);
    }

  }
  for(;;)
  {
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void powerTask(void *parameters)
{
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(100);
  
  for(;;)
  {
    uint32_t adcSum = 0;
    const uint8_t samples = 5;
    for(uint8_t i = 0; i < samples; i++)
    {
      adcSum += analogRead(BATTERY_PIN);
      // Serial.printf("batteryADC compounded value is %i is %i\n", adcSum, i);
    }
    float avgAdc = (float)adcSum / samples;
    // Serial.printf("Average battery ADCValue is %f\n", avgAdc);

    float batteryVoltage = (avgAdc / 3000) * BATTERY_FULL_VOLT; // 3103 because at max voltage 12.6 Vout is 2.5v

    pwrStatus_t localStatus;
    localStatus.timestamp_ms = millis();
    localStatus.batteryVoltage = batteryVoltage;

    xSemaphoreTake(sysPwrMutex, portMAX_DELAY);
    sysPwrStatus = localStatus;
    xSemaphoreGive(sysPwrMutex);

    // Serial.printf("Power: time=%lu, voltage=%.2f\n", localStatus.timestamp_ms, localStatus.batteryVoltage);
    vTaskDelayUntil(&lastWake, period);
  }
}

void F411StartupTask(void *parameters)
{
  // vTaskDelay(pdMS_TO_TICKS(10));
  performStartupHandshake();
  vTaskDelete(NULL);
}

void performStartupHandshake() {
    const int maxRetries = 3;
    const int timeoutMs = 500;
    // startupAckReceived = false;
    // startupNackReceived = false;
    for (int retry = 0; retry < maxRetries; retry++) {
        Serial.println("Sending CMD_STARTUP_REQ");
        sendCommand(CMD_STARTUP_REQ, NULL, 0);
        Serial.printf("Startup request sent (retry %d)\n", retry + 1);

        uint32_t start = millis();
        while (millis() - start < timeoutMs) {
            if (startupAckReceived) {
                Serial.println("F411 startup successful");
                sendCommand(CMD_RUN, NULL, 0);
                commsEstablished = true;
                indicateError(startupErrorCode);
                Serial.println("Sent RUN command");
                return; // success
            } else{
                Serial.println("startup ack wasn't received");
            }
            if (startupNackReceived) {
                Serial.printf("F411 startup failed with error code %d\n", startupErrorCode);
                indicateError(startupErrorCode); // optional
                playNote();
                return; // failure
            }
            vTaskDelay(pdMS_TO_TICKS(50)); // small delay to avoid tight loop
        }
        Serial.println("Timeout, retrying...");
    }
    Serial.println("F411 not responding. Check connection.");
    commsTimeoutExceeded = true;
    indicateError(255); // timeout error
    for(;;)
    {
        rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);  // Red
        vTaskDelay(pdMS_TO_TICKS(400));
        rgbLedWrite(RGB_BUILTIN, 0, 0, 0);  // black
        vTaskDelay(pdMS_TO_TICKS(400));
    }
}

void commsTask(void* parameter)
{
    commState currentState = WAIT_START;

    byte rx_buffer[256];
    int rx_index = 0;
    byte rx_len = 0;
    byte rx_type = 0;

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);
    for(;;)
    {
        if(commsEstablished)
        {
            pwrStatus_t currentPwrStatus;
            xSemaphoreTake(sysPwrMutex, portMAX_DELAY);
            currentPwrStatus = sysPwrStatus;
            xSemaphoreGive(sysPwrMutex);

            uint8_t packet[sizeof(pwrStatus_t) + 3];
            uint8_t idx = 0;
            // Packet header, Start byte
            packet[idx++] = PKT_START_BYTE;
            // Packet data type
            packet[idx++] = PWR_STATUS;
            // data type size
            packet[idx++] = sizeof(pwrStatus_t);
            // copy stored data into packet
            memcpy(&packet[idx], &currentPwrStatus, sizeof(pwrStatus_t));
            // increase packet size to contain sensor data struct
            idx += sizeof(pwrStatus_t);
            packet[idx++] = compute_checksum(packet, idx);
            commSerial.write(packet, idx);
            // Serial.println("Power data sent successfully");
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        while(commSerial.available()) {  
            byte b = commSerial.read();
            switch(currentState) {
            case WAIT_START: 
                if (b == PKT_START_BYTE) currentState = WAIT_TYPE;
                // Serial.println("packet start");
                break;
            case WAIT_TYPE: 
                rx_type = b;
                // Serial.printf("packet type is 0x%X\n", rx_type);
                currentState = WAIT_LEN;
                break;

            case WAIT_LEN: 
                rx_len = b;
                rx_index = 0;
                if(rx_len == 0)
                {
                currentState = WAIT_CHECKSUM;
                } else if(rx_len <= sizeof(rx_buffer)) {
                currentState = WAIT_PAYLOAD;
                } else {
                currentState = WAIT_START;
                }
                // Serial.printf("packet length is %i\n", rx_len);
                break;
            case WAIT_PAYLOAD: 
                rx_buffer[rx_index++] = b;
                if(rx_index >= rx_len)
                {
                    currentState = WAIT_CHECKSUM;
                }
                // Serial.println("packet payload is added to buffer");
                break;
            case WAIT_CHECKSUM: {
                byte header[3] = {PKT_START_BYTE, rx_type, rx_len};
                uint8_t combined[3 + rx_len];
                memcpy(combined, header, 3);
                memcpy(combined + 3, rx_buffer, rx_len);
                uint8_t calculated = compute_checksum(combined, 3 + rx_len);
                if(rx_type == PKT_TYPE_ACK)
                {
                    Serial.printf("payload len is: %i\n", rx_len);
                    Serial.printf("checksum for received ack is %i\n", b); 
                    Serial.printf("calculated checksum is %i\n", calculated); 
                    Serial.print("ACK bytes: ");
                    for (int i = 0; i < rx_index; i++) Serial.printf("0x%02X(HEX), %i(DEC)\n", rx_buffer[i], rx_buffer[i]);               
                }


                if(calculated == b) {
                    // Check packet type and parse data
                    if(rx_type == PKT_TYPE_SENSOR)
                    {
                        // Serial.println("Valid sensor data received");
                        // Serial.println("AHRS SENSOR DATA PACKET IS RECEIVED");
                        sendAck(PKT_TYPE_SENSOR, 0);
                        ahrsPacketPacked_t receivedPacket;
                        memcpy(&receivedPacket, rx_buffer, sizeof(ahrsPacketPacked_t));               
                        xSemaphoreTake(dataMutex, portMAX_DELAY);
                        lastestAhrsPacket = receivedPacket;
                        xSemaphoreGive(dataMutex);

                        // Copy data from buffer into struct
        
                        // printlastestAHRSPacket(lastestAhrsPacket);
                        // sendVisualizationData(lastestAhrsPacket);

                    } 
                    else if(rx_type == PKT_TYPE_COMMAND) {
                        Serial.println("Valid command received");
                        if(rx_len >= 1) {
                            uint8_t cmd = rx_buffer[0];
                            if (cmd == CMD_STARTUP_ACK) {
                                startupAckReceived = true;
                                Serial.println("received startup ack");

                            } else if (cmd == CMD_STARTUP_NACK && rx_len >= 2) {
                                startupNackReceived = true;
                                Serial.println("received startup Nack");
                                Serial.printf("errorcode is %i\n", startupErrorCode);
                                startupErrorCode = rx_buffer[1];
                            }
                        }
                    } 
                    else if (rx_type == PKT_TYPE_LOG && rx_len >= 1) {
                        uint8_t logType = rx_buffer[0];
                        size_t dataLen = rx_len - 1;
                        if (logType == LOG_TYPE_SPEED_TEST && dataLen == sizeof(speed_test_log_t)) {
                            speed_test_log_t log;
                            sendAck(LOG_TYPE_SPEED_TEST, 0);                            
                            memcpy(&log, &rx_buffer[1], dataLen);
   
                            // Optionally print or forward via ESP‑NOW
                            Serial.printf("SPEED_TEST: %.2f,%.2f,%.2f,%.2f\n",
                            log.leftVoltage, log.rightVoltage,
                            log.leftSpeed, log.rightSpeed);
                            speedTestResults.push_back(log);    
                            // Forward over ESP‑NOW (if needed)
                        
                        }
                        else {
                            Serial.println("Unknown log type or size mismatch");
                        }
                    }
                    else if(rx_type == PKT_TYPE_POSE_DATA) {
                        
                        sendAck(PKT_TYPE_POSE_DATA, 0);
                        pose_packet_packed_t rxPose;
                        memcpy(&rxPose, rx_buffer, sizeof(pose_packet_packed_t));               
                        lastestPosePacket = rxPose;
                        // Serial.println("pose data received");                        
                        printPose(lastestPosePacket);


                    }
                    else if(rx_type == PKT_TYPE_ACK) {
                        uint8_t packetType = rx_buffer[0];    
                        uint8_t status = rx_buffer[1];            
                        Serial.println("acknowledgement received");
                        Serial.printf("acknowledgement is from packet 0X0%X\n", packetType);
                        Serial.printf("acknowledgement status is %i\n", status);
                    }
                    else if(rx_type == PKT_TYPE_NACK) {
                        uint8_t packetType = rx_buffer[0];    
                        uint8_t status = rx_buffer[1];            
                        Serial.println("no acknowledgement received");
                        Serial.printf("acknowledgement is from packet 0X0%X\n", packetType);
                        Serial.printf("no acknowledgement status is %i\n", status);
                    }
                }
                else {
                    Serial.println("Checksum error – packet corrupted");
                }
                currentState = WAIT_START;
                break;
            }
            default:
                currentState = WAIT_START;
            }
        }
        vTaskDelayUntil(&lastWake, period);
    }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(921600);
  // while(!Serial){}
  nvs_flash_init();
  nvs_open("storage", NVS_READWRITE, &nvsHandle);

  if(!Wire.begin()){
    while(1);
  }
  Wire.setClock(400000UL);  

  // Serial.println("Hello World");
  pinMode(BATTERY_PIN, INPUT);
  // Creations of mutexes
  dataMutex =  xSemaphoreCreateMutex();
  sysPwrMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL) {
      Serial.println("Mutex creation failed");
      while (1);
  }



  commSerial.begin(115200, SERIAL_8N1, COMM_UART_RX, COMM_UART_TX);  
  xTaskCreate(commsTask, "commsTask", 4096, NULL, 3, &commsHandle);
  xTaskCreate(powerTask, "powerTask", 4096, NULL, 1, &powerHandle);
  // xTaskCreate(distSensorTask, "distSensorTask", 4096, NULL, 2, &powerHandle);
  xTaskCreate(F411StartupTask, "F411StartupTask", 4096, NULL, 1, NULL);
  // playNote();
  // delay(200);
  // sendCommand(CMD_START_SPEED_TEST, NULL, 0);
}

void loop() {
    systemIsReady = true;
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line == "test_start") {
            sendCommand(CMD_START_SPEED_TEST, NULL, 0);
            Serial.println("Sent test start to F411");
        }
        else if (line == "test_stop") {
            sendCommand(CMD_STOP_SPEED_TEST, NULL, 0);
            Serial.println("Sent test stop to F411");
        }
        else if (line == "print_results") {
            Serial.println("left_voltage,right_voltage,left_speed,right_speed, voltageDiff, yawRate_radps, yawRate_dps");
            for (size_t i = 0; i < speedTestResults.size(); i++) {
                Serial.printf("%.2f,%.2f,%.2f,%.2f,%.2f\n",
                    speedTestResults[i].leftVoltage,
                    speedTestResults[i].rightVoltage,
                    speedTestResults[i].leftSpeed,
                    speedTestResults[i].rightSpeed,
                    speedTestResults[i].yawRate_dps
                );
            }
            speedTestResults.clear();
        }
        else {
            Serial.println("Unknown command. Use test_start or test_stop");
        }
      
    }
    vTaskDelay(pdMS_TO_TICKS(50));
}


// LOGGED DATA ON GROUND
// 00:11:51.722 -> Sent command 0x13 with 0 params
// 00:11:52.703 -> Sent RUN command
// 00:11:52.772 -> SPEED_TEST: 2.00,0.00,21.67,0.00
// 00:11:55.894 -> SPEED_TEST: 3.00,0.00,35.67,0.00
// 00:11:58.977 -> SPEED_TEST: 4.00,0.00,50.00,0.00
// 00:12:02.072 -> SPEED_TEST: 5.00,0.00,65.00,0.00
// 00:14:11.462 -> left_voltage,right_voltage,left_speed,right_speed
// 00:14:11.462 -> 2.00,0.00,6.33,0.00
// 00:14:11.462 -> 3.00,0.00,17.33,0.67
// 00:14:11.462 -> 4.00,0.00,29.33,3.00
// 00:14:11.462 -> 5.00,0.00,43.67,4.00
// 00:14:11.462 -> 6.00,0.00,60.67,1.33
// 00:14:11.462 -> 7.00,0.00,78.67,1.67
// 00:14:11.462 -> 8.00,0.00,91.33,9.33
// 00:14:11.497 -> 9.00,0.00,105.33,6.33
// 00:14:11.497 -> 0.00,2.00,0.00,2.33
// 00:14:11.497 -> 0.00,3.00,0.00,18.67
// 00:14:11.497 -> 0.00,4.00,0.00,30.67
// 00:14:11.497 -> 0.00,5.00,0.00,50.00
// 00:14:11.497 -> 0.00,6.00,0.00,62.67
// 00:14:11.497 -> 0.00,7.00,0.33,78.33
// 00:14:11.497 -> 0.00,8.00,0.00,92.67
// 00:14:11.497 -> 0.00,9.00,0.00,107.00
// 00:14:11.497 -> 2.00,2.00,19.00,19.00
// 00:14:11.497 -> 3.00,3.00,31.67,33.00
// 00:14:11.497 -> 4.00,4.00,47.33,49.33
// 00:14:11.497 -> 5.00,5.00,62.67,64.00
// 00:14:11.497 -> 6.00,6.00,78.00,80.00
// 00:14:11.497 -> 7.00,7.00,93.33,94.67
// 00:14:11.497 -> 8.00,8.00,108.33,109.67
// 00:14:11.497 -> 9.00,9.00,123.00,123.67


// LOGGED DATA WITH FREE WHEELS
// 00:25:47.813 -> Sent command 0x07 with 0 params
// 00:25:47.813 -> Sent test start to F411
// 00:25:50.953 -> SPEED_TEST: 2.00,0.00,20.67,0.00
// 00:25:54.022 -> SPEED_TEST: 3.00,0.00,35.00,0.00
// 00:25:57.107 -> SPEED_TEST: 4.00,0.00,50.00,0.00
// 00:26:00.221 -> SPEED_TEST: 5.00,0.00,65.00,0.00
// 00:26:03.337 -> SPEED_TEST: 6.00,0.00,81.00,0.00
// 00:26:06.421 -> SPEED_TEST: 7.00,0.00,96.00,0.00
// 00:26:09.507 -> SPEED_TEST: 8.00,0.00,112.00,0.00
// 00:26:12.623 -> SPEED_TEST: 9.00,0.00,127.00,0.00
// 00:26:15.721 -> SPEED_TEST: 0.00,2.00,0.00,22.00
// 00:26:18.835 -> SPEED_TEST: 0.00,3.00,0.00,36.67
// 00:26:21.938 -> SPEED_TEST: 0.00,4.00,0.00,52.33
// 00:26:25.023 -> SPEED_TEST: 0.00,5.00,0.00,68.67
// 00:26:28.123 -> SPEED_TEST: 0.00,6.00,0.00,83.67
// 00:26:31.223 -> SPEED_TEST: 0.00,7.00,0.00,99.33
// 00:26:34.337 -> SPEED_TEST: 0.00,8.00,0.00,113.33
// 00:26:37.423 -> SPEED_TEST: 0.00,9.00,0.00,127.33
// 00:26:40.538 -> SPEED_TEST: 2.00,2.00,21.67,23.67
// 00:26:43.623 -> SPEED_TEST: 3.00,3.00,35.33,38.33
// 00:26:46.738 -> SPEED_TEST: 4.00,4.00,50.00,53.67
// 00:26:49.825 -> SPEED_TEST: 5.00,5.00,64.67,68.67
// 00:26:52.937 -> SPEED_TEST: 6.00,6.00,80.00,84.33
// 00:26:56.025 -> SPEED_TEST: 7.00,7.00,95.00,99.33
// 00:26:59.126 -> SPEED_TEST: 8.00,8.00,111.00,112.67
// 00:27:02.221 -> SPEED_TEST: 9.00,9.00,125.00,125.00
// 00:27:47.623 -> left_voltage,right_voltage,left_speed,right_speed
// 00:27:47.623 -> 2.00,0.00,20.67,0.00
// 00:27:47.656 -> 3.00,0.00,35.00,0.00
// 00:27:47.656 -> 4.00,0.00,50.00,0.00
// 00:27:47.656 -> 5.00,0.00,65.00,0.00
// 00:27:47.656 -> 6.00,0.00,81.00,0.00
// 00:27:47.656 -> 7.00,0.00,96.00,0.00
// 00:27:47.656 -> 8.00,0.00,112.00,0.00
// 00:27:47.656 -> 9.00,0.00,127.00,0.00
// 00:27:47.656 -> 0.00,2.00,0.00,22.00
// 00:27:47.656 -> 0.00,3.00,0.00,36.67
// 00:27:47.656 -> 0.00,4.00,0.00,52.33
// 00:27:47.656 -> 0.00,5.00,0.00,68.67
// 00:27:47.656 -> 0.00,6.00,0.00,83.67
// 00:27:47.656 -> 0.00,7.00,0.00,99.33
// 00:27:47.656 -> 0.00,8.00,0.00,113.33
// 00:27:47.656 -> 0.00,9.00,0.00,127.33
// 00:27:47.656 -> 2.00,2.00,21.67,23.67
// 00:27:47.656 -> 3.00,3.00,35.33,38.33
// 00:27:47.656 -> 4.00,4.00,50.00,53.67
// 00:27:47.688 -> 5.00,5.00,64.67,68.67
// 00:27:47.688 -> 6.00,6.00,80.00,84.33
// 00:27:47.688 -> 7.00,7.00,95.00,99.33
// 00:27:47.688 -> 8.00,8.00,111.00,112.67
// 00:27:47.688 -> 9.00,9.00,125.00,125.00



// 2.00,2.00,6.86,6.86,0.00
// 3.00,3.00,28.00,29.00,-0.44
// 4.00,4.00,41.00,42.00,-0.77
// 5.00,5.00,54.00,55.00,-1.11
// 6.00,6.00,67.00,68.00,-1.15
// 7.00,7.00,76.00,79.00,-1.15
// 8.00,8.00,88.00,88.00,-0.74
// 9.00,9.00,96.00,96.00,-0.71
// 2.00,-2.00,-2.00,-4.00,10.25
// 3.00,-3.00,-2.00,2.00,22.06
// 4.00,-4.00,2.00,0.00,35.39
// 5.00,-5.00,-4.00,0.00,45.27
// 6.00,-6.00,0.00,-4.00,58.27
// 7.00,-7.00,2.00,-4.00,61.77
// 8.00,-8.00,0.00,0.00,67.26
// 9.00,-9.00,0.00,0.00,69.20



// 2.00,2.00,7.35,7.84,0.00
// 3.00,3.00,28.00,29.00,-0.56
// 4.00,4.00,40.00,41.00,-0.64
// 5.00,5.00,53.00,54.00,-1.49
// 6.00,6.00,66.00,68.00,-1.02
// 7.00,7.00,75.00,77.00,-1.27
// 8.00,8.00,87.00,89.00,-1.60
// 9.00,9.00,95.00,95.00,-1.42
// 2.00,-2.00,2.00,0.00,11.00
// 3.00,-3.00,-2.00,2.00,22.58
// 4.00,-4.00,0.00,-2.00,33.63
// 5.00,-5.00,0.00,-4.00,46.45
// 6.00,-6.00,0.00,-4.00,54.92
// 7.00,-7.00,-6.00,2.00,61.29
// 8.00,-8.00,-6.00,2.00,66.93
// 9.00,-9.00,0.00,-2.00,71.77




// left_voltage,right_voltage,left_speed,right_speed, voltageDiff, yawRate_radps, yawRate_dps
// 2.00,2.00,7.84,8.33,0.00
// 3.00,3.00,29.00,30.00,-0.42
// 4.00,4.00,41.00,43.00,-0.36
// 5.00,5.00,54.00,54.00,-0.41
// 6.00,6.00,67.00,67.00,-0.08
// 7.00,7.00,77.00,78.00,-0.15
// 8.00,8.00,88.00,90.00,-0.13
// 9.00,9.00,96.00,97.00,0.18
// 2.00,-2.00,2.00,-2.00,11.64
// 3.00,-3.00,-4.00,2.00,22.02
// 4.00,-4.00,0.00,0.00,35.63
// 5.00,-5.00,2.00,-2.00,47.63
// 6.00,-6.00,0.00,-2.00,58.00
// 7.00,-7.00,-6.00,0.00,61.10
// 8.00,-8.00,-6.00,2.00,68.83
// 9.00,-9.00,-2.00,0.00,73.86




// left_voltage,right_voltage,left_speed,right_speed, voltageDiff, yawRate_radps, yawRate_dps
// 2.00,2.00,7.84,7.84,0.00
// 3.00,3.00,29.00,30.00,-0.73
// 4.00,4.00,41.00,43.00,-0.56
// 5.00,5.00,55.00,56.00,-0.62
// 6.00,6.00,68.00,69.00,-0.56
// 7.00,7.00,78.00,80.00,-0.72
// 8.00,8.00,89.00,90.00,-0.79
// 9.00,9.00,98.00,97.00,-0.49
// 2.00,-2.00,0.00,-2.00,10.86
// 3.00,-3.00,4.00,-2.00,21.95
// 4.00,-4.00,-2.00,0.00,35.89
// 5.00,-5.00,-2.00,2.00,46.87
// 6.00,-6.00,-2.00,2.00,58.66
// 7.00,-7.00,0.00,-2.00,64.01
// 8.00,-8.00,0.00,-2.00,68.51
// 9.00,-9.00,-4.00,0.00,72.83



// left_voltage,right_voltage,left_speed,right_speed, voltageDiff, yawRate_radps, yawRate_dps
// 2.00,2.00,7.84,8.33,-0.00
// 3.00,3.00,28.00,30.00,-0.55
// 4.00,4.00,41.00,43.00,-0.34
// 5.00,5.00,55.00,55.00,-0.29
// 6.00,6.00,67.00,69.00,0.04
// 7.00,7.00,77.00,78.00,-0.33
// 8.00,8.00,89.00,91.00,0.07
// 9.00,9.00,98.00,98.00,0.55
// 2.00,-2.00,0.00,-2.00,11.48
// 3.00,-3.00,2.00,-2.00,23.71
// 4.00,-4.00,0.00,0.00,35.67
// 5.00,-5.00,2.00,-2.00,47.44
// 6.00,-6.00,-2.00,2.00,58.15
// 7.00,-7.00,0.00,0.00,63.76
// 8.00,-8.00,0.00,0.00,71.10
// 9.00,-9.00,0.00,0.00,74.44


// 2.00,2.00,7.84,8.33,-0.00
// 3.00,3.00,28.00,29.00,0.03
// 4.00,4.00,42.00,42.00,0.22
// 5.00,5.00,55.00,56.00,0.07
// 6.00,6.00,68.00,68.00,0.05
// 7.00,7.00,77.00,78.00,0.07
// 8.00,8.00,89.00,90.00,0.02
// 9.00,9.00,98.00,98.00,0.93
// 2.00,-2.00,2.00,0.00,12.28
// 3.00,-3.00,2.00,0.00,24.46
// 4.00,-4.00,2.00,-2.00,36.81
// 5.00,-5.00,0.00,0.00,47.83
// 6.00,-6.00,2.00,-2.00,58.83
// 7.00,-7.00,2.00,-2.00,64.14
// 8.00,-8.00,0.00,-2.00,71.45
// 9.00,-9.00,0.00,-4.00,74.84


