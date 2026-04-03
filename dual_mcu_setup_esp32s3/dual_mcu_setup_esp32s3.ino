#include "pitches.h"
#include "pins.h"
#include "communications.h"
#include "pwr_mgmt.h"
#include "nvs_flash.h"
#include "nvs.h"


TaskHandle_t commsHandle = NULL;
TaskHandle_t powerHandle = NULL;

// NVS storage
nvs_handle_t nvsHandle;
String speedTestResult;
// const String speedTestResult = R"({
//   "leftVoltage":"0",  
//   "rightVoltage":"0",
//   "leftSpeed":"0",
//   "rightSpeed":"0",
//   "timestamp : 0"
// })";

void powerTask(void *parameters)
{
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1);
  
  for(;;)
  {
    uint32_t adcSum = 0;
    const uint8_t samples = 16;
    for(uint8_t i = 0; i < samples; i++)
    {
      adcSum += analogRead(BATTERY_PIN);
      // Serial.printf("batteryADC compounded value is %i is %i\n", adcSum, i);
    }
    float avgAdc = (float)adcSum / samples;
    // Serial.printf("Average battery ADCValue is %f\n", avgAdc);

    float batteryVoltage = (avgAdc / 3103.0303) * BATTERY_FULL_VOLT; // 3103 because at max voltage 12.6 Vout is 2.5v

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
  vTaskDelay(pdMS_TO_TICKS(200));
  performStartupHandshake();
  vTaskDelete(NULL);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  nvs_flash_init();
  nvs_open("storage", NVS_READWRITE, &nvsHandle);
  // while(!Serial){}
  delay(100);
  Serial.println("Hello World");
  pinMode(BATTERY_PIN, INPUT);
  // Creations of mutexes
  dataMutex =  xSemaphoreCreateMutex();
  sysPwrMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL) {
      Serial.println("Mutex creation failed");
      while (1);
  }
  commSerial.begin(115200, SERIAL_8N1, COMM_UART_RX, COMM_UART_TX);  
  xTaskCreate(commsTask, "commsTask", 4096, NULL, 2, &commsHandle);
  xTaskCreate(powerTask, "powerTask", 4096, NULL, 1, &powerHandle);
  xTaskCreate(F411StartupTask, "F411StartupTask", 4096, NULL, 1, NULL);
  // playNote();
  // delay(200);
  // sendCommand(CMD_START_SPEED_TEST, NULL, 0);
}

void loop() {
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
            Serial.println("left_voltage,right_voltage,left_speed,right_speed");
            for (size_t i = 0; i < speedTestResults.size(); i++) {
                Serial.printf("%.2f,%.2f,%.2f,%.2f\n",
                    speedTestResults[i].leftVoltage,
                    speedTestResults[i].rightVoltage,
                    speedTestResults[i].leftSpeed,
                    speedTestResults[i].rightSpeed);
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