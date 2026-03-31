#include "pitches.h"
#include "pins.h"
#include "communications.h"
#include "pwr_mgmt.h"


TaskHandle_t commsHandle = NULL;
TaskHandle_t powerHandle = NULL;


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
      Serial.printf("batteryADC compounded value is %i is %i\n", adcSum, i);
    }
    float avgAdc = (float)adcSum / samples;
    Serial.printf("Average battery ADCValue is %f\n", avgAdc);

    float batteryVoltage = (avgAdc / 3103.0303) * BATTERY_FULL_VOLT; // 3103 because at max voltage 12.6 Vout is 2.5v

    pwrStatus_t localStatus;
    localStatus.timestamp_ms = millis();
    localStatus.batteryVoltage = batteryVoltage;

    xSemaphoreTake(sysPwrMutex, portMAX_DELAY);
    sysPwrStatus = localStatus;
    xSemaphoreGive(sysPwrMutex);

        Serial.printf("Power: time=%lu, voltage=%.2f\n", localStatus.timestamp_ms, localStatus.batteryVoltage);
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
}

void loop() {
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line == "test_start") {
            sendCommand(CMD_START_TEST, NULL, 0);
            Serial.println("Sent test start to F411");
        }
        else if (line == "test_stop") {
            sendCommand(CMD_STOP_TEST, NULL, 0);
            Serial.println("Sent test stop to F411");
        }
        else {
            Serial.println("Unknown command. Use test_start or test_stop");
        }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
}
