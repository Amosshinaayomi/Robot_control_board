#include "pitches.h"
#include "pins.h"
#include "communications.h"
// #include "comm_protocol.h"

TaskHandle_t commsHandle = NULL;

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

  Serial.println("Hello World");

  dataMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL) {
      Serial.println("Mutex creation failed");
      while (1);
  }
  commSerial.begin(115200, SERIAL_8N1, COMM_UART_RX, COMM_UART_TX);  
  xTaskCreate(commsTask, "commsTask", 4096, NULL, 2, &commsHandle);
  xTaskCreate(F411StartupTask, "F411StartupTask", 4096, NULL, 1, NULL);
  // playNote();
  // delay(200);
}

void loop() {

}
