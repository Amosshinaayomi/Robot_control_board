#pragma once

#pragma pack(1)
typedef struct {
  unsigned long timestamp_ms;
  float batteryVoltage;
  float systemCurrent;
} pwrStatus_t;
#pragma pack()


// Power system 
extern SemaphoreHandle_t sysPwrMutex;
extern pwrStatus_t       sysPwrStatus;