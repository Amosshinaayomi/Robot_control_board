// comm_protocol.h
#pragma once

#include <stdint.h>

// Packet start byte
#define PKT_START_BYTE      0xAA
// Data type
#define PKT_TYPE_SENSOR 0x01
#define PKT_TYPE_COMMAND    0x02   // command either way


// Startup protocol commands (sent as payload of type PKT_TYPE_COMMAND)
#define CMD_STARTUP_REQ     0x10   // S3 → F411: request initialization
#define CMD_STARTUP_ACK     0x11   // F411 → S3: all hardware OK
#define CMD_STARTUP_NACK    0x12   // F411 → S3: hardware init failed (with error code)
#define CMD_RUN             0x13   // S3 → F411: start normal operation

// Power Status data
#define PWR_STATUS          0X14


// comm_protocol.h (additions)
#define CMD_SET_LEFT_VOLTAGE   0x05   // set left motor voltage (float, volts)
#define CMD_SET_RIGHT_VOLTAGE  0x06   // set right motor voltage (float, volts)

// Velocity control
#define CMD_SET_V           0x01   // set linear velocity (float m/s)
#define CMD_SET_W           0x02   // set angular velocity (float rad/s)
#define CMD_SET_STRAIGHT    0x03   // enable/disable straight mode (uint8_t 0/1)
#define CMD_EMERGENCY_STOP  0x04   // no parameters


#pragma pack(1)
struct ahrsPacketPacked_t {
  unsigned long timestamp_ms;
  float roll, pitch, yaw;
  float accel_g[3];
  float gyro_dps[3];
  float yawRate;
  int32_t encoder_ticks[4];
};
#pragma pack()

#pragma pack(1)
typedef struct {
  unsigned long timestamp_ms;
  float batteryVoltage;
  float systemCurrent;
} pwrStatus_t;
#pragma pack()



enum commState {
  WAIT_START,
  WAIT_TYPE,
  WAIT_LEN,
  WAIT_PAYLOAD,
  WAIT_CHECKSUM
};





