// comm_protocol.h
#pragma once

#include <stdint.h>

// Packet start byte
#define PKT_START_BYTE      0xAA
// Data type
#define PKT_TYPE_SENSOR 0x01
#define PKT_TYPE_COMMAND    0x02   // command either way
#define PKT_TYPE_LOG                0x03   // logged data packet

// Startup protocol commands (sent as payload of type PKT_TYPE_COMMAND)
#define CMD_STARTUP_REQ     0x10   // S3 → F411: request initialization
#define CMD_STARTUP_ACK     0x11   // F411 → S3: all hardware OK
#define CMD_STARTUP_NACK    0x12   // F411 → S3: hardware init failed (with error code)
#define CMD_RUN             0x13   // S3 → F411: start normal operation

// Power Status data
#define PWR_STATUS          0X14


// Velocity control
#define CMD_SET_V           0x01   // set linear velocity (float m/s)
#define CMD_SET_W           0x02   // set angular velocity (float rad/s)
#define CMD_SET_STRAIGHT    0x03   // enable/disable straight mode (uint8_t 0/1)
#define CMD_EMERGENCY_STOP  0x04   // no parameters

// comm_protocol.h (additions)
#define CMD_START_SPEED_TEST        0x07   // start test sequence (no parameters)
#define CMD_STOP_SPEED_TEST         0x08   // stop test (no parameters)

#define LOG_TYPE_SPEED_TEST 0XA1
#define LOG_TYPE_SENSOR_CAL 0XA2

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

#pragma pack(1)

typedef struct {
    uint32_t timestamp_ms;   // not strictly needed but useful
    float leftVoltage;
    float rightVoltage;
    float leftSpeed;         // ticks/s
    float rightSpeed;
} speed_test_log_t;
#pragma pack()

enum commState {
  WAIT_START,
  WAIT_TYPE,
  WAIT_LEN,
  WAIT_PAYLOAD,
  WAIT_CHECKSUM
};





