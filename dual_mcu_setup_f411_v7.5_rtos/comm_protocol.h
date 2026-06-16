// comm_protocol.h
#pragma once

#include <stdint.h>

// Packet start byte
#define PKT_START_BYTE      0xAA
// Data type
#define PKT_TYPE_SENSOR     0x01
#define PKT_TYPE_COMMAND    0x02   // command either way
#define PKT_TYPE_LOG        0x03   // logged data packet
#define PKT_TYPE_POSE_DATA  0X04

// Power Status data
#define PWR_STATUS          0X05
// Packets acknowledgements
#define PKT_TYPE_ACK        0x06   // or any unused value
#define PKT_TYPE_NACK       0x07
// Startup protocol commands (sent as payload of type PKT_TYPE_COMMAND)
#define CMD_STARTUP_REQ     0x11   // S3 → F411: request initialization
#define CMD_STARTUP_ACK     0x12   // F411 → S3: all hardware OK
#define CMD_STARTUP_NACK    0x13   // F411 → S3: hardware init failed (with error code)
#define CMD_RUN             0x14   // S3 → F411: start normal operation


// Velocity control
#define CMD_SET_V             0x15   // set linear velocity (float m/s)
#define CMD_SET_W             0x16   // set angular velocity (float rad/s)
#define CMD_SET_STRAIGHT      0x17   // enable/disable straight mode (uint8_t 0/1)
#define CMD_EMERGENCY_STOP    0x18   // no parameters

// comm_protocol.h (additions)
#define CMD_START_SPEED_TEST  0x30   // start test sequence (no parameters)
#define CMD_STOP_SPEED_TEST   0x31   // stop test (no parameters)

#define LOG_TYPE_SPEED_TEST   0XA1
#define LOG_TYPE_SENSOR_CAL   0XA2



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
  float yawRate;

} speed_test_log_t;
#pragma pack()


// Packed for serialization (send over UART)
#pragma pack(1)
typedef struct {
    uint32_t timestamp_ms;
    float x; float y; float theta;
    float v_linear; float v_angular;
    float roll, pitch, yaw;
} pose_packet_packed_t;
#pragma pack()


typedef struct __attribute__((packed)) {
    uint8_t command;        // the command being acknowledged (e.g., CMD_RUN)
    uint8_t status;         // 0 = success, non-zero = error code
} ack_packet_t;


enum commState {
  WAIT_START,
  WAIT_TYPE,
  WAIT_LEN,
  WAIT_PAYLOAD,
  WAIT_CHECKSUM
};





