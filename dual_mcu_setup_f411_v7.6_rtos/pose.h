#pragma once
// Normal (natural alignment) for internal use
typedef struct {
    uint32_t timestamp_ms;
    float x; float y; float theta;
    float v_linear; float v_angular;
    float roll, pitch, yaw;
} pose_packet_t;
