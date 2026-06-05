#pragma once

#include "comm_protocol.h"
#include "pwr_mgmt.h"

#define commSerial Serial2


volatile bool startupAckReceived = false;
volatile bool startupNackReceived = false;
volatile uint8_t startupErrorCode = 0;

void printlastestAHRSPacket(ahrsPacketPacked_t data);
void indicateError(uint8_t code);
void sendVisualizationData(ahrsPacketPacked_t data);
// commState state = WAIT_START;



byte compute_checksum(byte* data, int len) {
    byte sum = 0;
    for (size_t i = 0; i < len; i++) sum ^= data[i];
    return sum;
}
void sendAck(uint8_t command, uint8_t status) {
    ack_packet_t ack;
    ack.command = command;
    ack.status = status;
    uint8_t packet[sizeof(ack_packet_t) + 3 + 1];
    uint8_t idx = 0;
    packet[idx++] = PKT_START_BYTE;
    packet[idx++] = (status == 0) ? PKT_TYPE_ACK : PKT_TYPE_NACK;
    packet[idx++] = sizeof(ack_packet_t);
    if(command == PKT_TYPE_POSE_DATA || command ==  PKT_TYPE_SENSOR) {
        // Serial.printf("ack packet length for 0X0%X is %i bytes\n", command, sizeof(ack_packet_t));
    }
    memcpy(&packet[idx], &ack, sizeof(ack_packet_t));
    idx += sizeof(ack_packet_t);
    uint8_t checksum = compute_checksum(packet, idx);
    // Serial.printf("computed checksum for sent ack is %i\n", checksum);
    packet[idx++] = checksum;
    // Serial.printf("Sending acknowledgement for packet 0X0%X, status %i\n", command, status);
    commSerial.write(packet, idx);
}

void sendCommand(uint8_t cmd, const uint8_t* params, uint8_t paramLen) {
    uint8_t packet[64];
    uint8_t idx = 0;
    packet[idx++] = PKT_START_BYTE;
    packet[idx++] = PKT_TYPE_COMMAND;
    packet[idx++] = 1 + paramLen;
    packet[idx++] = cmd;
    if(params && paramLen > 0) {
        memcpy(&packet[idx], params, paramLen);
        idx += paramLen;
    }
    uint8_t checksum = compute_checksum(packet, idx);
    packet[idx++] = checksum;
    commSerial.write(packet, idx);
    // Serial.printf("Sent command 0x%02X with %d params\n", cmd, paramLen);

}
void send_message(byte type, byte len, byte* payload) {
    byte header[3] = {0xAA, type, len};
    byte checksum = compute_checksum(header, len);

    commSerial.write(header, 3);
    commSerial.write(payload, len);
    commSerial.write(checksum);
}



void indicateError(uint8_t code) {
    pinMode(LED_BUILTIN, OUTPUT); // or your RGB LED
    switch(code) {
        case 0:
            rgbLedWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);  // Red
            delay(1000);
            break;
        case 1:
            rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);  // Red
            delay(1000);
            break;
        case 2:
            rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, RGB_BRIGHTNESS);  // Red
            delay(1000);
            break;
        case 3:
            rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, RGB_BRIGHTNESS, 0);  // Red
            delay(1000);
            break;
    }
}


void printlastestAHRSPacket(ahrsPacketPacked_t data)
{
    Serial.print("Time stamp: "); Serial.println(data.timestamp_ms);
    Serial.print("Gyro (dps) X,Y,Z: ");
    Serial.print(data.gyro_dps[0], 3); Serial.print(", ");
    Serial.print(data.gyro_dps[1], 3); Serial.print(", ");
    Serial.println(data.gyro_dps[2], 3);
    Serial.print("Accel(g) X,Y,Z: ");
    Serial.print(data.accel_g[0], 3); Serial.print(", ");
    Serial.print(data.accel_g[1], 3); Serial.print(", ");
    Serial.println(data.accel_g[2], 3);

    Serial.print("PITCH:"); Serial.print(data.pitch);
    Serial.print(", ROLL:"); Serial.print(data.roll);
    Serial.print(", YAW:"); Serial.println(data.yaw); 

    Serial.print("YawRate: "); Serial.println(data.yawRate);
    // Serial.printf("Front Left side encoder tick is %i\nFront right side encoder tick is %i\nBack Left side encoder tick is %i\nBack right side encoder tick is %i\n",data.encoder_ticks[0], data.encoder_ticks[1], data.encoder_ticks[2], data.encoder_ticks[3]);  
    Serial.println();    
}

void sendVisualizationData(ahrsPacketPacked_t data)
{
    Serial.print("PITCH:"); Serial.print(data.pitch);
    Serial.print(",ROLL:"); Serial.print(data.roll);
    Serial.print(",YAW:"); Serial.println(data.yaw);       
}

void printPose(const pose_packet_packed_t &pose) {
    // Copy to align (though unpacked struct already aligned)
    uint32_t ts = pose.timestamp_ms;
    float x = pose.x;
    float y = pose.y;
    float theta = pose.theta;
    float v_lin = pose.v_linear;
    float v_ang = pose.v_angular;
    float roll = pose.roll;
    float pitch = pose.pitch;

    Serial.print("Pose: t=");
    Serial.println(ts);
    Serial.print(", x=");
    Serial.print(x, 3);
    Serial.print(", y=");
    Serial.print(y, 3);
    Serial.print(", theta=");
    Serial.print(theta, 3);
    Serial.print(" (");
    Serial.print(theta * RAD_TO_DEG, 1);
    Serial.print("deg), v_lin=");
    Serial.print(v_lin, 3);
    Serial.print(", v_ang=");
    Serial.print(v_ang, 3);
    Serial.print(", roll=");
    Serial.print(roll, 2);
    Serial.print(", pitch=");
    Serial.println(pitch, 2);
}