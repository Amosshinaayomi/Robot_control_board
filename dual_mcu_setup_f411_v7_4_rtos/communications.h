
#pragma once
#define SERIAL_RX_BUFFER_SIZE 256
#define SERIAL_TX_BUFFER_SIZE 256
#include "HardwareSerial.h"
#include "comm_protocol.h"

HardwareSerial commSerial(PA3, PA2);


commState state = WAIT_START;
byte rx_buffer[256];
int rx_index = 0;
byte rx_len = 0;
byte rx_type = 0;

byte compute_checksum(byte* data, int len) {
    byte sum = 0;
    for (int i = 0; i < len; i++) sum ^= data[i];
    return sum;
}

void sendAck(uint8_t command, uint8_t status) {
    ack_packet_t ack;
    ack.command = command;
    ack.status = status;
    uint8_t packet[sizeof(ack_packet_t) + 3];
    uint8_t idx = 0;
    packet[idx++] = PKT_START_BYTE;
    packet[idx++] = (status == 0) ? PKT_TYPE_ACK : PKT_TYPE_NACK;
    packet[idx++] = sizeof(ack_packet_t);
    memcpy(&packet[idx], &ack, sizeof(ack_packet_t));
    idx += sizeof(ack_packet_t);
    uint8_t checksum = compute_checksum(packet, idx);
    Serial.printf("computed checksum for sent ack is %i\n", checksum);
    packet[idx++] = checksum;
    Serial.printf("Sending acknowledgement for packet 0X0%X, status %i\n", command, status);
    commSerial.write(packet, idx);
}

void send_message(byte type, byte len, byte* payload) {
    byte header[3] = {PKT_START_BYTE, type, len};
    byte checksum = compute_checksum(header, len + 3);

    commSerial.write(header, 3);
    commSerial.write(payload, len);
    commSerial.write(checksum);
}



void send_log_packet(uint8_t logType, void* data, size_t dataLen) {
    size_t totalPayloadLen = 1 + dataLen;   // logType + data
    uint8_t packet[totalPayloadLen + 3];    // start, type, len, payload, checksum
    uint8_t idx = 0;
    packet[idx++] = PKT_START_BYTE;
    packet[idx++] = PKT_TYPE_LOG;
    packet[idx++] = totalPayloadLen;        // total payload length
    packet[idx++] = logType;                // log type identifier
    memcpy(&packet[idx], data, dataLen);
    idx += dataLen;
    uint8_t checksum = compute_checksum(packet, idx);
    packet[idx++] = checksum;
    commSerial.write(packet, idx);
}



void sendStartupAck() {
    uint8_t packet[5]; // max size for command + param
    uint8_t idx = 0;
    packet[idx++] = PKT_START_BYTE;
    packet[idx++] = PKT_TYPE_COMMAND;
    packet[idx++] = 1;                     // payload length (just command byte)
    packet[idx++] = CMD_STARTUP_ACK;
    uint8_t checksum = compute_checksum(packet, idx);
    packet[idx++] = checksum;
    commSerial.write(packet, idx);
    Serial.println("Sent STARTUP_ACK");
}

void sendStartupNack(uint8_t errorCode) {
    uint8_t packet[6];
    uint8_t idx = 0;
    packet[idx++] = PKT_START_BYTE;
    packet[idx++] = PKT_TYPE_COMMAND;
    packet[idx++] = 2;                     // payload length = command + error code
    packet[idx++] = CMD_STARTUP_NACK;
    packet[idx++] = errorCode;
    uint8_t checksum = compute_checksum(packet, idx);
    packet[idx++] = checksum;
    commSerial.write(packet, idx);
    Serial.printf("Sent STARTUP_NACK with code %d\n", errorCode);
}

// void sendAck(uint8_t command, uint8_t status) {
//     ack_packet_t ack;
//     ack.command = command;
//     ack.status = status;
//     uint8_t packet[sizeof(ack_packet_t) + 3];
//     packet[0] = PKT_START_BYTE;
//     packet[1] = (status == 0) ? PKT_TYPE_ACK : PKT_TYPE_NACK;
//     packet[2] = sizeof(ack_packet_t);
//     memcpy(&packet[3], &ack, sizeof(ack_packet_t));
//     uint8_t checksum = compute_checksum(packet, 3 + sizeof(ack_packet_t)) ^
//                        compute_checksum(&packet[3], sizeof(ack_packet_t));
//     packet[3 + sizeof(ack_packet_t)] = checksum;
//     commSerial.write(packet, 4 + sizeof(ack_packet_t));
// }




void printAHRSPacket( ahrsPacketPacked_t data)
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
    Serial.printf("Front Left side encoder tick is %i\nFront right side encoder tick is %i\nBack Left side encoder tick is %i\nBack right side encoder tick is %i\n",data.encoder_ticks[0], data.encoder_ticks[1], data.encoder_ticks[2], data.encoder_ticks[3]);  
    Serial.println();    
}
