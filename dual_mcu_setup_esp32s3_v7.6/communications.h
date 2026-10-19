#pragma once
#include <Arduino.h>
#include "comm_protocol.h"

#define commSerial Serial2


extern bool commsEstablished;
extern bool commsTimeoutExceeded;
extern volatile bool poseDataValid;
extern SemaphoreHandle_t dataMutex;
extern ahrsPacketPacked_t lastestAhrsPacket;
extern pose_packet_packed_t lastestPosePacket;



extern volatile bool startupAckReceived;
extern volatile bool startupNackReceived;
extern volatile uint8_t startupErrorCode;


uint8_t compute_checksum(uint8_t* data, int len); 
void    sendAck(uint8_t type, uint8_t status);
void    sendCommand(uint8_t cmd, const uint8_t* payload, uint8_t len);
void    send_message(uint8_t a, uint8_t b, uint8_t* buf);
void    indicateError(uint8_t code);
void    printlastestAHRSPacket(ahrsPacketPacked_t p);
void    printPose(const pose_packet_packed_t& p);
void    sendVisualizationData(ahrsPacketPacked_t p);


