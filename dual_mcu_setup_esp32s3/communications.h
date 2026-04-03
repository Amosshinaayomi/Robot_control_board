#pragma once

#include "comm_protocol.h"
#include "pwr_mgmt.h"

#define commSerial Serial2

// communication flags
bool commsEstablished;
bool commsTimeoutExceeded;

ahrsPacketPacked_t lastestAhrsPacket;
SemaphoreHandle_t dataMutex;

volatile bool startupAckReceived = false;
volatile bool startupNackReceived = false;
volatile uint8_t startupErrorCode = 0;



void printlastestAHRSPacket(ahrsPacketPacked_t data);
void indicateError(uint8_t code);
void sendVisualizationData(ahrsPacketPacked_t data);
// commState state = WAIT_START;

commState currentState = WAIT_START;

byte rx_buffer[256];
int rx_index = 0;
byte rx_len = 0;
byte rx_type = 0;

byte compute_checksum(byte* data, int len) {
    byte sum = 0;
    for (int i = 0; i < len; i++) sum ^= data[i];
    return sum;
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
    Serial.printf("Sent command 0x%02X with %d params\n", cmd, paramLen);

}
void send_message(byte type, byte len, byte* payload) {
    byte header[3] = {0xAA, type, len};
    byte checksum = compute_checksum(header, len) ^ compute_checksum(payload, len);

    commSerial.write(header, 3);
    commSerial.write(payload, len);
    commSerial.write(checksum);
}



void commsTask(void* parameter)
{
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);
    for(;;)
    {
        if(commsEstablished)
        {
            pwrStatus_t currentPwrStatus;
            xSemaphoreTake(sysPwrMutex, portMAX_DELAY);
            currentPwrStatus = sysPwrStatus;
            xSemaphoreGive(sysPwrMutex);

            uint8_t packet[sizeof(pwrStatus_t) + 3];
            uint8_t idx = 0;
            // Packet header, Start byte
            packet[idx++] = PKT_START_BYTE;
            // Packet data type
            packet[idx++] = PWR_STATUS;
            // data type size
            packet[idx++] = sizeof(pwrStatus_t);
            // copy stored data into packet
            memcpy(&packet[idx], &currentPwrStatus, sizeof(pwrStatus_t));
            // increase packet size to contain sensor data struct
            idx += sizeof(pwrStatus_t);

            packet[idx++] = compute_checksum(packet, idx);
            commSerial.write(packet, idx);
            // Serial.println("Power data sent successfully");

        }

        while (commSerial.available()) {  
            byte b = commSerial.read();
            switch(currentState) {
            case WAIT_START: 
                if (b == PKT_START_BYTE) currentState = WAIT_TYPE;
                // Serial.println("packet start");
                break;
            case WAIT_TYPE: 
                rx_type = b;
                // Serial.printf("packet type is 0x%X\n", rx_type);
                currentState = WAIT_LEN;
                break;

            case WAIT_LEN: 
                rx_len = b;
                rx_index = 0;
                if(rx_len == 0)
                {
                currentState = WAIT_CHECKSUM;
                } else if(rx_len <= sizeof(rx_buffer)) {
                currentState = WAIT_PAYLOAD;
                } else {
                currentState = WAIT_START;
                }
                // Serial.printf("packet length is %i\n", rx_len);
                break;
            case WAIT_PAYLOAD: 
                rx_buffer[rx_index++] = b;
                if(rx_index >= rx_len)
                {
                    currentState = WAIT_CHECKSUM;
                }
                // Serial.println("packet payload is added to buffer");
                break;
            case WAIT_CHECKSUM: {
                byte header[3] = {PKT_START_BYTE, rx_type, rx_len};
                byte calculated = compute_checksum(header, 3) ^ compute_checksum(rx_buffer, rx_len);

                if(calculated == b) {
                // Check packet type and parse data
                if(rx_type == PKT_TYPE_SENSOR)
                {
                    // Serial.println("Valid sensor data received");
                    // Serial.println("AHRS SENSOR DATA PACKET IS RECEIVED");
                    ahrsPacketPacked_t receivedPacket;
                    memcpy(&receivedPacket, rx_buffer, sizeof(ahrsPacketPacked_t));               

                    xSemaphoreTake(dataMutex, portMAX_DELAY);
                    lastestAhrsPacket = receivedPacket;
                    xSemaphoreGive(dataMutex);
                    // Copy data from buffer into struct
     
                    // printlastestAHRSPacket(lastestAhrsPacket);
                    // sendVisualizationData(lastestAhrsPacket);

                } else if(rx_type == PKT_TYPE_COMMAND) {
                    Serial.println("Valid command received");
                    if(rx_len >= 1) {
                        uint8_t cmd = rx_buffer[0];
                        if (cmd == CMD_STARTUP_ACK) {
                            startupAckReceived = true;
                            Serial.println("received startup ack");
                        } else if (cmd == CMD_STARTUP_NACK && rx_len >= 2) {
                            startupNackReceived = true;
                            Serial.println("received startup Nack");
                            Serial.printf("errorcode is %i\n", startupErrorCode);
                            startupErrorCode = rx_buffer[1];
                        }
                    }
                } else if (rx_type == PKT_TYPE_LOG && rx_len >= 1) {
                    uint8_t logType = rx_buffer[0];
                    size_t dataLen = rx_len - 1;
                    if (logType == LOG_TYPE_SPEED_TEST && dataLen == sizeof(speed_test_log_t)) {
                        speed_test_log_t log;
                        memcpy(&log, &rx_buffer[1], dataLen);
                        // Optionally print or forward via ESP‑NOW
                        Serial.printf("SPEED_TEST: %.2f,%.2f,%.2f,%.2f\n",
                        log.leftVoltage, log.rightVoltage,
                        log.leftSpeed, log.rightSpeed);
                        speedTestResults.push_back(log);    
                        // Forward over ESP‑NOW (if needed)
                    
                    }
    else {
        Serial.println("Unknown log type or size mismatch");
    }
}
                } else {
                    Serial.println("Checksum error – packet corrupted");
                }
                currentState = WAIT_START;
                break;
            }
            default:
                currentState = WAIT_START;
            }
        }
        vTaskDelayUntil(&lastWake, period);
    }
}

void performStartupHandshake() {
    const int maxRetries = 3;
    const int timeoutMs = 500;
    // startupAckReceived = false;
    // startupNackReceived = false;
    for (int retry = 0; retry < maxRetries; retry++) {
        Serial.println("Sending CMD_STARTUP_REQ");
        sendCommand(CMD_STARTUP_REQ, NULL, 0);
        Serial.printf("Startup request sent (retry %d)\n", retry + 1);

        uint32_t start = millis();
        while (millis() - start < timeoutMs) {
            if (startupAckReceived) {
                Serial.println("F411 startup successful");
                sendCommand(CMD_RUN, NULL, 0);
                commsEstablished = true;
                indicateError(startupErrorCode);
                Serial.println("Sent RUN command");
                return; // success
            } else{
                Serial.println("startup ack wasn't received");
            }
            if (startupNackReceived) {
                Serial.printf("F411 startup failed with error code %d\n", startupErrorCode);
                indicateError(startupErrorCode); // optional
                playNote();
                return; // failure
            }
            vTaskDelay(pdMS_TO_TICKS(50)); // small delay to avoid tight loop
        }
        Serial.println("Timeout, retrying...");
    }
    Serial.println("F411 not responding. Check connection.");
    commsTimeoutExceeded = true;
    indicateError(255); // timeout error
    for(;;)
    {
        rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);  // Red
        vTaskDelay(pdMS_TO_TICKS(400));
        rgbLedWrite(RGB_BUILTIN, 0, 0, 0);  // black
        vTaskDelay(pdMS_TO_TICKS(400));
    }
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
    Serial.printf("Front Left side encoder tick is %i\nFront right side encoder tick is %i\nBack Left side encoder tick is %i\nBack right side encoder tick is %i\n",data.encoder_ticks[0], data.encoder_ticks[1], data.encoder_ticks[2], data.encoder_ticks[3]);  
    Serial.println();    
}

void sendVisualizationData(ahrsPacketPacked_t data)
{
    Serial.print("PITCH:"); Serial.print(data.pitch);
    Serial.print(",ROLL:"); Serial.print(data.roll);
    Serial.print(",YAW:"); Serial.println(data.yaw);       
}