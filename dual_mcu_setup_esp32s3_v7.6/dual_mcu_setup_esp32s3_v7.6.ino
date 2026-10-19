#include "pitches.h"
#include "pins.h"
#include "communications.h"
#include "pwr_mgmt.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "tof_sensors.h"
#include "wifiAsyncServer.h"
#include "wifi_credentials.h"
#include <WiFi.h>

std::vector<speed_test_log_t> speedTestResults;

volatile bool systemIsReady = false;

TaskHandle_t commsHandle       = NULL;
TaskHandle_t powerHandle       = NULL;
TaskHandle_t distSensorHandle  = NULL;

// NVS storage
nvs_handle_t nvsHandle;
String speedTestResult;

SemaphoreHandle_t sysPwrMutex;
pwrStatus_t       sysPwrStatus;

// ---- Spinlock protecting motionCommand / motionGains / gainsDirty ----
portMUX_TYPE motionMux = portMUX_INITIALIZER_UNLOCKED;

volatile cmd_velocity_t motionCommand = {
    .linear  = 0.0f,
    .angular = 0.0f,
    .heading = 90.0f,
};

volatile cmd_gains_t motionGains = {
    .kp_vel     = 0.11f,
    .ki_vel     = 0.025f,
    .kd_vel     = 0.0f,
    .kp_heading = 3.5f,
    .kp_omega   = 1.6f,
    .ki_omega   = 0.025f,
    .kd_omega   = 0.0f,
};

volatile bool gainsDirty = false;

// -------------------------------------------------------------------------
//  motionPublisherTask
//  Owns the value of motionCommand. For now: if no browser is connected,
//  zero the linear/angular commands so the robot doesn't keep driving.
//  Later this is where motion planning / setpoint ramping will live.
//  Does NOT touch serial — commsTask is the sole UART owner.
// -------------------------------------------------------------------------
void motionPublisherTask(void *parameters)
{
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(50);   // 20 Hz

    for (;;) {
        if (ws.count() == 0) {
            taskENTER_CRITICAL(&motionMux);
            motionCommand.linear  = 0.0f;
            motionCommand.angular = 0.0f;
            // heading left alone
            taskEXIT_CRITICAL(&motionMux);
        }

        // future hook: setpoint ramping, waypoint following, pure pursuit
        // writes into motionCommand here.

        vTaskDelayUntil(&lastWake, period);
    }
}

// -------------------------------------------------------------------------
//  wifiTask
// -------------------------------------------------------------------------
void wifiTask(void *parameters)
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.print('.');
    }
    Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());

    setupWebServer();

    for (;;) {
        broadcastState();
        ws.cleanupClients();
        uint32_t period = pushIntervalMs;
        if (period < 100) period = 100;
        vTaskDelay(pdMS_TO_TICKS(period));
    }
}

// -------------------------------------------------------------------------
//  distSensorTask
// -------------------------------------------------------------------------
void distSensorTask(void *parameters)
{
    while (!systemIsReady) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    bool ioInit = ioExpander.begin(0);
    if (ioInit) {
        for (int i = 0; i < DIST_SEN_NUMB; i++) {
            ioExpander.write(XSHUT_PINS[i], LOW);
        }
        for (int i = 0; i < DIST_SEN_NUMB - 1; i++) {
            Serial.print("\nInitializing sensor ");
            Serial.println(i);

            ioExpander.write(XSHUT_PINS[i], HIGH);
            vTaskDelay(5 / portTICK_PERIOD_MS);

            if (!tof_sensors[i].begin(SENSOR_ADDRESSES[i])) {
                Serial.println(" FAILED!");
                Serial.print("  Could not initialize sensor at address 0x");
                Serial.println(SENSOR_ADDRESSES[i], HEX);
            }

            Serial.println("Success");
            tof_sensors[i].configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED);
            tof_sensors[i].startRangeContinuous(UPDATE_INTERVAL_MS);
        }
    }
    for (;;) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// -------------------------------------------------------------------------
//  powerTask
// -------------------------------------------------------------------------
void powerTask(void *parameters)
{
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100);

    for (;;) {
        uint32_t adcSum = 0;
        const uint8_t samples = 5;
        for (uint8_t i = 0; i < samples; i++) {
            adcSum += analogRead(BATTERY_PIN);
        }
        float avgAdc = (float)adcSum / samples;
        float batteryVoltage = (avgAdc / 3000) * BATTERY_FULL_VOLT;

        pwrStatus_t localStatus;
        localStatus.timestamp_ms = millis();
        localStatus.batteryVoltage = batteryVoltage;

        xSemaphoreTake(sysPwrMutex, portMAX_DELAY);
        sysPwrStatus = localStatus;
        xSemaphoreGive(sysPwrMutex);

        vTaskDelayUntil(&lastWake, period);
    }
}

// -------------------------------------------------------------------------
//  F411StartupTask — runs once, then deletes itself
// -------------------------------------------------------------------------
void F411StartupTask(void *parameters)
{
    performStartupHandshake();
    vTaskDelete(NULL);
}

void performStartupHandshake() {
    const int maxRetries = 3;
    const int timeoutMs = 500;
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
                return;
            } else {
                Serial.println("startup ack wasn't received");
            }
            if (startupNackReceived) {
                Serial.printf("F411 startup failed with error code %d\n", startupErrorCode);
                indicateError(startupErrorCode);
                playNote();
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        Serial.println("Timeout, retrying...");
    }
    Serial.println("F411 not responding. Check connection.");
    commsTimeoutExceeded = true;
    indicateError(255);
    for (;;) {
        rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(400));
        rgbLedWrite(RGB_BUILTIN, 0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(400));
    }
}

// -------------------------------------------------------------------------
//  commsTask — sole owner of commSerial
// -------------------------------------------------------------------------
void commsTask(void* parameter)
{
    commState currentState = WAIT_START;

    byte rx_buffer[256];
    int  rx_index = 0;
    byte rx_len   = 0;
    byte rx_type  = 0;

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);   // 50 Hz

    for (;;) {
        if (commsEstablished) {
            // ---------- Send power status ----------
            pwrStatus_t currentPwrStatus;
            xSemaphoreTake(sysPwrMutex, portMAX_DELAY);
            currentPwrStatus = sysPwrStatus;
            xSemaphoreGive(sysPwrMutex);

            uint8_t pwrPkt[sizeof(pwrStatus_t) + 3 + 1];
            uint8_t pi = 0;
            pwrPkt[pi++] = PKT_START_BYTE;
            pwrPkt[pi++] = PWR_STATUS;
            pwrPkt[pi++] = sizeof(pwrStatus_t);
            memcpy(&pwrPkt[pi], &currentPwrStatus, sizeof(pwrStatus_t));
            pi += sizeof(pwrStatus_t);
            pwrPkt[pi++] = compute_checksum(pwrPkt, pi);
            commSerial.write(pwrPkt, pi);

            // ---------- Send velocity to F4 (20 Hz) ----------
            static uint32_t lastVelSendMs = 0;
            if (millis() - lastVelSendMs >= 50) {
                lastVelSendMs = millis();

                cmd_velocity_t v;
                taskENTER_CRITICAL(&motionMux);
                v.linear  = motionCommand.linear;
                v.angular = motionCommand.angular;
                v.heading = motionCommand.heading;
                taskEXIT_CRITICAL(&motionMux);

                uint8_t velPkt[3 + sizeof(cmd_velocity_t) + 1];
                uint8_t vi = 0;
                velPkt[vi++] = PKT_START_BYTE;
                velPkt[vi++] = PKT_TYPE_VELOCITY;
                velPkt[vi++] = sizeof(cmd_velocity_t);
                memcpy(&velPkt[vi], &v, sizeof(v));
                vi += sizeof(v);
                velPkt[vi++] = compute_checksum(velPkt, vi);
                commSerial.write(velPkt, vi);
            }

            // ---------- Send gains only when dirty ----------
            if (gainsDirty) {
                cmd_gains_t g;
                taskENTER_CRITICAL(&motionMux);
                g.kp_vel     = motionGains.kp_vel;
                g.ki_vel     = motionGains.ki_vel;
                g.kd_vel     = motionGains.kd_vel;
                g.kp_heading = motionGains.kp_heading;
                g.kp_omega   = motionGains.kp_omega;
                g.ki_omega   = motionGains.ki_omega;
                g.kd_omega   = motionGains.kd_omega;
                gainsDirty   = false;
                taskEXIT_CRITICAL(&motionMux);

                sendCommand(CMD_SET_GAINS, (uint8_t*)&g, sizeof(g));
            }
        }

        // ---------- Parse incoming UART ----------
        while (commSerial.available()) {
            byte b = commSerial.read();
            switch (currentState) {
            case WAIT_START:
                if (b == PKT_START_BYTE) currentState = WAIT_TYPE;
                break;
            case WAIT_TYPE:
                rx_type = b;
                currentState = WAIT_LEN;
                break;
            case WAIT_LEN:
                rx_len = b;
                rx_index = 0;
                if (rx_len == 0) {
                    currentState = WAIT_CHECKSUM;
                } else if (rx_len <= sizeof(rx_buffer)) {
                    currentState = WAIT_PAYLOAD;
                } else {
                    currentState = WAIT_START;
                }
                break;
            case WAIT_PAYLOAD:
                rx_buffer[rx_index++] = b;
                if (rx_index >= rx_len) {
                    currentState = WAIT_CHECKSUM;
                }
                break;
            case WAIT_CHECKSUM: {
                byte header[3] = {PKT_START_BYTE, rx_type, rx_len};
                uint8_t combined[3 + rx_len];
                memcpy(combined, header, 3);
                memcpy(combined + 3, rx_buffer, rx_len);
                uint8_t calculated = compute_checksum(combined, 3 + rx_len);

                if (calculated == b) {
                    if (rx_type == PKT_TYPE_SENSOR) {
                        sendAck(PKT_TYPE_SENSOR, 0);
                        ahrsPacketPacked_t receivedPacket;
                        memcpy(&receivedPacket, rx_buffer, sizeof(ahrsPacketPacked_t));
                        xSemaphoreTake(dataMutex, portMAX_DELAY);
                        lastestAhrsPacket = receivedPacket;
                        xSemaphoreGive(dataMutex);
                    }
                    else if (rx_type == PKT_TYPE_COMMAND) {
                        if (rx_len >= 1) {
                            uint8_t cmd = rx_buffer[0];
                            if (cmd == CMD_STARTUP_ACK) {
                                startupAckReceived = true;
                            } else if (cmd == CMD_STARTUP_NACK && rx_len >= 2) {
                                startupNackReceived = true;
                                startupErrorCode = rx_buffer[1];
                            }
                        }
                    }
                    else if (rx_type == PKT_TYPE_LOG && rx_len >= 1) {
                        uint8_t logType = rx_buffer[0];
                        size_t dataLen = rx_len - 1;
                        if (logType == LOG_TYPE_SPEED_TEST && dataLen == sizeof(speed_test_log_t)) {
                            speed_test_log_t log;
                            sendAck(LOG_TYPE_SPEED_TEST, 0);
                            memcpy(&log, &rx_buffer[1], dataLen);
                            Serial.printf("SPEED_TEST: %.2f,%.2f,%.2f,%.2f\n",
                                          log.leftVoltage, log.rightVoltage,
                                          log.leftSpeed, log.rightSpeed);
                            speedTestResults.push_back(log);
                        } else {
                            Serial.println("Unknown log type or size mismatch");
                        }
                    }
                    else if (rx_type == PKT_TYPE_POSE_DATA) {
                        sendAck(PKT_TYPE_POSE_DATA, 0);
                        pose_packet_packed_t rxPose;
                        memcpy(&rxPose, rx_buffer, sizeof(pose_packet_packed_t));
                        xSemaphoreTake(dataMutex, portMAX_DELAY);
                        lastestPosePacket = rxPose;
                        poseDataValid = true;
                        xSemaphoreGive(dataMutex);
                    }
                    else if (rx_type == PKT_TYPE_CTRL_STATE) {
                        if (rx_len == sizeof(ctrl_state_t)) {
                            ctrl_state_t cs;
                            memcpy(&cs, rx_buffer, sizeof(cs));
                            updateCmdMirror(cs);
                        }
                    }
                    else if (rx_type == PKT_TYPE_ACK) {
                        // silent — high-rate acks are noise
                    }
                    else if (rx_type == PKT_TYPE_NACK) {
                        // silent
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

// -------------------------------------------------------------------------
//  setup
// -------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    nvs_flash_init();
    nvs_open("storage", NVS_READWRITE, &nvsHandle);

    pinMode(BATTERY_PIN, INPUT);

    dataMutex   = xSemaphoreCreateMutex();
    sysPwrMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL) {
        Serial.println("Mutex creation failed");
        while (1);
    }

    commSerial.begin(115200, SERIAL_8N1, COMM_UART_RX, COMM_UART_TX);

    xTaskCreate(commsTask,            "commsTask",         4096, NULL, 3, &commsHandle);
    xTaskCreate(powerTask,            "powerTask",         4096, NULL, 2, &powerHandle);
    // xTaskCreate(distSensorTask,    "distSensorTask",    4096, NULL, 2, &distSensorHandle);
    xTaskCreate(wifiTask,             "wifiTask",          4096, NULL, 1, NULL);
    xTaskCreate(motionPublisherTask,  "motionPublisher",   2048, NULL, 2, NULL);
    xTaskCreate(F411StartupTask,      "F411StartupTask",   4096, NULL, 1, NULL);

    playNote();
}

// -------------------------------------------------------------------------
//  loop
// -------------------------------------------------------------------------
void loop() {
    systemIsReady = true;
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line == "test_start") {
            sendCommand(CMD_START_SPEED_TEST, NULL, 0);
            Serial.println("Sent test start to F411");
        }
        else if (line == "test_stop") {
            sendCommand(CMD_STOP_SPEED_TEST, NULL, 0);
            Serial.println("Sent test stop to F411");
        }
        else if (line == "print_results") {
            Serial.println("left_voltage,right_voltage,left_speed,right_speed,yawRate_dps");
            for (size_t i = 0; i < speedTestResults.size(); i++) {
                Serial.printf("%.2f,%.2f,%.2f,%.2f,%.2f\n",
                              speedTestResults[i].leftVoltage,
                              speedTestResults[i].rightVoltage,
                              speedTestResults[i].leftSpeed,
                              speedTestResults[i].rightSpeed,
                              speedTestResults[i].yawRate_dps);
            }
            speedTestResults.clear();
        }
        else {
            Serial.println("Unknown command. Use test_start, test_stop, print_results");
        }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
}