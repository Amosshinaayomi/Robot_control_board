#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include "comm_protocol.h"

extern AsyncWebServer server;
extern AsyncWebSocket  ws;
extern uint32_t        pushIntervalMs;

extern portMUX_TYPE motionMux;


// ---- Shared command state (defined in main.ino) ----
extern volatile cmd_velocity_t motionCommand;   // WS writes, publisher manages, commsTask sends
extern volatile cmd_gains_t    motionGains;     // WS writes, commsTask sends
extern volatile bool           gainsDirty;      // WS sets, commsTask clears after send

void setupWebServer();
void buildStateJson(String& out);
void broadcastState();
void updateCmdMirror(const ctrl_state_t& cs);