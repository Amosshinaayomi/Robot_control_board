#include "wifiAsyncServer.h"
#include "communications.h"
#include "pwr_mgmt.h"
#include "comm_protocol.h"
#include <WiFi.h>
#include <ArduinoJson.h>

// ---- Server instances ----
AsyncWebServer server(80);
AsyncWebSocket  ws("/ws");

uint32_t pushIntervalMs = 500;
static uint32_t bootMs  = 0;

//  HTML page
// -------------------------------------------------------------------------
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>Robot Telemetry</title>
  <style>
    body { font-family: system-ui, sans-serif; margin: 2rem; background:#111; color:#eee; }
    h1 { font-size: 1.4rem; }
    h2 { font-size: 1rem; margin-top: 1.5rem; color:#8af; }
    #status { font-weight: bold; }
    .ok  { color: #4ade80; }
    .bad { color: #f87171; }
    button { padding: .5rem 1rem; margin: .2rem .2rem 0 0;
             background:#222; color:#eee; border:1px solid #444; border-radius:4px; cursor:pointer; }
    button:hover { background:#333; }
    .grid { display: grid; grid-template-columns: repeat(3, minmax(0,1fr));
            gap:.3rem 1rem; max-width: 520px; }
    .grid span { color:#888; font-size: .85rem; }
    .grid b { color: #fbbf24; }
    .controls { display: flex; flex-wrap: wrap; gap: .5rem 1rem;
                align-items: end; margin-bottom: .5rem; }
    .controls label { display: flex; flex-direction: column;
                      font-size: .85rem; color:#888; }
    .controls input[type=number] { width: 6.5rem; padding: .3rem;
                       background:#222; color:#eee; border:1px solid #444; border-radius:4px; }
    .controls .row { display: flex; gap: .3rem; }
    .danger { background:#7f1d1d; color:#fff; border-color:#b91c1c; }
    .danger:hover { background:#991b1b; }
    .lbl { font-size:.8rem; color:#666; margin-top: .3rem; }
    .lbl b { color: #fbbf24; }
  </style>
</head>
<body>
  <h1>Robot Telemetry</h1>
  <p>Connection: <span id="status" class="bad">disconnected</span></p>

  <h2>System</h2>
  <p>Uptime:  <b id="uptime">?</b> ms</p>
  <p>Battery: <b id="battery">?</b> V</p>
  <p>Comms:   <b id="comms">?</b></p>

  <h2>Motion Control</h2>
  <div class="controls">
    <label>Linear (m/s)
      <input id="in-lin" type="number" min="-1" max="1" step="0.05" value="0"
             onchange="sendVelocity()">
    </label>
    <label>Angular (&deg;/s)
      <input id="in-ang" type="number" min="-360" max="360" step="1" value="0"
             onchange="sendVelocity()">
    </label>
    <label>Heading (&deg;)
      <input id="in-head" type="number" min="0" max="360" step="1" value="90"
             onchange="sendVelocity()">
    </label>
    <div class="row">
      <button onclick="stopMotion()" class="danger">STOP</button>
    </div>
  </div>
  <div class="lbl">
    lin=<b id="lbl-lin">0.00</b>
    ang(rad/s)=<b id="lbl-ang">0.000</b>
    head=<b id="lbl-head">90</b>
  </div>

  <h2>PID Gains</h2>
  <div class="controls">
    <label>Kp_vel  <input id="in-kpvel"  type="number" step="0.01" value="0.11"></label>
    <label>Ki_vel  <input id="in-kivel"  type="number" step="0.01" value="0.025"></label>
    <label>Kd_vel  <input id="in-kdvel"  type="number" step="0.01" value="0.00"></label>
    <label>Kp_head <input id="in-kphead" type="number" step="0.1"  value="3.5"></label>
    <label>Kp_om   <input id="in-kpom"   type="number" step="0.1"  value="1.6"></label>
    <label>Ki_om   <input id="in-kiom"   type="number" step="0.01" value="0.025"></label>
    <label>Kd_om   <input id="in-kdom"   type="number" step="0.01" value="0.00"></label>
    <div class="row">
      <button onclick="sendGains()">Apply Gains</button>
    </div>
  </div>

  <h2>Pose (F411 state estimator)</h2>
  <p>Valid: <b id="pose-valid">no</b></p>
  <div class="grid">
    <span>X (m)</span><span>Y (m)</span><span>Theta (&deg;)</span>
    <b id="px">?</b><b id="py">?</b><b id="ptheta">?</b>
    <span>v_lin (m/s)</span><span>v_ang (rad/s)</span><span>t (ms)</span>
    <b id="pvlin">?</b><b id="pvang">?</b><b id="pt">?</b>
    <span>Roll (&deg;)</span><span>Pitch (&deg;)</span><span>Yaw (&deg;)</span>
    <b id="proll">?</b><b id="ppitch">?</b><b id="pyaw">?</b>
  </div>

  <h2>AHRS</h2>
  <div class="grid">
    <span>Roll</span><span>Pitch</span><span>Yaw</span>
    <b id="aroll">?</b><b id="apitch">?</b><b id="ayaw">?</b>
    <span>Ax (g)</span><span>Ay (g)</span><span>Az (g)</span>
    <b id="ax">?</b><b id="ay">?</b><b id="az">?</b>
    <span>Gx (&deg;/s)</span><span>Gy (&deg;/s)</span><span>Gz (&deg;/s)</span>
    <b id="gx">?</b><b id="gy">?</b><b id="gz">?</b>
    <span>Enc0</span><span>Enc1</span><span>Enc2</span>
    <b id="e0">?</b><b id="e1">?</b><b id="e2">?</b>
  </div>

  <h2>Update rate</h2>
  <button onclick="send({cmd:'set_interval', value:100})">100 ms</button>
  <button onclick="send({cmd:'set_interval', value:500})">500 ms</button>
  <button onclick="send({cmd:'set_interval', value:1000})">1 s</button>
  <button onclick="send({cmd:'get_state'})">Refresh now</button>

  <script>
    const RAD2DEG = 180 / Math.PI;
    const DEG2RAD = Math.PI / 180;

    let ws = null;
    let reconnectTimer = null;
    let uiPopulated = false;

    function setStatus(text, ok) {
      const el = document.getElementById('status');
      el.textContent = text;
      el.className = ok ? 'ok' : 'bad';
    }

    function connect() {
      const proto = location.protocol === 'https:' ? 'wss://' : 'ws://';
      ws = new WebSocket(proto + location.host + '/ws');

      ws.onopen = () => {
        setStatus('connected', true);
        send({cmd:'get_state'});
      };

      ws.onclose = () => {
        setStatus('disconnected', false);
        scheduleReconnect();
      };

      ws.onerror = () => ws.close();

      ws.onmessage = (evt) => {
        let msg;
        try { msg = JSON.parse(evt.data); }
        catch (e) { console.warn('bad json', evt.data); return; }

        if (msg.error) { console.warn('server error:', msg.error); return; }
        if (msg.type !== 'state') return;

        const set = (id, v) => {
          const el = document.getElementById(id);
          if (el) el.textContent = v;
        };

        // ---- System ----
        set('uptime',  msg.uptime_ms);
        set('battery', (msg.battery_v ?? 0).toFixed(2));
        set('comms',   msg.comms_ok ? 'OK' : 'WAIT');

        // ---- Populate UI fields once on connect ----
        if (!uiPopulated && msg.cmd) {
          document.getElementById('in-lin').value  = msg.cmd.linear.toFixed(2);
          document.getElementById('in-ang').value  = (msg.cmd.angular * RAD2DEG).toFixed(1);
          document.getElementById('in-head').value = msg.cmd.heading.toFixed(0);
          document.getElementById('in-kpvel').value  = msg.cmd.kp_vel.toFixed(3);
          document.getElementById('in-kivel').value  = msg.cmd.ki_vel.toFixed(3);
          document.getElementById('in-kdvel').value  = msg.cmd.kd_vel.toFixed(3);
          document.getElementById('in-kphead').value = msg.cmd.kp_heading.toFixed(2);
          document.getElementById('in-kpom').value   = msg.cmd.kp_omega.toFixed(2);
          document.getElementById('in-kiom').value   = msg.cmd.ki_omega.toFixed(3);
          document.getElementById('in-kdom').value   = msg.cmd.kd_omega.toFixed(3);
          document.getElementById('lbl-lin').textContent  = msg.cmd.linear.toFixed(2);
          document.getElementById('lbl-ang').textContent  = msg.cmd.angular.toFixed(3);
          document.getElementById('lbl-head').textContent = msg.cmd.heading.toFixed(0);
          uiPopulated = true;
        }

        // ---- Pose (F4 sends radians; display in degrees) ----
        const p = msg.pose || {};
        set('pose-valid', p.valid ? 'yes' : 'no');
        if (p.valid) {
          set('px',     (p.x         ?? 0).toFixed(3));
          set('py',     (p.y         ?? 0).toFixed(3));
          set('ptheta', ((p.theta    ?? 0) * RAD2DEG).toFixed(1));
          set('pvlin',  (p.v_linear  ?? 0).toFixed(3));
          set('pvang',  (p.v_angular ?? 0).toFixed(3));
          set('pt',     p.t_ms ?? 0);
          set('proll',  ((p.roll     ?? 0) * RAD2DEG).toFixed(1));
          set('ppitch', ((p.pitch    ?? 0) * RAD2DEG).toFixed(1));
          set('pyaw',   ((p.yaw      ?? 0) * RAD2DEG).toFixed(1));
        }

        // ---- AHRS (already degrees from the library) ----
        const a = msg.ahrs || {};
        set('aroll',  (a.roll  ?? 0).toFixed(1));
        set('apitch', (a.pitch ?? 0).toFixed(1));
        set('ayaw',   (a.yaw   ?? 0).toFixed(1));

        if (a.accel_g) {
          set('ax', a.accel_g[0].toFixed(3));
          set('ay', a.accel_g[1].toFixed(3));
          set('az', a.accel_g[2].toFixed(3));
        }
        if (a.gyro_dps) {
          set('gx', a.gyro_dps[0].toFixed(1));
          set('gy', a.gyro_dps[1].toFixed(1));
          set('gz', a.gyro_dps[2].toFixed(1));
        }
        if (a.enc) {
          set('e0', a.enc[0]);
          set('e1', a.enc[1]);
          set('e2', a.enc[2]);
        }
      };
    }

    function scheduleReconnect() {
      if (reconnectTimer) return;
      reconnectTimer = setTimeout(() => {
        reconnectTimer = null;
        connect();
      }, 2000);
    }

    function send(obj) {
      if (!ws || ws.readyState !== WebSocket.OPEN) return;
      ws.send(JSON.stringify(obj));
    }

    // ---- Velocity ----
    // Browser shows angular in deg/s; send to S3 in deg/s; S3 converts to rad/s.
    function sendVelocity() {
      const lin  = parseFloat(document.getElementById('in-lin').value)  || 0;
      const ang  = parseFloat(document.getElementById('in-ang').value)  || 0;
      const head = parseFloat(document.getElementById('in-head').value) || 0;
      document.getElementById('lbl-lin').textContent  = lin.toFixed(2);
      document.getElementById('lbl-ang').textContent  = (ang * DEG2RAD).toFixed(3);
      document.getElementById('lbl-head').textContent = head.toFixed(0);
      send({ cmd: 'set_velocity', linear: lin, angular: ang, heading: head });
    }

    function stopMotion() {
      document.getElementById('in-lin').value = 0;
      document.getElementById('in-ang').value = 0;
      sendVelocity();
    }

    // ---- Gains ----
    function sendGains() {
      send({
        cmd: 'set_gains',
        kp_vel:     parseFloat(document.getElementById('in-kpvel').value)  || 0,
        ki_vel:     parseFloat(document.getElementById('in-kivel').value)  || 0,
        kd_vel:     parseFloat(document.getElementById('in-kdvel').value)  || 0,
        kp_heading: parseFloat(document.getElementById('in-kphead').value) || 0,
        kp_omega:   parseFloat(document.getElementById('in-kpom').value)   || 0,
        ki_omega:   parseFloat(document.getElementById('in-kiom').value)   || 0,
        kd_omega:   parseFloat(document.getElementById('in-kdom').value)   || 0,
      });
    }

    connect();
  </script>
</body>
</html>
)HTML";

// ---- Static prototypes ----
static void onWsEvent(AsyncWebSocket*, AsyncWebSocketClient*, AwsEventType,
                      void*, uint8_t*, size_t);
static void handleWsMessage(AsyncWebSocketClient*, uint8_t*, size_t);
static void sendState(AsyncWebSocketClient*);

// -------------------------------------------------------------------------
//  updateCmdMirror — called from S3 commsTask when F4 echoes ctrl state.
// -------------------------------------------------------------------------
void updateCmdMirror(const ctrl_state_t& cs) {
    taskENTER_CRITICAL(&motionMux);
    motionCommand.linear  = cs.vel.linear;
    motionCommand.angular = cs.vel.angular;
    motionCommand.heading = cs.vel.heading;

    motionGains.kp_vel     = cs.gains.kp_vel;
    motionGains.ki_vel     = cs.gains.ki_vel;
    motionGains.kd_vel     = cs.gains.kd_vel;
    motionGains.kp_heading = cs.gains.kp_heading;
    motionGains.kp_omega   = cs.gains.kp_omega;
    motionGains.ki_omega   = cs.gains.ki_omega;
    motionGains.kd_omega   = cs.gains.kd_omega;
    taskEXIT_CRITICAL(&motionMux);
}

// -------------------------------------------------------------------------
//  WebSocket event callback
// -------------------------------------------------------------------------
static void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("[WS] #%u connected from %s\n",
                    client->id(), client->remoteIP().toString().c_str());
      sendState(client);
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("[WS] #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA: {
      AwsFrameInfo* info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len
          && info->opcode == WS_TEXT) {
        data[len] = 0;
        handleWsMessage(client, data, len);
      }
      break;
    }
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

// -------------------------------------------------------------------------
//  Command handler — writes shared state only. No serial access here.
// -------------------------------------------------------------------------
static void handleWsMessage(AsyncWebSocketClient* client, uint8_t* data, size_t len) {
  JsonDocument doc;
  if (deserializeJson(doc, data, len)) {
    client->text("{\"error\":\"bad json\"}");
    return;
  }
  const char* cmd = doc["cmd"] | "";
  Serial.printf("[WS] cmd=%s\n", cmd);

  if (strcmp(cmd, "set_interval") == 0) {
    uint32_t ms = doc["value"] | 500;
    if (ms < 100) ms = 100;
    pushIntervalMs = ms;
    broadcastState();
  }
  else if (strcmp(cmd, "get_state") == 0) {
    sendState(client);
  }
  else if (strcmp(cmd, "set_velocity") == 0) {
    float lin  = doc["linear"]  | 0.0f;
    float ang  = (doc["angular"] | 0.0f) * DEG_TO_RAD;   // deg/s → rad/s
    float head = doc["heading"] | 90.0f;

    taskENTER_CRITICAL(&motionMux);
    motionCommand.linear  = lin;
    motionCommand.angular = ang;
    motionCommand.heading = head;
    taskEXIT_CRITICAL(&motionMux);

    broadcastState();
  }
  else if (strcmp(cmd, "set_gains") == 0) {
    taskENTER_CRITICAL(&motionMux);
    motionGains.kp_vel     = doc["kp_vel"]     | motionGains.kp_vel;
    motionGains.ki_vel     = doc["ki_vel"]     | motionGains.ki_vel;
    motionGains.kd_vel     = doc["kd_vel"]     | motionGains.kd_vel;
    motionGains.kp_heading = doc["kp_heading"] | motionGains.kp_heading;
    motionGains.kp_omega   = doc["kp_omega"]   | motionGains.kp_omega;
    motionGains.ki_omega   = doc["ki_omega"]   | motionGains.ki_omega;
    motionGains.kd_omega   = doc["kd_omega"]   | motionGains.kd_omega;
    gainsDirty = true;
    taskEXIT_CRITICAL(&motionMux);

    broadcastState();
  }
  else {
    client->text("{\"error\":\"unknown cmd\"}");
  }
}

// -------------------------------------------------------------------------
//  State JSON
// -------------------------------------------------------------------------
void buildStateJson(String& out) {
  JsonDocument doc;
  doc["type"]      = "state";
  doc["uptime_ms"] = millis() - bootMs;
  doc["interval"]  = pushIntervalMs;
  doc["comms_ok"]  = commsEstablished;

  // ---- Battery ----
  {
    xSemaphoreTake(sysPwrMutex, portMAX_DELAY);
    pwrStatus_t p = sysPwrStatus;
    xSemaphoreGive(sysPwrMutex);
    doc["battery_v"] = p.batteryVoltage;
  }

  // ---- Command mirror (copy fields, not structs) ----
  float   vLin, vAng, vHead;
  float   kpV, kiV, kdV, kpH, kpO, kiO, kdO;
  taskENTER_CRITICAL(&motionMux);
  vLin  = motionCommand.linear;
  vAng  = motionCommand.angular;
  vHead = motionCommand.heading;
  kpV = motionGains.kp_vel;
  kiV = motionGains.ki_vel;
  kdV = motionGains.kd_vel;
  kpH = motionGains.kp_heading;
  kpO = motionGains.kp_omega;
  kiO = motionGains.ki_omega;
  kdO = motionGains.kd_omega;
  taskEXIT_CRITICAL(&motionMux);

  doc["cmd"]["linear"]     = vLin;
  doc["cmd"]["angular"]    = vAng;
  doc["cmd"]["heading"]    = vHead;
  doc["cmd"]["kp_vel"]     = kpV;
  doc["cmd"]["ki_vel"]     = kiV;
  doc["cmd"]["kd_vel"]     = kdV;
  doc["cmd"]["kp_heading"] = kpH;
  doc["cmd"]["kp_omega"]   = kpO;
  doc["cmd"]["ki_omega"]   = kiO;
  doc["cmd"]["kd_omega"]   = kdO;

  // ---- AHRS + Pose ----
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    ahrsPacketPacked_t   a = lastestAhrsPacket;
    pose_packet_packed_t p = lastestPosePacket;
    bool poseOk = poseDataValid;
    xSemaphoreGive(dataMutex);

    doc["ahrs"]["t_ms"]    = a.timestamp_ms;
    doc["ahrs"]["roll"]    = a.roll;
    doc["ahrs"]["pitch"]   = a.pitch;
    doc["ahrs"]["yaw"]     = a.yaw;
    doc["ahrs"]["yawRate"] = a.yawRate;

    doc["ahrs"]["accel_g"][0] = a.accel_g[0];
    doc["ahrs"]["accel_g"][1] = a.accel_g[1];
    doc["ahrs"]["accel_g"][2] = a.accel_g[2];

    doc["ahrs"]["gyro_dps"][0] = a.gyro_dps[0];
    doc["ahrs"]["gyro_dps"][1] = a.gyro_dps[1];
    doc["ahrs"]["gyro_dps"][2] = a.gyro_dps[2];

    doc["ahrs"]["enc"][0] = a.encoder_ticks[0];
    doc["ahrs"]["enc"][1] = a.encoder_ticks[1];
    doc["ahrs"]["enc"][2] = a.encoder_ticks[2];
    doc["ahrs"]["enc"][3] = a.encoder_ticks[3];

    doc["pose"]["valid"] = poseOk;
    if (poseOk) {
      doc["pose"]["t_ms"]      = p.timestamp_ms;
      doc["pose"]["x"]         = p.x;
      doc["pose"]["y"]         = p.y;
      doc["pose"]["theta"]     = p.theta;
      doc["pose"]["v_linear"]  = p.v_linear;
      doc["pose"]["v_angular"] = p.v_angular;
      doc["pose"]["roll"]      = p.roll;
      doc["pose"]["pitch"]     = p.pitch;
      doc["pose"]["yaw"]       = p.yaw;
    }
  } else {
    doc["pose"]["valid"] = false;
  }

  serializeJson(doc, out);
}

static void sendState(AsyncWebSocketClient* client) {
  String json;
  buildStateJson(json);
  client->text(json);
}

void broadcastState() {
  if (ws.count() == 0) return;
  String json;
  buildStateJson(json);
  ws.textAll(json);
}

// -------------------------------------------------------------------------
//  HTML page — use your existing blob, unmodified
// -------------------------------------------------------------------------

// -------------------------------------------------------------------------
//  One-shot setup
// -------------------------------------------------------------------------
void setupWebServer() {
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", INDEX_HTML);
  });

  server.on("/api/state", HTTP_GET, [](AsyncWebServerRequest* req) {
    String json;
    buildStateJson(json);
    req->send(200, "application/json", json);
  });

  server.onNotFound([](AsyncWebServerRequest* req) {
    req->send(404, "text/plain", "Not found");
  });

  server.begin();
  bootMs = millis();
  Serial.println("HTTP server started");
}

// -------------------------------------------------------------------------

