/*
 *  8-Sensor Line Follower — ESP32 + DRV8833  v5.2 "SLAM-Lite"
 * ============================================================
 *  IR:     IR2→36  IR3→39  IR4→34  IR5→35  IR6→32  IR7→33
 *  Motors: IN1→23  IN2→19 (Left)   IN3→13  IN4→27 (Right)
 *  Batt:   GPIO35 via voltage divider (100k/100k → ×2 ratio)
 *
 *  NEW in v5.1:
 *    · Line-lost recovery (spin in last-known direction)
 *    · Lap timer & counter (lap mode — no auto-stop)
 *    · OTA firmware update via /update HTTP endpoint
 *    · Battery voltage monitor (ADC + alert threshold)
 *    · Live PID error graph (scrolling canvas)
 *    · Motor PWM gauges (L/R live)
 *    · Settings profiles (save/load named presets)
 *    · SLAM-Lite (Track mapping, predictive braking, 2D visualizer)
 *
 *  Libraries: WebSockets by Markus Sattler, ArduinoJson by Blanchon
 * ============================================================
 */

#include <ArduinoJson.h>
#include <Preferences.h>
#include <Update.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <Wire.h>

// ── MPU6050 ──────────────────────────────────────────────
const int MPU_ADDR = 0x68;
float gyroZ = 0.0f;    // deg/s
float accelY = 0.0f;   // g
float yawAngle = 0.0f; // degrees
unsigned long lastMpuTime = 0;

bool mpuPresent = false;
bool driftAssistEnable = false;
float driftAssistGain = 10.0f;

bool gyroTurnEnable = false;
bool isGyroTurning = false;

bool autoTuneActive = false;

// ── WiFi ─────────────────────────────────────────────────
const char *SSID = "AEGIS";
const char *PASSWORD = "sagar321";

// ── IR sensor pins ───────────────────────────────────────
const int IR_PINS[6] = {36, 39, 34, 35, 32, 33};
const int NUM_SENSORS = 6;
const int SENSOR_WEIGHTS[6] = {-2500, -1500, -500, 500, 1500, 2500};

// ── Motor pins ───────────────────────────────────────────
#define MOTOR_L_IN1 23
#define MOTOR_L_IN2 19
#define MOTOR_R_IN3 13
#define MOTOR_R_IN4 27
#define PWM_L1_CH 0
#define PWM_L2_CH 1
#define PWM_R3_CH 2
#define PWM_R4_CH 3
#define PWM_FREQ 20000
#define PWM_RES 8

// ── Battery ADC ──────────────────────────────────────────
#define BATT_PIN 35
#define BATT_DIVIDER 2.0f // 100k/100k divider → ×2
#define BATT_SAMPLE_MS 2000
float battVoltage = 0.0f;
float battAlertThreshold = 3.5f; // alert below this (V)
bool battLow = false;
unsigned long lastBattRead = 0;

// ── PID ──────────────────────────────────────────────────
float Kp = 0.4f, Ki = 0.0f, Kd = 1.2f;
float pidError = 0, pidLastError = 0, pidIntegral = 0;
int lastPosition = 0;

// ── Tracked motor PWM ────────────────────────────────────
int lastPwmL = 0, lastPwmR = 0;

// ── Speed ────────────────────────────────────────────────
int BASE_SPEED = 150, MAX_SPEED = 200, MIN_SPEED = 40;

// ── Turn control ─────────────────────────────────────────
float TURN_SHARPNESS = 1.0f;
int TURN_BRAKE = 0;

// ── Hairpin / curve assist ────────────────────────────────
float CURVE_EXP = 1.0f;
int TURN_SPEED_REDUCE = 0;
int PIVOT_THRESHOLD = 0;

// ── Corner detection ─────────────────────────────────────
int CORNER_SENSITIVITY = 0;
float CORNER_BOOST = 2.0f;
int CORNER_DURATION_MS = 200;
int CORNER_SPEED_PCT = 50;
float errorDerivative = 0;
bool cornerActive = false;
unsigned long cornerStartMs = 0;

// ── Line-lost recovery ────────────────────────────────────
// When all sensors lose the line, spin in the last-error direction.
// Timeout: give up and stop after LINE_LOST_TIMEOUT ms.
bool lineLost = false;
unsigned long lineLostStart = 0;
int LINE_LOST_TIMEOUT = 1500; // ms, 0 = disabled

// ── Lap timer & counter ───────────────────────────────────
// lapTimerMode: if true, finish line crossing counts a lap (no auto-stop).
// If false, behaves like v5 (stop on finish line).
bool lapTimerMode = false;
int lapCount = 0;
unsigned long lapStartMs = 0;
#define MAX_LAP_TIMES 10
unsigned long lapTimes[MAX_LAP_TIMES];

// ── Calibration ──────────────────────────────────────────
int calMin[NUM_SENSORS], calMax[NUM_SENSORS];
bool sensorValid[NUM_SENSORS];
bool isCalibrated = false;
bool calibrating = false;
unsigned long calStart = 0;
const unsigned long CAL_DURATION = 8000;

// ── Sensor data ──────────────────────────────────────────
int rawVals[NUM_SENSORS], normVals[NUM_SENSORS];

// ── Line color mode ──────────────────────────────────────
bool invertLine = false;

// ── State ────────────────────────────────────────────────
bool robotRunning = false;

// ── End-zone detection ───────────────────────────────────
bool endZoneEnable = true;
int endZoneMinSensors = 5;
int endZoneConfirmMs = 100;
int endZoneThreshold = 500;
unsigned long endZoneFirstSeen = 0;
bool endZonePending = false;
bool endZoneTriggered = false;

// ── Track Mapping & Speed Run (SLAM-lite) ─────────────────
enum SegmentType { SEG_STRAIGHT, SEG_TURN_L, SEG_TURN_R };
struct TrackSegment {
  SegmentType type;
  unsigned long durationMs;
};
#define MAX_SEGMENTS 100
TrackSegment trackMap[MAX_SEGMENTS];
int segmentCount = 0;
bool trackMapped = false; // True if lap 1 mapping is complete

// Tuning parameters
bool mappingEnabled = false;
bool speedRunEnabled = false;
int speedRunMax = 255;
int preBrakeMs = 150;
int preBrakeSpeed = 80;

// Internal runtime state
int currentSegmentIdx = 0;
unsigned long segmentStartMs = 0;
SegmentType currentGyStateType = SEG_STRAIGHT;

// ── Web servers ──────────────────────────────────────────
WebServer httpServer(80);
WebSocketsServer wsServer(81);
Preferences prefs;
unsigned long lastWsBroadcast = 0;
const unsigned long WS_INTERVAL = 50;

#define MAX_PROFILES 8

// ── Function Prototypes ──────────────────────────────────
String buildStateJson();
String buildSensorJson();
String buildLogJson();
String listProfilesJson();
void wsLog(const String &msg);
void evaluateSensors();
void updateCalibration();
void readBattery();
bool detectEndZone();
void runPID();
void saveAll();
void loadAll();
void saveProfile(int slot, const String &name);
void loadProfile(int slot);
void deleteProfile(int slot);
void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);

// ══════════════════════════════════════════════════════════
//  HTML Dashboard (placeholder — replaced at build time)
// ══════════════════════════════════════════════════════════
// HTML_PLACEHOLDER_START
const char INDEX_HTML[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Line Follower v5.2</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,sans-serif;background:#111;color:#ddd;padding:16px}
h1{font-size:1.2rem;color:#7dd3fc;margin-bottom:14px}
.card{background:#1c1c1c;border:1px solid #2a2a2a;border-radius:10px;padding:14px;margin-bottom:12px}
.card h2{font-size:.75rem;text-transform:uppercase;letter-spacing:.1em;color:#555;margin-bottom:10px}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-bottom:8px}
button{padding:8px 16px;border:none;border-radius:6px;font-size:.85rem;cursor:pointer;font-weight:700}
button:disabled{opacity:.35;cursor:not-allowed}
.bg{background:#22c55e;color:#000}.br{background:#ef4444;color:#fff}
.by{background:#f59e0b;color:#000}.bb{background:#3b82f6;color:#fff}
.bp{background:#a855f7;color:#fff}.bd{background:#333;color:#aaa}
.status{display:flex;align-items:center;gap:7px;font-size:.85rem;margin-bottom:10px}
.dot{width:10px;height:10px;border-radius:50%;background:#444}
.dot.on{background:#22c55e;box-shadow:0 0 6px #22c55e}
.dot.cal{background:#f59e0b;animation:blink 1s infinite}
.dot.off{background:#ef4444}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.2}}
label{font-size:.82rem;color:#999;display:flex;flex-direction:column;gap:4px;min-width:155px}
label small{font-size:.7rem;color:#555}
input[type=range]{width:100%;accent-color:#7dd3fc}
input[type=text]{background:#111;border:1px solid #333;border-radius:5px;color:#ddd;padding:5px 8px;font-size:.82rem}
.val{color:#7dd3fc;font-weight:700;font-size:.88rem}
.sensors{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:6px}
.sb{display:flex;flex-direction:column;align-items:center;gap:3px}
.bwrap{height:72px;width:28px;background:#222;border-radius:4px;overflow:hidden;display:flex;align-items:flex-end;border:1px solid #2a2a2a}
.bar{width:100%;transition:height .07s;border-radius:3px 3px 0 0}
.bar.norm{background:#7dd3fc}.bar.raw{background:#f59e0b}
.sl{font-size:.6rem;color:#555}.sn{font-size:.7rem;color:#aaa}.sn.raw{color:#f59e0b}
.pos-track{position:relative;height:24px;background:#222;border-radius:12px;overflow:hidden;border:1px solid #2a2a2a}
.pos-dot{position:absolute;top:2px;width:20px;height:20px;background:#7dd3fc;border-radius:50%;transition:left .07s;transform:translateX(-50%)}
#calBar{height:5px;background:#2a2a2a;border-radius:3px;margin-top:8px;overflow:hidden}
#calFill{height:100%;background:#f59e0b;width:0%;transition:width .3s}
.hint{font-size:.7rem;color:#444;margin-top:5px}
#logBox{background:#0a0a0a;border:1px solid #222;border-radius:7px;height:180px;overflow-y:auto;padding:8px;font-family:monospace;font-size:.7rem;color:#5a9a5a}
.ll{padding:1px 0;border-bottom:1px solid #111;white-space:pre-wrap;word-break:break-all}
.ll.w{color:#d97706}.ll.e{color:#dc2626}.ll.i{color:#60a5fa}
.toggle-row{display:flex;align-items:center;gap:12px;margin-bottom:10px}
.toggle-wrap{position:relative;display:inline-flex;align-items:center;gap:10px}
.toggle-track{position:relative;width:52px;height:26px;border-radius:13px;cursor:pointer;background:#222;border:1px solid #3a3a3a;transition:background .3s,border-color .3s}
.toggle-track.black-mode{background:#1a1a1a;border-color:#3a3a3a}
.toggle-track.white-mode{background:#d1d5db;border-color:#9ca3af}
.toggle-thumb{position:absolute;top:3px;left:3px;width:18px;height:18px;border-radius:50%;background:#555;transition:left .25s cubic-bezier(.4,0,.2,1),background .25s;box-shadow:0 1px 4px rgba(0,0,0,.5)}
.toggle-track.white-mode .toggle-thumb{left:29px;background:#fff;box-shadow:0 1px 6px rgba(0,0,0,.3)}
.line-icon{font-size:.75rem;font-weight:700;letter-spacing:.05em}
.line-icon.black{color:#444}.line-icon.white{color:#aaa}
.mode-badge{display:inline-block;padding:2px 8px;border-radius:4px;font-size:.7rem;font-weight:700;letter-spacing:.05em;transition:background .3s,color .3s}
.mode-badge.black-line{background:#1e1e1e;color:#888;border:1px solid #333}
.mode-badge.white-line{background:#e5e7eb;color:#1f2937;border:1px solid #9ca3af}
/* Battery */
.batt-bar-wrap{height:18px;background:#222;border-radius:9px;overflow:hidden;border:1px solid #2a2a2a;flex:1}
.batt-bar{height:100%;border-radius:9px;transition:width .5s,background .5s}
/* PWM gauges */
.gauge-wrap{display:flex;gap:12px;align-items:flex-end}
.gauge{display:flex;flex-direction:column;align-items:center;gap:4px;flex:1}
.gauge-track{width:100%;height:80px;background:#222;border-radius:6px;overflow:hidden;border:1px solid #2a2a2a;display:flex;flex-direction:column;justify-content:flex-end}
.gauge-fill{width:100%;border-radius:4px 4px 0 0;transition:height .07s}
.gauge-fill.L{background:#22c55e}.gauge-fill.R{background:#3b82f6}
.gauge-fill.rev{background:#ef4444}
/* PID graph */
#pidGraph{width:100%;height:90px;border-radius:6px;display:block}
/* Lap table */
#lapTable{width:100%;font-size:.72rem;border-collapse:collapse}
#lapTable th,#lapTable td{padding:3px 6px;border-bottom:1px solid #222;text-align:right}
#lapTable th{color:#555;font-weight:600}
#lapTable td:first-child{text-align:center}
/* Profile list */
.prof-item{display:flex;align-items:center;gap:6px;padding:4px 0;border-bottom:1px solid #1a1a1a}
.prof-name{flex:1;font-size:.8rem;color:#bbb}
/* OTA */
#otaStatus{font-size:.75rem;color:#555;margin-top:6px}
#otaBar{height:4px;background:#2a2a2a;border-radius:2px;margin-top:4px;overflow:hidden}
#otaFill{height:100%;background:#3b82f6;width:0%;transition:width .2s}
</style>
</head>
<body>
<h1>Line Follower v5.1 <span id="wsStatus" style="font-size:.7rem;color:#444">connecting...</span></h1>

<!-- CONTROL -->
<div class="card">
  <h2>Control</h2>
  <div class="status"><div class="dot" id="runDot"></div><span id="runLabel">Stopped</span></div>
  <div class="row">
    <button class="bg" id="btnStart" onclick="cmd('start')">▶ Start</button>
    <button class="br" id="btnStop"  onclick="cmd('stop')" disabled>⏹ Stop</button>
    <button class="by" id="btnCal"   onclick="cmd('calibrate')">⚡ Calibrate</button>
    <button class="bb" id="btnSave"  onclick="cmd('save')" disabled>💾 Save</button>
  </div>
  <div class="hint">Sweep bot over line for 8 seconds during calibration.</div>
  <div id="calBar"><div id="calFill"></div></div>
</div>

<!-- BATTERY -->
<div class="card" id="battCard">
  <h2>Battery Voltage
    <span id="battBadge" style="display:inline-block;margin-left:8px;padding:1px 8px;border-radius:4px;font-size:.65rem;font-weight:700;letter-spacing:.06em;background:#1a1a1a;color:#444;border:1px solid #2a2a2a">--</span>
  </h2>
  <div style="display:flex;align-items:center;gap:10px;margin-bottom:8px">
    <span id="battVoltTxt" style="font-size:1.4rem;font-weight:700;color:#7dd3fc;min-width:60px">--V</span>
    <div class="batt-bar-wrap"><div class="batt-bar" id="battBar" style="width:0%;background:#22c55e"></div></div>
  </div>
  <label>Alert Threshold (V)<small>Flash & alert when battery drops below this</small>
    <input type="range" id="battThresh" min="2.5" max="4.2" step="0.05" value="3.5" oninput="sendBatt()">
    <span class="val" id="battThreshv">3.50 V</span>
  </label>
</div>

<!-- LAP TIMER -->
<div class="card">
  <h2>Lap Timer &amp; Counter
    <span id="lapBadge" style="display:inline-block;margin-left:8px;padding:1px 8px;border-radius:4px;font-size:.65rem;font-weight:700;background:#1a1a1a;color:#444;border:1px solid #2a2a2a">STOP MODE</span>
  </h2>
  <div class="toggle-row" style="margin-bottom:8px">
    <span style="font-size:.82rem;color:#999">Lap Timer Mode</span>
    <div class="toggle-wrap" style="margin-left:8px">
      <div class="toggle-track" id="lapToggle" onclick="toggleLapMode()" style="background:#222;border-color:#3a3a3a">
        <div class="toggle-thumb" style="left:3px"></div>
      </div>
    </div>
    <span style="font-size:.75rem;color:#555;margin-left:4px">Count laps instead of stopping</span>
  </div>
  <div style="font-size:1.6rem;font-weight:700;color:#4ade80;margin-bottom:6px">
    Lap <span id="lapCount">0</span>
    <span style="font-size:.9rem;color:#555;margin-left:8px" id="lapCurrentTime"></span>
  </div>
  <table id="lapTable">
    <thead><tr><th>#</th><th>Time (s)</th><th>Diff</th></tr></thead>
    <tbody id="lapTbody"></tbody>
  </table>
</div>

<!-- PID GRAPH + PWM GAUGES -->
<div class="card">
  <h2>Live PID Error Graph</h2>
  <canvas id="pidGraph" width="600" height="90" style="background:#0d0d0d;border-radius:6px"></canvas>
  <div style="font-size:.68rem;color:#444;margin-top:4px">Scrolling error (±2500) — blue=error, orange=zero line</div>
</div>
<div class="card">
  <h2>Motor PWM Gauges</h2>
  <div class="gauge-wrap">
    <div class="gauge">
      <div class="gauge-track">
        <div class="gauge-fill L" id="gL" style="height:0%"></div>
      </div>
      <div style="font-size:.7rem;color:#22c55e;font-weight:700">L</div>
      <div class="val" id="gLv">0</div>
    </div>
    <div class="gauge">
      <div class="gauge-track">
        <div class="gauge-fill R" id="gR" style="height:0%"></div>
      </div>
      <div style="font-size:.7rem;color:#3b82f6;font-weight:700">R</div>
      <div class="val" id="gRv">0</div>
    </div>
  </div>
</div>

<!-- LINE COLOR MODE -->
<div class="card">
  <h2>Line Color Mode</h2>
  <div class="toggle-row">
    <span class="line-icon black">■ BLACK</span>
    <div class="toggle-wrap">
      <div class="toggle-track black-mode" id="lineToggle" onclick="toggleLineMode()">
        <div class="toggle-thumb"></div>
      </div>
    </div>
    <span class="line-icon white">□ WHITE</span>
    <span class="mode-badge black-line" id="modeBadge">BLACK LINE</span>
  </div>
</div>

<!-- LINE LOST RECOVERY -->
<div class="card">
  <h2>Line-Lost Recovery</h2>
  <label>Spin Timeout<small>Spin duration before giving up (ms). 0 = stop immediately.</small>
    <input type="range" id="lltSlider" min="0" max="5000" step="100" value="1500" oninput="sendLineLost()">
    <span class="val" id="lltv">1500ms</span>
  </label>
  <div style="font-size:.68rem;color:#444;margin-top:6px">When all sensors lose the line the bot spins in the last-known turn direction. If the line isn't found within the timeout, the bot stops.</div>
</div>

<!-- END ZONE -->
<div class="card">
  <h2>Finish Line / End Zone
    <span id="endZoneBadge" style="display:inline-block;margin-left:8px;padding:1px 8px;border-radius:4px;font-size:.65rem;font-weight:700;letter-spacing:.06em;background:#1a1a1a;color:#444;border:1px solid #2a2a2a;transition:all .2s">ARMED</span>
  </h2>
  <div id="endZoneBanner" style="display:none;background:#14532d;border:1px solid #166534;border-radius:7px;padding:10px 14px;margin-bottom:10px;text-align:center">
    <div style="font-size:1rem;font-weight:700;color:#4ade80;letter-spacing:.08em">🏁 FINISH LINE REACHED</div>
    <div style="font-size:.72rem;color:#86efac;margin-top:3px">Bot stopped automatically. Press ▶ Start to reset.</div>
  </div>
  <div class="toggle-row" style="margin-bottom:10px">
    <span style="font-size:.82rem;color:#999">Enable auto-stop</span>
    <div class="toggle-wrap" style="margin-left:8px">
      <div class="toggle-track" id="ezToggle" onclick="toggleEndZone()" style="background:#22c55e;border-color:#166534">
        <div class="toggle-thumb" style="left:29px;background:#fff"></div>
      </div>
    </div>
  </div>
  <div class="row">
    <label>Min Sensors Active<small>How many sensors must see black (4–6)</small>
      <input type="range" id="ezMinSensors" min="1" max="6" step="1" value="5" oninput="sendEndZone()">
      <span class="val" id="ezMinSensorsv">5 / 6</span></label>
    <label>Debounce Time<small>Must hold for this long (ms)</small>
      <input type="range" id="ezConfirmMs" min="10" max="500" step="10" value="100" oninput="sendEndZone()">
      <span class="val" id="ezConfirmMsv">100ms</span></label>
    <label>Sensor Threshold<small>Norm value to count as "over black"</small>
      <input type="range" id="ezThreshold" min="100" max="900" step="50" value="500" oninput="sendEndZone()">
      <span class="val" id="ezThresholdv">500</span></label>
  </div>
  <div style="font-size:.7rem;color:#555;margin-top:4px;margin-bottom:4px">Live sensor coverage</div>
  <div style="display:flex;gap:4px" id="ezSensorDots"></div>
</div>

<!-- SENSORS -->
<div class="card">
  <h2>IR Sensors</h2>
  <div style="font-size:.72rem;color:#7dd3fc;margin-bottom:4px">Normalised (0–1000)</div>
  <div class="sensors" id="normBars"></div>
  <div style="font-size:.72rem;color:#f59e0b;margin:8px 0 4px">Raw ADC (0–4095)</div>
  <div class="sensors" id="rawBars"></div>
  <div style="font-size:.72rem;color:#555;margin:8px 0 4px">Line position</div>
  <div class="pos-track"><div class="pos-dot" id="posDot" style="left:50%"></div></div>
  <div style="font-size:.68rem;color:#444;margin-top:3px" id="posText">pos: 0  err: 0</div>
</div>

<!-- PID -->
<div class="card">
  <h2>PID Tuning</h2>
  <div class="row">
    <label>Kp<small>Proportional</small>
      <input type="range" id="kp" min="0" max="5" step="0.05" value="0.4" oninput="sendPID()">
      <span class="val" id="kpv">0.40</span></label>
    <label>Ki<small>Integral</small>
      <input type="range" id="ki" min="0" max="1" step="0.005" value="0" oninput="sendPID()">
      <span class="val" id="kiv">0.000</span></label>
    <label>Kd<small>Derivative</small>
      <input type="range" id="kd" min="0" max="5" step="0.05" value="1.2" oninput="sendPID()">
      <span class="val" id="kdv">1.20</span></label>
  </div>
</div>

<!-- SPEED -->
<div class="card">
  <h2>Speed</h2>
  <div class="row">
    <label>Base<small>Normal running speed</small>
      <input type="range" id="spd" min="30" max="255" step="5" value="150" oninput="sendSpeed()">
      <span class="val" id="spdv">150</span></label>
    <label>Max<small>Maximum</small>
      <input type="range" id="maxspd" min="80" max="255" step="5" value="200" oninput="sendSpeed()">
      <span class="val" id="maxspdv">200</span></label>
    <label>Min<small>Stall guard</small>
      <input type="range" id="minspd" min="20" max="120" step="5" value="40" oninput="sendSpeed()">
      <span class="val" id="minspdv">40</span></label>
  </div>
</div>

<!-- MPU6050 ASSIST -->
<div class="card" id="mpuCard">
  <h2>MPU6050 Assist
    <span id="mpuBadge" style="display:inline-block;margin-left:8px;padding:1px 8px;border-radius:4px;font-size:.65rem;font-weight:700;letter-spacing:.06em;background:#1a1a1a;color:#444;border:1px solid #2a2a2a">CHECKING...</span>
  </h2>
  <div class="row">
    <button id="btnAutoTune" class="bg" onclick="cmd('mpu',{autoTuneActive:true})">🚀 Start Auto Tune</button>
    <button id="btnStopTune" class="br" onclick="cmd('mpu',{autoTuneActive:false})" style="display:none">⏹ Stop Auto Tune</button>
    <button id="btnRescanMpu" class="by" onclick="cmd('rescan_mpu',{})" style="margin-left:8px">🔄 Rescan</button>
  </div>
  
  <div class="toggle-row" style="margin-top:10px;margin-bottom:6px">
    <span style="font-size:.82rem;color:#999">Drift Assist (Straight Line)</span>
    <div class="toggle-wrap" style="margin-left:8px">
      <div class="toggle-track" id="driftToggle" onclick="toggleDrift()" style="background:#222;border-color:#3a3a3a">
        <div class="toggle-thumb" style="left:3px"></div>
      </div>
    </div>
  </div>
  <label>Drift Gain<small>Gyro resistance multiplier</small>
    <input type="range" id="driftGain" min="0" max="30" step="1" value="10" oninput="sendMPU()">
    <span class="val" id="driftGainv">10.0</span>
  </label>
  
  <div class="toggle-row" style="margin-top:10px">
    <span style="font-size:.82rem;color:#999">90° Gyro Pivot on Line-Lost</span>
    <div class="toggle-wrap" style="margin-left:8px">
      <div class="toggle-track" id="gyroPivotToggle" onclick="toggleGyroPivot()" style="background:#222;border-color:#3a3a3a">
        <div class="toggle-thumb" style="left:3px"></div>
      </div>
    </div>
  </div>
  
  <div style="font-size:.7rem;color:#666;margin-top:10px;display:flex;gap:15px">
    <span>Yaw: <b id="mpuYaw" style="color:#ddd">0.0°</b></span>
    <span>Rate: <b id="mpuRate" style="color:#ddd">0.0°/s</b></span>
    <span>Accel Y: <b id="mpuAccel" style="color:#ddd">0.00g</b></span>
  </div>
</div>

<!-- TRACK MAPPING (SPEED RUN) -->
<div class="card" id="mapCard" style="display:none">
  <h2>Track Mapping (SLAM-Lite)
    <span id="mapBadge" style="display:inline-block;margin-left:8px;padding:1px 8px;border-radius:4px;font-size:.65rem;font-weight:700;letter-spacing:.06em;background:#1a1a1a;color:#444;border:1px solid #2a2a2a">UNMAPPED</span>
  </h2>
  
  <div class="toggle-row" style="margin-top:10px;margin-bottom:6px">
    <span style="font-size:.82rem;color:#999">Record Map (Lap 1)</span>
    <div class="toggle-wrap" style="margin-left:8px">
      <div class="toggle-track" id="mappingToggle" onclick="toggleMapping()" style="background:#222;border-color:#3a3a3a">
        <div class="toggle-thumb" style="left:3px"></div>
      </div>
    </div>
  </div>
  
  <div class="toggle-row" style="margin-bottom:10px">
    <span style="font-size:.82rem;color:#999">Speed Run (Lap 2+)</span>
    <div class="toggle-wrap" style="margin-left:8px">
      <div class="toggle-track" id="speedRunToggle" onclick="toggleSpeedRun()" style="background:#222;border-color:#3a3a3a">
        <div class="toggle-thumb" style="left:3px"></div>
      </div>
    </div>
  </div>

  <div class="row">
    <label>Speed Run Max<small>Straight-line boost speed</small>
      <input type="range" id="srMax" min="150" max="255" step="5" value="255" oninput="sendMap()">
      <span class="val" id="srMaxv">255</span>
    </label>
    <label>Pre-Brake Time<small>Early brake ms before corner</small>
      <input type="range" id="pbMs" min="50" max="500" step="10" value="150" oninput="sendMap()">
      <span class="val" id="pbMsv">150ms</span>
    </label>
    <label>Pre-Brake Speed<small>Speed to drop to for corner entry</small>
      <input type="range" id="pbSpd" min="30" max="150" step="5" value="80" oninput="sendMap()">
      <span class="val" id="pbSpdv">80</span>
    </label>
  </div>
  <div class="row" style="margin-top:8px">
    <button class="bg" type="button" onclick="cmd('trackmap_get',{})" style="font-size:.7rem;padding:4px 10px;margin-right:8px">🗺 Fetch Map</button>
    <button class="bd" type="button" onclick="cmd('trackmap',{clearMap:true})" style="font-size:.7rem;padding:4px 10px">🗑 Clear Map</button>
  </div>
  <div style="position:relative;height:140px;background:#111;border:1px solid #2a2a2a;border-radius:8px;overflow:hidden;margin-top:10px;display:none" id="mapCanvasContainer">
    <canvas id="mapCanvas" width="400" height="140" style="width:100%;height:100%"></canvas>
  </div>
</div>

<!-- TURN RADIUS -->
<div class="card">
  <h2>Turning Radius &amp; Hairpin Assist</h2>
  <div class="row">
    <label>Sharpness<small>PID correction multiplier</small>
      <input type="range" id="sharpness" min="0.1" max="3.0" step="0.05" value="1.0" oninput="sendTurn()">
      <span class="val" id="sharpnessv">1.00×</span></label>
    <label>Inner Brake<small>Extra braking during turns</small>
      <input type="range" id="brake" min="0" max="100" step="5" value="0" oninput="sendTurn()">
      <span class="val" id="brakev">0%</span></label>
  </div>
  <div class="row">
    <label>Curve Exponent<small>Non-linear response</small>
      <input type="range" id="curveExp" min="1.0" max="3.0" step="0.1" value="1.0" oninput="sendTurn()">
      <span class="val" id="curveExpv">1.0</span></label>
    <label>Speed Reduce on Turn<small>Auto-slow % at max error</small>
      <input type="range" id="turnSpeedReduce" min="0" max="100" step="5" value="0" oninput="sendTurn()">
      <span class="val" id="turnSpeedReducev">0%</span></label>
    <label>Pivot Threshold<small>Error at which inner wheel reverses</small>
      <input type="range" id="pivotThreshold" min="0" max="2500" step="100" value="0" oninput="sendTurn()">
      <span class="val" id="pivotThresholdv">off</span></label>
  </div>
  <div style="position:relative;height:56px;background:#111;border:1px solid #2a2a2a;border-radius:8px;overflow:hidden;margin-top:4px">
    <canvas id="turnCanvas" width="400" height="56" style="width:100%;height:100%"></canvas>
  </div>
</div>

<!-- CORNER -->
<div class="card">
  <h2>90° Corner Detection
    <span id="cornerBadge" style="display:inline-block;margin-left:8px;padding:1px 7px;border-radius:4px;font-size:.65rem;font-weight:700;background:#1a1a1a;color:#444;border:1px solid #2a2a2a;transition:all .15s">IDLE</span>
  </h2>
  <div class="row">
    <label>Sensitivity<small>Min error-rate to trigger (0=off)</small>
      <input type="range" id="cornerSensitivity" min="0" max="1500" step="50" value="0" oninput="sendTurn()">
      <span class="val" id="cornerSensitivityv">off</span></label>
    <label>Boost Gain<small>Kp multiplier during corner</small>
      <input type="range" id="cornerBoost" min="1.0" max="5.0" step="0.1" value="2.0" oninput="sendTurn()">
      <span class="val" id="cornerBoostv">2.0×</span></label>
  </div>
  <div class="row">
    <label>Boost Duration<small>How long corner mode stays on (ms)</small>
      <input type="range" id="cornerDuration" min="50" max="600" step="25" value="200" oninput="sendTurn()">
      <span class="val" id="cornerDurationv">200ms</span></label>
    <label>Corner Speed<small>Speed % of base during boost</small>
      <input type="range" id="cornerSpeedPct" min="10" max="100" step="5" value="50" oninput="sendTurn()">
      <span class="val" id="cornerSpeedPctv">50%</span></label>
  </div>
</div>

<!-- SETTINGS PROFILES -->
<div class="card">
  <h2>Settings Profiles</h2>
  <div class="row" style="margin-bottom:10px">
    <input type="text" id="profName" placeholder="Profile name..." style="flex:1">
    <select id="profSlot" style="background:#111;border:1px solid #333;color:#ddd;padding:5px 8px;border-radius:5px;font-size:.82rem">
      <option value="0">Slot 0</option><option value="1">Slot 1</option>
      <option value="2">Slot 2</option><option value="3">Slot 3</option>
      <option value="4">Slot 4</option><option value="5">Slot 5</option>
      <option value="6">Slot 6</option><option value="7">Slot 7</option>
    </select>
    <button class="bg" onclick="saveProfile()">💾 Save</button>
    <button class="bb" onclick="loadProfile()">📂 Load</button>
    <button class="br" onclick="deleteProfile()">🗑 Del</button>
  </div>
  <div id="profList" style="font-size:.75rem;color:#555">No profiles saved.</div>
</div>

<!-- OTA UPDATE -->
<div class="card">
  <h2>OTA Firmware Update</h2>
  <div style="font-size:.75rem;color:#666;margin-bottom:8px">Flash a new <code style="color:#7dd3fc">.bin</code> compiled from Arduino IDE. <b style="color:#ef4444">Device will reboot after upload.</b></div>
  <form id="otaForm" enctype="multipart/form-data">
    <div class="row">
      <input type="file" id="otaFile" accept=".bin" style="font-size:.8rem;color:#aaa;flex:1">
      <button class="bp" type="button" onclick="uploadOTA()">⬆ Flash</button>
    </div>
  </form>
  <div id="otaStatus">Select a .bin file to upload.</div>
  <div id="otaBar"><div id="otaFill"></div></div>
</div>

<!-- SERIAL MONITOR -->
<div class="card">
  <h2>Serial Monitor
    <button onclick="clearLog()" style="padding:2px 8px;font-size:.68rem;background:#222;color:#888;border:1px solid #2a2a2a;border-radius:4px;cursor:pointer;margin-left:8px;font-weight:400">Clear</button>
  </h2>
  <div id="logBox"></div>
</div>

<script>
let ws, calTimer=null, calProgress=0;
let invertLine=false, endZoneEnabled=true, lapModEnabled=false, driftAssistEnabled=false, gyroPivotEnabled=false, mappingEnabled=false, speedRunEnabled=false;
const pidHistory=new Array(200).fill(0);

// Init sensor bars
(function(){
  const c=document.getElementById('ezSensorDots');
  if(c) for(let i=0;i<6;i++) c.innerHTML+=`<div id="ezDot${i}" style="flex:1;height:14px;border-radius:3px;background:#1a1a1a;border:1px solid #2a2a2a;transition:background .1s"></div>`;
})();
function initBars(id,cc,nc){
  const c=document.getElementById(id); c.innerHTML='';
  for(let i=0;i<6;i++) c.innerHTML+=`<div class="sb"><div class="bwrap"><div class="bar ${cc}" id="${id}${i}" style="height:0%"></div></div><div class="sn ${nc}" id="${id}n${i}">0</div><div class="sl">IR${i+2}</div></div>`;
}
initBars('normBars','norm','');
initBars('rawBars','raw','raw');

// PID graph setup
const gCvs=document.getElementById('pidGraph');
const gCtx=gCvs.getContext('2d');
function drawPIDGraph(){
  const W=gCvs.width, H=gCvs.height;
  gCtx.clearRect(0,0,W,H);
  // Zero line
  gCtx.beginPath(); gCtx.moveTo(0,H/2); gCtx.lineTo(W,H/2);
  gCtx.strokeStyle='#f59e0b33'; gCtx.lineWidth=1; gCtx.stroke();
  // Error line
  gCtx.beginPath();
  pidHistory.forEach((v,i)=>{
    const x=i*(W/pidHistory.length);
    const y=H/2 - (v/2500)*(H/2);
    i===0?gCtx.moveTo(x,y):gCtx.lineTo(x,y);
  });
  gCtx.strokeStyle='#7dd3fc'; gCtx.lineWidth=1.5; gCtx.stroke();
}

function connect(){
  ws=new WebSocket('ws://'+location.hostname+':81/');
  ws.onopen=()=>setWs(true);
  ws.onclose=()=>{ setWs(false); setTimeout(connect,2000); };
  ws.onmessage=(e)=>{
    const d=JSON.parse(e.data);
    if(d.type==='state')    updateState(d);
    if(d.type==='sensors')  updateSensors(d);
    if(d.type==='log')      appendLog(d.lines);
    if(d.type==='profiles') updateProfileList(d.list);
    if(d.type==='map_data') drawMap(d.segs);
  };
}
connect();

function setWs(ok){
  const el=document.getElementById('wsStatus');
  el.textContent=ok?'● connected':'○ disconnected';
  el.style.color=ok?'#22c55e':'#ef4444';
}

function updateState(d){
  const dot=document.getElementById('runDot');
  dot.className='dot '+(d.calibrating?'cal':d.running?'on':'off');
  document.getElementById('runLabel').textContent=d.calibrating?'Calibrating...':d.running?'Running':'Stopped';
  document.getElementById('btnStart').disabled=d.running||d.calibrating;
  document.getElementById('btnStop').disabled=!d.running&&!d.calibrating;
  document.getElementById('btnCal').disabled=d.running||d.calibrating;
  document.getElementById('btnSave').disabled=!d.calibrated&&!d.calibrating;

  // Battery
  if(typeof d.battVoltage!=='undefined'){
    const v=d.battVoltage, thr=d.battAlertThreshold;
    document.getElementById('battVoltTxt').textContent=v>0.5?v.toFixed(2)+'V':'--V';
    const pct=Math.min(100,Math.max(0,((v-3.0)/(4.2-3.0))*100));
    const bar=document.getElementById('battBar');
    bar.style.width=pct+'%';
    bar.style.background=d.battLow?'#ef4444':'#22c55e';
    const badge=document.getElementById('battBadge');
    if(d.battLow){ badge.textContent='LOW!'; badge.style.color='#ef4444'; badge.style.background='#2d1010'; }
    else{ badge.textContent=v>0.5?v.toFixed(2)+'V':'--'; badge.style.color='#7dd3fc'; badge.style.background='#1a1a1a'; }
    document.getElementById('battThresh').value=thr;
    document.getElementById('battThreshv').textContent=thr.toFixed(2)+' V';
  }

  // Lap timer
  if(typeof d.lapCount!=='undefined'){
    document.getElementById('lapCount').textContent=d.lapCount;
    lapModEnabled=d.lapTimerMode;
    const tog=document.getElementById('lapToggle');
    const thumb=tog.querySelector('.toggle-thumb');
    if(lapModEnabled){ tog.style.background='#22c55e'; tog.style.borderColor='#166534'; thumb.style.left='29px'; }
    else{ tog.style.background='#222'; tog.style.borderColor='#3a3a3a'; thumb.style.left='3px'; }
    const badge=document.getElementById('lapBadge');
    badge.textContent=lapModEnabled?'LAP MODE':'STOP MODE';
    badge.style.color=lapModEnabled?'#4ade80':'#555';
    badge.style.background=lapModEnabled?'#14532d':'#1a1a1a';
    badge.style.borderColor=lapModEnabled?'#166534':'#2a2a2a';
    // Lap table
    const tbody=document.getElementById('lapTbody'); tbody.innerHTML='';
    if(d.lapTimes&&d.lapTimes.length){
      let prev=0;
      d.lapTimes.forEach((t,i)=>{
        const diff=i>0?t-d.lapTimes[i-1]:0;
        const tr=document.createElement('tr');
        tr.innerHTML=`<td>${i+1}</td><td>${(t/1000).toFixed(3)}</td><td style="color:${i>0&&diff<0?'#22c55e':'#f59e0b'}">${i>0?(diff>0?'+':''+(diff/1000).toFixed(3)):'-'}</td>`;
        tbody.appendChild(tr);
        prev=t;
      });
    }
  }

  // Line lost timeout
  if(typeof d.lineLostTimeout!=='undefined'){
    document.getElementById('lltSlider').value=d.lineLostTimeout;
    document.getElementById('lltv').textContent=d.lineLostTimeout===0?'off (stop)':d.lineLostTimeout+'ms';
  }

  // End zone
  if(typeof d.endZoneEnable!=='undefined'){
    endZoneEnabled=d.endZoneEnable; updateEndZoneToggleUI();
    document.getElementById('ezMinSensors').value=d.endZoneMinSensors;
    document.getElementById('ezMinSensorsv').textContent=d.endZoneMinSensors+' / 6';
    document.getElementById('ezConfirmMs').value=d.endZoneConfirmMs;
    document.getElementById('ezConfirmMsv').textContent=d.endZoneConfirmMs+'ms';
    document.getElementById('ezThreshold').value=d.endZoneThreshold;
    document.getElementById('ezThresholdv').textContent=d.endZoneThreshold;
    const badge=document.getElementById('endZoneBadge');
    const banner=document.getElementById('endZoneBanner');
    if(d.endZoneTriggered){ badge.textContent='FINISHED'; badge.style.background='#14532d'; badge.style.color='#4ade80'; badge.style.borderColor='#166534'; banner.style.display='block'; }
    else if(d.endZoneEnable){ badge.textContent='ARMED'; badge.style.background='#1a1a1a'; badge.style.color='#22c55e'; badge.style.borderColor='#166534'; banner.style.display='none'; }
    else{ badge.textContent='OFF'; badge.style.background='#1a1a1a'; badge.style.color='#444'; badge.style.borderColor='#2a2a2a'; banner.style.display='none'; }
  }

  // Line color
  if(typeof d.invertLine!=='undefined'&&d.invertLine!==invertLine){ invertLine=d.invertLine; applyLineModeUI(); }

  // Cal bar
  if(d.calibrating){
    if(!calTimer){ calProgress=0; calTimer=setInterval(()=>{ calProgress=Math.min(calProgress+100/80,100); document.getElementById('calFill').style.width=calProgress+'%'; if(calProgress>=100){clearInterval(calTimer);calTimer=null;} },100); }
  } else { if(calTimer){clearInterval(calTimer);calTimer=null;} document.getElementById('calFill').style.width=d.calibrated?'100%':'0%'; }

  // PID sliders
  document.getElementById('kp').value=d.kp; document.getElementById('kpv').textContent=d.kp.toFixed(2);
  document.getElementById('ki').value=d.ki; document.getElementById('kiv').textContent=d.ki.toFixed(3);
  document.getElementById('kd').value=d.kd; document.getElementById('kdv').textContent=d.kd.toFixed(2);
  document.getElementById('spd').value=d.baseSpeed; document.getElementById('spdv').textContent=d.baseSpeed;
  document.getElementById('maxspd').value=d.maxSpeed; document.getElementById('maxspdv').textContent=d.maxSpeed;
  document.getElementById('minspd').value=d.minSpeed; document.getElementById('minspdv').textContent=d.minSpeed;
  if(typeof d.turnSharpness!=='undefined'){
    document.getElementById('sharpness').value=d.turnSharpness; document.getElementById('sharpnessv').textContent=d.turnSharpness.toFixed(2)+'×';
    document.getElementById('brake').value=d.turnBrake; document.getElementById('brakev').textContent=d.turnBrake+'%';
    document.getElementById('curveExp').value=d.curveExp; document.getElementById('curveExpv').textContent=d.curveExp.toFixed(1);
    document.getElementById('turnSpeedReduce').value=d.turnSpeedReduce; document.getElementById('turnSpeedReducev').textContent=d.turnSpeedReduce+'%';
    document.getElementById('pivotThreshold').value=d.pivotThreshold; document.getElementById('pivotThresholdv').textContent=d.pivotThreshold>0?d.pivotThreshold:'off';
    document.getElementById('cornerSensitivity').value=d.cornerSensitivity; document.getElementById('cornerSensitivityv').textContent=d.cornerSensitivity>0?d.cornerSensitivity:'off';
    document.getElementById('cornerBoost').value=d.cornerBoost; document.getElementById('cornerBoostv').textContent=d.cornerBoost.toFixed(1)+'×';
    document.getElementById('cornerDuration').value=d.cornerDuration; document.getElementById('cornerDurationv').textContent=d.cornerDuration+'ms';
    document.getElementById('cornerSpeedPct').value=d.cornerSpeedPct; document.getElementById('cornerSpeedPctv').textContent=d.cornerSpeedPct+'%';
    const cbadge=document.getElementById('cornerBadge');
    if(d.cornerActive){ cbadge.textContent='CORNER!'; cbadge.style.background='#78350f'; cbadge.style.color='#f59e0b'; cbadge.style.borderColor='#92400e'; }
    else{ cbadge.textContent='IDLE'; cbadge.style.background='#1a1a1a'; cbadge.style.color='#444'; cbadge.style.borderColor='#2a2a2a'; }
    drawTurnPreview(d.turnSharpness,d.turnBrake,d.curveExp,d.turnSpeedReduce,d.pivotThreshold);
  }

  if(typeof d.mpuPresent !== 'undefined'){
    const badge = document.getElementById('mpuBadge');
    const isMpu = d.mpuPresent;
    if(isMpu){
      badge.textContent='CONNECTED'; badge.style.background='#14532d'; badge.style.color='#4ade80'; badge.style.borderColor='#166534';
      document.getElementById('btnAutoTune').disabled = false;
      document.getElementById('driftGain').disabled = false;
      document.getElementById('driftToggle').style.opacity = '1';
      document.getElementById('driftToggle').style.pointerEvents = 'auto';
      document.getElementById('gyroPivotToggle').style.opacity = '1';
      document.getElementById('gyroPivotToggle').style.pointerEvents = 'auto';
      
      driftAssistEnabled = d.driftAssistEnable;
      const t1 = document.getElementById('driftToggle'), th1 = t1.querySelector('.toggle-thumb');
      if(driftAssistEnabled){ t1.style.background='#22c55e'; t1.style.borderColor='#166534'; th1.style.left='29px'; }
      else{ t1.style.background='#222'; t1.style.borderColor='#3a3a3a'; th1.style.left='3px'; }

      gyroPivotEnabled = d.gyroTurnEnable;
      const t2 = document.getElementById('gyroPivotToggle'), th2 = t2.querySelector('.toggle-thumb');
      if(gyroPivotEnabled){ t2.style.background='#22c55e'; t2.style.borderColor='#166534'; th2.style.left='29px'; }
      else{ t2.style.background='#222'; t2.style.borderColor='#3a3a3a'; th2.style.left='3px'; }

      document.getElementById('driftGain').value = d.driftAssistGain;
      document.getElementById('driftGainv').textContent = d.driftAssistGain.toFixed(1);

      document.getElementById('btnAutoTune').style.display = d.autoTuneActive?'none':'block';
      document.getElementById('btnStopTune').style.display = d.autoTuneActive?'block':'none';
    } else {
      badge.textContent='NOT FOUND'; badge.style.background='#2d1010'; badge.style.color='#ef4444'; badge.style.borderColor='#450a0a';
      document.getElementById('btnAutoTune').disabled = true;
      document.getElementById('driftGain').disabled = true;
      document.getElementById('driftToggle').style.opacity = '0.3';
      document.getElementById('driftToggle').style.pointerEvents = 'none';
      document.getElementById('gyroPivotToggle').style.opacity = '0.3';
      document.getElementById('gyroPivotToggle').style.pointerEvents = 'none';
    }
  }

  if(typeof d.mappingEnabled !== 'undefined'){
    document.getElementById('mapCard').style.display='block';
    
    mappingEnabled = d.mappingEnabled;
    const t1 = document.getElementById('mappingToggle'), th1 = t1.querySelector('.toggle-thumb');
    if(mappingEnabled){ t1.style.background='#22c55e'; t1.style.borderColor='#166534'; th1.style.left='29px'; }
    else{ t1.style.background='#222'; t1.style.borderColor='#3a3a3a'; th1.style.left='3px'; }

    speedRunEnabled = d.speedRunEnabled;
    const t2 = document.getElementById('speedRunToggle'), th2 = t2.querySelector('.toggle-thumb');
    if(speedRunEnabled){ t2.style.background='#22c55e'; t2.style.borderColor='#166534'; th2.style.left='29px'; }
    else{ t2.style.background='#222'; t2.style.borderColor='#3a3a3a'; th2.style.left='3px'; }

    document.getElementById('srMax').value = d.speedRunMax; document.getElementById('srMaxv').textContent = d.speedRunMax;
    document.getElementById('pbMs').value = d.preBrakeMs; document.getElementById('pbMsv').textContent = d.preBrakeMs+'ms';
    document.getElementById('pbSpd').value = d.preBrakeSpeed; document.getElementById('pbSpdv').textContent = d.preBrakeSpeed;
    
    const badge = document.getElementById('mapBadge');
    if(d.trackMapped) { badge.textContent=d.segmentCount+' Segs Mapped'; badge.style.background='#14532d'; badge.style.color='#4ade80'; badge.style.borderColor='#166534'; }
    else if(d.mappingEnabled) { badge.textContent='RECORDING...'; badge.style.background='#78350f'; badge.style.color='#f59e0b'; badge.style.borderColor='#92400e'; }
    else { badge.textContent='UNMAPPED'; badge.style.background='#1a1a1a'; badge.style.color='#444'; badge.style.borderColor='#2a2a2a'; }
  }
}

function updateSensors(d){
  const ezThresh=+document.getElementById('ezThreshold').value;
  for(let i=0;i<6;i++){
    document.getElementById('normBars'+i).style.height=(d.norm[i]/10)+'%';
    document.getElementById('normBarsn'+i).textContent=d.norm[i];
    if(d.raw){ document.getElementById('rawBars'+i).style.height=(d.raw[i]/40.95)+'%'; document.getElementById('rawBarsn'+i).textContent=d.raw[i]; }
    const dot=document.getElementById('ezDot'+i);
    if(dot){ const ov=d.norm[i]>=ezThresh; dot.style.background=ov?'#22c55e':'#1a1a1a'; dot.style.borderColor=ov?'#166534':'#2a2a2a'; }
  }
  const pct=((d.position+3500)/7000)*100;
  document.getElementById('posDot').style.left=pct+'%';
  document.getElementById('posText').textContent='pos: '+d.position+'  err: '+d.error.toFixed(1);
  // PID graph
  pidHistory.push(d.error); pidHistory.shift(); drawPIDGraph();
  // PWM gauges
  if(typeof d.pwmL!=='undefined'){
    const absL=Math.abs(d.pwmL), absR=Math.abs(d.pwmR);
    const gL=document.getElementById('gL'), gR=document.getElementById('gR');
    gL.style.height=(absL/255*100)+'%'; gL.style.background=d.pwmL>=0?'#22c55e':'#ef4444';
    gR.style.height=(absR/255*100)+'%'; gR.style.background=d.pwmR>=0?'#3b82f6':'#ef4444';
    document.getElementById('gLv').textContent=d.pwmL;
    document.getElementById('gRv').textContent=d.pwmR;
  }
  if(typeof d.gyroZ !== 'undefined'){
    document.getElementById('mpuYaw').textContent = d.yawAngle.toFixed(1) + '°';
    document.getElementById('mpuRate').textContent = d.gyroZ.toFixed(1) + '°/s';
    document.getElementById('mpuAccel').textContent = d.accelY.toFixed(2) + 'g';
  }
}

// ── Line mode ────────────────────────────────────────────
function toggleLineMode(){ invertLine=!invertLine; applyLineModeUI(); cmd('linemode',{invert:invertLine}); }
function applyLineModeUI(){
  const track=document.getElementById('lineToggle'); const badge=document.getElementById('modeBadge');
  if(invertLine){ track.className='toggle-track white-mode'; badge.className='mode-badge white-line'; badge.textContent='WHITE LINE'; }
  else{ track.className='toggle-track black-mode'; badge.className='mode-badge black-line'; badge.textContent='BLACK LINE'; }
}

// ── Lap mode ────────────────────────────────────────────
function toggleLapMode(){ lapModEnabled=!lapModEnabled; cmd('laptimer',{enable:lapModEnabled}); }

// ── End zone ─────────────────────────────────────────────
function toggleEndZone(){ endZoneEnabled=!endZoneEnabled; updateEndZoneToggleUI(); sendEndZone(); }
function updateEndZoneToggleUI(){
  const track=document.getElementById('ezToggle'); const thumb=track.querySelector('.toggle-thumb');
  if(endZoneEnabled){ track.style.background='#22c55e'; track.style.borderColor='#166534'; thumb.style.left='29px'; }
  else{ track.style.background='#222'; track.style.borderColor='#3a3a3a'; thumb.style.left='3px'; }
}
function sendEndZone(){
  const mn=+document.getElementById('ezMinSensors').value;
  const ms=+document.getElementById('ezConfirmMs').value;
  const th=+document.getElementById('ezThreshold').value;
  document.getElementById('ezMinSensorsv').textContent=mn+' / 6';
  document.getElementById('ezConfirmMsv').textContent=ms+'ms';
  document.getElementById('ezThresholdv').textContent=th;
  cmd('endzone',{enable:endZoneEnabled,minSensors:mn,confirmMs:ms,threshold:th});
}

// ── MPU6050 ──────────────────────────────────────────────
function toggleDrift(){ driftAssistEnabled=!driftAssistEnabled; cmd('mpu',{driftAssistEnable:driftAssistEnabled}); }
function toggleGyroPivot(){ gyroPivotEnabled=!gyroPivotEnabled; cmd('mpu',{gyroTurnEnable:gyroPivotEnabled}); }
function sendMPU(){
  const dg = +document.getElementById('driftGain').value;
  document.getElementById('driftGainv').textContent = dg.toFixed(1);
  cmd('mpu',{driftAssistGain:dg});
}

// ── Track Mapping ──────────────────────────────────────────
function toggleMapping(){ cmd('trackmap',{mappingEnabled:!mappingEnabled}); }
function toggleSpeedRun(){ cmd('trackmap',{speedRunEnabled:!speedRunEnabled}); }
function sendMap(){
  const srm=+document.getElementById('srMax').value;
  const pbm=+document.getElementById('pbMs').value;
  const pbs=+document.getElementById('pbSpd').value;
  document.getElementById('srMaxv').textContent=srm;
  document.getElementById('pbMsv').textContent=pbm+'ms';
  document.getElementById('pbSpdv').textContent=pbs;
  cmd('trackmap',{speedRunMax:srm,preBrakeMs:pbm,preBrakeSpeed:pbs});
}
function drawMap(segs){
  const cvs=document.getElementById('mapCanvas'); const container=document.getElementById('mapCanvasContainer');
  if(!cvs) return; container.style.display='block';
  const ctx=cvs.getContext('2d'); const W=cvs.width, H=cvs.height;
  ctx.clearRect(0,0,W,H);
  if(!segs||segs.length===0){ ctx.fillStyle='#555'; ctx.font='10px sans-serif'; ctx.fillText('No track data recorded.',10,20); return; }
  
  let maxD=100;
  for(let i=0;i<segs.length;i++) if(segs[i].d>maxD) maxD=segs[i].d;
  const scale = (Math.min(W,H)*0.4) / maxD;

  ctx.beginPath();
  let x=W/2, y=H-20, angle=-Math.PI/2;
  ctx.moveTo(x,y);
  for(let i=0;i<segs.length;i++){
    let type=segs[i].t, dur=segs[i].d;
    if(type===0){ // STRAIGHT
      x += Math.cos(angle)*(dur*scale); y += Math.sin(angle)*(dur*scale); ctx.lineTo(x,y);
    } else { // TURN
       let ta = (type===1 ? -1 : 1) * (Math.PI/2);
       let cx = x + Math.cos(angle + ta/2)*10; let cy = y + Math.sin(angle + ta/2)*10;
       angle += ta; x += Math.cos(angle)*15; y += Math.sin(angle)*15;
       ctx.quadraticCurveTo(cx, cy, x, y);
    }
  }
  ctx.strokeStyle='#4ade80'; ctx.lineWidth=2; ctx.stroke();
  ctx.beginPath(); ctx.arc(x,y,4,0,2*Math.PI); ctx.fillStyle='#ef4444'; ctx.fill();
  
  wsLog('[MAP] Rendered '+segs.length+' segments');
}

// ── Battery ──────────────────────────────────────────────
function sendBatt(){
  const t=+document.getElementById('battThresh').value;
  document.getElementById('battThreshv').textContent=t.toFixed(2)+' V';
  cmd('batt',{threshold:t});
}

// ── Line lost ────────────────────────────────────────────
function sendLineLost(){
  const t=+document.getElementById('lltSlider').value;
  document.getElementById('lltv').textContent=t===0?'off (stop)':t+'ms';
  cmd('linelost',{timeout:t});
}

// ── Log ──────────────────────────────────────────────────
function appendLog(lines){
  const box=document.getElementById('logBox');
  lines.forEach(l=>{
    const div=document.createElement('div');
    div.className='ll'+(l.includes('[ERR]')?' e':l.includes('[WARN]')?' w':(l.includes('[IMU]')||l.includes('[WIFI]')||l.includes('[WEB]')||l.includes('[MODE]'))?' i':'');
    div.textContent=l; box.appendChild(div);
  });
  box.scrollTop=box.scrollHeight;
}
function clearLog(){ document.getElementById('logBox').innerHTML=''; }

// ── Commands ──────────────────────────────────────────────
function cmd(c,ex={}){ if(ws&&ws.readyState===1) ws.send(JSON.stringify({cmd:c,...ex})); }
function sendPID(){
  const kp=+document.getElementById('kp').value, ki=+document.getElementById('ki').value, kd=+document.getElementById('kd').value;
  document.getElementById('kpv').textContent=kp.toFixed(2); document.getElementById('kiv').textContent=ki.toFixed(3); document.getElementById('kdv').textContent=kd.toFixed(2);
  cmd('pid',{kp,ki,kd});
}
function sendSpeed(){
  const b=+document.getElementById('spd').value, mx=+document.getElementById('maxspd').value, mn=+document.getElementById('minspd').value;
  document.getElementById('spdv').textContent=b; document.getElementById('maxspdv').textContent=mx; document.getElementById('minspdv').textContent=mn;
  cmd('speed',{baseSpeed:b,maxSpeed:mx,minSpeed:mn});
}
function sendTurn(){
  const sh=+document.getElementById('sharpness').value, br=+document.getElementById('brake').value;
  const ce=+document.getElementById('curveExp').value, sr=+document.getElementById('turnSpeedReduce').value;
  const pt=+document.getElementById('pivotThreshold').value, cs=+document.getElementById('cornerSensitivity').value;
  const cb=+document.getElementById('cornerBoost').value, cd=+document.getElementById('cornerDuration').value;
  const csp=+document.getElementById('cornerSpeedPct').value;
  document.getElementById('sharpnessv').textContent=sh.toFixed(2)+'×'; document.getElementById('brakev').textContent=br+'%';
  document.getElementById('curveExpv').textContent=ce.toFixed(1); document.getElementById('turnSpeedReducev').textContent=sr+'%';
  document.getElementById('pivotThresholdv').textContent=pt>0?pt:'off'; document.getElementById('cornerSensitivityv').textContent=cs>0?cs:'off';
  document.getElementById('cornerBoostv').textContent=cb.toFixed(1)+'×'; document.getElementById('cornerDurationv').textContent=cd+'ms';
  document.getElementById('cornerSpeedPctv').textContent=csp+'%';
  drawTurnPreview(sh,br,ce,sr,pt);
  cmd('turn',{sharpness:sh,brake:br,curveExp:ce,turnSpeedReduce:sr,pivotThreshold:pt,cornerSensitivity:cs,cornerBoost:cb,cornerDuration:cd,cornerSpeedPct:csp});
}

// ── Turn preview ─────────────────────────────────────────
function drawTurnPreview(sh,br,ce,sr,pt){
  const c=document.getElementById('turnCanvas'); if(!c) return;
  const ctx=c.getContext('2d'); const W=c.width, H=c.height;
  ctx.clearRect(0,0,W,H);
  ctx.beginPath(); let x=W*0.15, y=H*0.85, angle=-Math.PI/2;
  for(let s=0;s<=80;s++){
    const err=(s/80)*2500, ne=err/2500;
    const ss=1-(ne*(sr/100));
    const base=Math.max(0.2,ss);
    const cm=Math.pow(ne,ce)*sh;
    let L=base-cm, R=base+cm;
    if(br>0) L=L-(L-0.1)*(ne*(br/100));
    if(pt>0&&err>=pt){ const pd=(err-pt)/(2500-pt); L=-0.1-pd*0.4; }
    const da=(R-L)*0.04; angle+=da; x+=Math.cos(angle)*3.5; y+=Math.sin(angle)*3.5;
    s===0?ctx.moveTo(x,y):ctx.lineTo(x,y);
  }
  ctx.strokeStyle='#7dd3fc'; ctx.lineWidth=2; ctx.stroke();
  ctx.beginPath(); ctx.arc(W*0.15,H*0.85,4,0,2*Math.PI); ctx.fillStyle='#22c55e'; ctx.fill();
}

// ── Profiles ──────────────────────────────────────────────
function saveProfile(){
  const name=document.getElementById('profName').value.trim()||'Preset';
  const slot=+document.getElementById('profSlot').value;
  cmd('saveprofile',{slot,name});
}
function loadProfile(){
  const slot=+document.getElementById('profSlot').value;
  cmd('loadprofile',{slot});
}
function deleteProfile(){
  const slot=+document.getElementById('profSlot').value;
  cmd('deleteprofile',{slot});
}
function updateProfileList(list){
  const el=document.getElementById('profList');
  if(!list||!list.length){ el.innerHTML='<span style="color:#444">No profiles saved.</span>'; return; }
  el.innerHTML=list.map(p=>`<div class="prof-item"><span class="prof-name">Slot ${p.slot}: <b style="color:#ddd">${p.name}</b></span><button class="bb" style="padding:2px 8px;font-size:.7rem" onclick="loadSlot(${p.slot})">Load</button></div>`).join('');
}
function loadSlot(slot){ document.getElementById('profSlot').value=slot; loadProfile(); }

// ── OTA ──────────────────────────────────────────────────
function uploadOTA(){
  const file=document.getElementById('otaFile').files[0];
  if(!file){ alert('Pick a .bin file first'); return; }
  const formData=new FormData();
  formData.append('firmware',file);
  const xhr=new XMLHttpRequest();
  xhr.open('POST','http://'+location.hostname+'/update');
  xhr.upload.onprogress=(e)=>{
    if(e.lengthComputable){
      const pct=Math.round(e.loaded/e.total*100);
      document.getElementById('otaFill').style.width=pct+'%';
      document.getElementById('otaStatus').textContent='Uploading... '+pct+'%';
    }
  };
  xhr.onload=()=>{
    document.getElementById('otaStatus').textContent=xhr.responseText==='OK'?'✔ Upload OK. Device rebooting...':'✘ Upload failed: '+xhr.responseText;
    document.getElementById('otaFill').style.background=xhr.responseText==='OK'?'#22c55e':'#ef4444';
  };
  xhr.onerror=()=>{ document.getElementById('otaStatus').textContent='Connection error.'; };
  xhr.send(formData);
}
</script>
</body>
</html>
)rawhtml";
// HTML_PLACEHOLDER_END

// ══════════════════════════════════════════════════════════
//  Log buffer
// ══════════════════════════════════════════════════════════
#define LOG_BUF 40
String logLines[LOG_BUF];
int logHead = 0, logCount = 0;
void wsLog(const String &msg) {
  Serial.println(msg);
  logLines[logHead] = msg;
  logHead = (logHead + 1) % LOG_BUF;
  if (logCount < LOG_BUF)
    logCount++;
}

// ══════════════════════════════════════════════════════════
//  Motors
// ══════════════════════════════════════════════════════════
void motorSetup() {
  ledcSetup(PWM_L1_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_L_IN1, PWM_L1_CH);
  ledcSetup(PWM_L2_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_L_IN2, PWM_L2_CH);
  ledcSetup(PWM_R3_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_R_IN3, PWM_R3_CH);
  ledcSetup(PWM_R4_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_R_IN4, PWM_R4_CH);
}
void setMotors(int L, int R) {
  L = constrain(L, -255, 255);
  R = constrain(R, -255, 255);
  lastPwmL = L;
  lastPwmR = R;
  ledcWrite(PWM_L1_CH, L >= 0 ? L : 0);
  ledcWrite(PWM_L2_CH, L < 0 ? -L : 0);
  ledcWrite(PWM_R3_CH, R >= 0 ? R : 0);
  ledcWrite(PWM_R4_CH, R < 0 ? -R : 0);
}
void stopMotors() {
  lastPwmL = 0;
  lastPwmR = 0;
  ledcWrite(PWM_L1_CH, 0);
  ledcWrite(PWM_L2_CH, 0);
  ledcWrite(PWM_R3_CH, 0);
  ledcWrite(PWM_R4_CH, 0);
}

// ══════════════════════════════════════════════════════════
//  Battery voltage
// ══════════════════════════════════════════════════════════
void readBattery() {
  unsigned long now = millis();
  if (now - lastBattRead < BATT_SAMPLE_MS)
    return;
  lastBattRead = now;
  // Average 4 samples for stability
  long acc = 0;
  for (int i = 0; i < 4; i++) {
    acc += analogRead(BATT_PIN);
    delay(1);
  }
  float adcV = (acc / 4) / 4095.0f * 3.3f;
  battVoltage = adcV * BATT_DIVIDER;
  bool wasLow = battLow;
  battLow = (battVoltage < battAlertThreshold && battVoltage > 0.5f);
  if (battLow && !wasLow)
    wsLog("[BATT] LOW! " + String(battVoltage, 2) + "V < " +
          String(battAlertThreshold, 2) + "V");
}

// ══════════════════════════════════════════════════════════
//  MPU6050
// ══════════════════════════════════════════════════════════
void setupMPU() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    wsLog("[MPU] Not found at 0x68");
    mpuPresent = false;
    return;
  }
  mpuPresent = true;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true); // Wake
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true); // Accel +/-2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission(true); // Gyro +/-500 deg/s
  wsLog("[MPU] Init OK.");
}

void readMPU() {
  if (!mpuPresent)
    return;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0)
    return;
  uint8_t bytes = Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)14, true);
  if (bytes < 14)
    return;

  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  int16_t tp = Wire.read() << 8 | Wire.read();
  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();

  gyroZ = (float)gz / 65.5f;
  accelY = (float)ay / 16384.0f;

  unsigned long now = millis();
  if (lastMpuTime > 0) {
    float dt = (now - lastMpuTime) / 1000.0f;
    if (abs(gyroZ) > 1.5f)
      yawAngle += gyroZ * dt; // deadband
  }
  lastMpuTime = now;
}

// ══════════════════════════════════════════════════════════
//  IR sensors
// ══════════════════════════════════════════════════════════
void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++)
    rawVals[i] = analogRead(IR_PINS[i]);
}
void evaluateSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int span = calMax[i] - calMin[i];
    sensorValid[i] = (span >= 150);
    wsLog("  IR" + String(i + 1) + " span=" + String(span) +
          (sensorValid[i] ? " OK" : " EXCLUDED"));
  }
}
void normaliseSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int v;
    if (!isCalibrated) {
      v = constrain(map(rawVals[i], 50, 4000, 0, 1000), 0, 1000);
    } else {
      if (!sensorValid[i]) {
        normVals[i] = 0;
        continue;
      }
      v = constrain(map(rawVals[i], calMin[i], calMax[i], 0, 1000), 0, 1000);
    }
    if (invertLine)
      v = 1000 - v;
    normVals[i] = v;
  }
}
int calcPosition() {
  long wSum = 0, sSum = 0;
  bool any = false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValid[i] && normVals[i] > 150) {
      wSum += (long)SENSOR_WEIGHTS[i] * normVals[i];
      sSum += normVals[i];
      any = true;
    }
  }
  lineLost = !any;
  if (!any)
    return lastPosition;
  lastPosition = (int)(wSum / sSum);
  return lastPosition;
}

// ══════════════════════════════════════════════════════════
//  Calibration
// ══════════════════════════════════════════════════════════
void startCalibration() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    calMin[i] = 4095;
    calMax[i] = 0;
  }
  isCalibrated = false;
  calibrating = true;
  calStart = millis();
  wsLog("[CAL] Started -- sweep bot over line for 8 seconds");
  wsLog("[CAL] Mode: " + (invertLine ? String("WHITE line on black BG")
                                     : String("BLACK line on white BG")));
}
void updateCalibration() {
  if (!calibrating)
    return;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (rawVals[i] < calMin[i])
      calMin[i] = rawVals[i];
    if (rawVals[i] > calMax[i])
      calMax[i] = rawVals[i];
  }
  if (millis() - calStart >= CAL_DURATION) {
    calibrating = false;
    isCalibrated = true;
    wsLog("[CAL] Done!");
    evaluateSensors();
    String s = buildStateJson();
    wsServer.broadcastTXT(s);
  }
}

// ══════════════════════════════════════════════════════════
//  PID (with hairpin + corner assist + line-lost recovery)
// ══════════════════════════════════════════════════════════
void runPID() {
  // ── Auto PID Tune ────────────────────────────────────────
  if (autoTuneActive) {
    int u = (lastPosition > 0) ? 60 : -60;
    setMotors(BASE_SPEED - u, BASE_SPEED + u);
    return;
  }

  // ── Line-lost recovery & Gyro Pivot ──────────────────────
  if (lineLost) {
    if (gyroTurnEnable && mpuPresent) {
      if (!isGyroTurning) {
        isGyroTurning = true;
        yawAngle = 0;
        wsLog("[MPU] 90-deg pivot start");
      }
      if (abs(yawAngle) >= 90.0f) {
        stopMotors();
        return;
      }
      int s = (lastPosition > 0) ? BASE_SPEED : -BASE_SPEED;
      setMotors(s, -s);
      return;
    }
    isGyroTurning = false;
    if (LINE_LOST_TIMEOUT == 0) {
      stopMotors();
      return;
    }
    if (!lineLostStart)
      lineLostStart = millis();
    if (millis() - lineLostStart > (unsigned long)LINE_LOST_TIMEOUT) {
      robotRunning = false;
      stopMotors();
      wsLog("[LOST] Timeout. Stopped.");
      return;
    }
    // Spin in direction of last error
    int spin = (lastPosition >= 0) ? BASE_SPEED : -BASE_SPEED;
    setMotors(-spin, spin);
    return;
  }
  lineLostStart = 0;
  isGyroTurning = false;

  // ── PID Update ───────────────────────────────────────────
  float currentError = (float)lastPosition;

  // Drift assist with MPU
  if (driftAssistEnable && mpuPresent && abs(currentError) < 1000.0f) {
    currentError -= gyroZ * driftAssistGain;
  }

  float rawDeriv = currentError - pidError;
  errorDerivative = 0.6f * errorDerivative + 0.4f * rawDeriv;

  if (CORNER_SENSITIVITY > 0 && !cornerActive &&
      fabsf(errorDerivative) > (float)CORNER_SENSITIVITY) {
    cornerActive = true;
    cornerStartMs = millis();
    wsLog("[CORNER] dErr=" + String((int)errorDerivative) +
          " err=" + String((int)currentError));
  }
  if (cornerActive &&
      (millis() - cornerStartMs) > (unsigned long)CORNER_DURATION_MS)
    cornerActive = false;

  pidError = currentError;
  pidIntegral = constrain(pidIntegral + pidError, -10000.0f, 10000.0f);

  float effKp = Kp * (cornerActive ? CORNER_BOOST : 1.0f);
  float rawCorr =
      effKp * pidError + Ki * pidIntegral + Kd * (pidError - pidLastError);
  pidLastError = pidError;

  float normCorr = rawCorr / (float)MAX_SPEED;
  float sign = (normCorr >= 0) ? 1.0f : -1.0f;
  float expCorr = sign * powf(fabsf(normCorr), CURVE_EXP) * (float)MAX_SPEED;
  float corr = expCorr * TURN_SHARPNESS;

  float normErr = constrain(fabsf(pidError) / 2500.0f, 0.0f, 1.0f);
  float errScale = 1.0f - normErr * (TURN_SPEED_REDUCE / 100.0f);
  int effBase = (int)(BASE_SPEED * errScale);
  if (cornerActive) {
    int cSpd = (int)(BASE_SPEED * (CORNER_SPEED_PCT / 100.0f));
    effBase = min(effBase, cSpd);
  }

  // ── Track Mapping & Speed Run Logic ──────────────────────
  if (mappingEnabled && mpuPresent) {
    SegmentType currentType = SEG_STRAIGHT;
    if (abs(gyroZ) > 80.0f || abs(pidError) > 800.0f) {
      // Assuming positive gyroZ or positive pidError means Right turn (depends
      // on sensor polarity, adjust if necessary)
      currentType = (gyroZ > 0 || pidError > 0) ? SEG_TURN_R : SEG_TURN_L;
    }

    if (segmentStartMs == 0) {
      segmentStartMs = millis();
      currentGyStateType = currentType;
      segmentCount = 0;
    } else if (currentType != currentGyStateType) {
      if (millis() - segmentStartMs > 50) {
        if (segmentCount < MAX_SEGMENTS) {
          trackMap[segmentCount].type = currentGyStateType;
          trackMap[segmentCount].durationMs = millis() - segmentStartMs;
          segmentCount++;
          String tName =
              currentGyStateType == SEG_STRAIGHT
                  ? "STRAIGHT"
                  : (currentGyStateType == SEG_TURN_L ? "LEFT" : "RIGHT");
          wsLog("[MAP] Seg " + String(segmentCount) + ": " + tName + " " +
                String(trackMap[segmentCount - 1].durationMs) + "ms");
        }
        currentGyStateType = currentType;
        segmentStartMs = millis();
      }
    }
  }

  // Speed Run Playback
  if (speedRunEnabled && trackMapped && segmentCount > 0 && mpuPresent) {
    SegmentType physicalType = SEG_STRAIGHT;
    if (abs(gyroZ) > 80.0f || abs(pidError) > 800.0f) {
      physicalType = (gyroZ > 0 || pidError > 0) ? SEG_TURN_R : SEG_TURN_L;
    }
    unsigned long elapsed = millis() - segmentStartMs;

    // Resync logic
    if (trackMap[currentSegmentIdx].type == SEG_STRAIGHT &&
        physicalType != SEG_STRAIGHT && elapsed > 100) {
      currentSegmentIdx = (currentSegmentIdx + 1) % segmentCount;
      segmentStartMs = millis();
      elapsed = 0;
    } else if (trackMap[currentSegmentIdx].type != SEG_STRAIGHT &&
               physicalType == SEG_STRAIGHT && elapsed > 100) {
      currentSegmentIdx = (currentSegmentIdx + 1) % segmentCount;
      segmentStartMs = millis();
      elapsed = 0;
    }

    if (trackMap[currentSegmentIdx].type == SEG_STRAIGHT) {
      unsigned long dur = trackMap[currentSegmentIdx].durationMs;
      if (dur > (unsigned long)preBrakeMs && elapsed > (dur - preBrakeMs)) {
        effBase = preBrakeSpeed; // Slam brakes
      } else {
        effBase = speedRunMax; // Sprint
      }
    }
  }

  effBase = constrain(effBase, MIN_SPEED, MAX_SPEED);

  int L = constrain(effBase - (int)corr, -MAX_SPEED, MAX_SPEED);
  int R = constrain(effBase + (int)corr, -MAX_SPEED, MAX_SPEED);

  if (TURN_BRAKE > 0) {
    float bs = normErr * (TURN_BRAKE / 100.0f);
    if (pidError > 0) {
      L = (int)(L - (L - MIN_SPEED) * bs);
      L = constrain(L, MIN_SPEED, MAX_SPEED);
    } else {
      R = (int)(R - (R - MIN_SPEED) * bs);
      R = constrain(R, MIN_SPEED, MAX_SPEED);
    }
  }

  bool doPivot =
      (PIVOT_THRESHOLD > 0 && fabsf(pidError) >= (float)PIVOT_THRESHOLD) ||
      (cornerActive && fabsf(pidError) > 800.0f);
  if (doPivot) {
    float pRef = cornerActive ? 800.0f : (float)PIVOT_THRESHOLD;
    float pDepth =
        constrain((fabsf(pidError) - pRef) / (2500.0f - pRef), 0.0f, 1.0f);
    if (cornerActive)
      pDepth = max(pDepth, 0.4f);
    int revSpd = (int)(MIN_SPEED + pDepth * (MAX_SPEED - MIN_SPEED));
    if (pidError > 0)
      L = -revSpd;
    else
      R = -revSpd;
  }

  setMotors(constrain(L, -MAX_SPEED, MAX_SPEED),
            constrain(R, -MAX_SPEED, MAX_SPEED));
}

// ══════════════════════════════════════════════════════════
//  Settings Profiles
// ══════════════════════════════════════════════════════════
String currentSettingsJson(const String &name) {
  StaticJsonDocument<512> d;
  d["name"] = name;
  d["kp"] = Kp;
  d["ki"] = Ki;
  d["kd"] = Kd;
  d["bs"] = BASE_SPEED;
  d["ms"] = MAX_SPEED;
  d["mn"] = MIN_SPEED;
  d["ts"] = TURN_SHARPNESS;
  d["tb"] = TURN_BRAKE;
  d["ce"] = CURVE_EXP;
  d["tsr"] = TURN_SPEED_REDUCE;
  d["pt"] = PIVOT_THRESHOLD;
  d["cs"] = CORNER_SENSITIVITY;
  d["cb"] = CORNER_BOOST;
  d["cd"] = CORNER_DURATION_MS;
  d["csp"] = CORNER_SPEED_PCT;
  d["llt"] = LINE_LOST_TIMEOUT;
  d["bat"] = battAlertThreshold;
  d["inv"] = invertLine;
  d["me"] = mappingEnabled;
  d["se"] = speedRunEnabled;
  d["srm"] = speedRunMax;
  d["pbm"] = preBrakeMs;
  d["pbs"] = preBrakeSpeed;
  String out;
  serializeJson(d, out);
  return out;
}

void applySettingsJson(const String &json) {
  StaticJsonDocument<512> d;
  if (deserializeJson(d, json))
    return;
  Kp = d["kp"] | Kp;
  Ki = d["ki"] | Ki;
  Kd = d["kd"] | Kd;
  BASE_SPEED = d["bs"] | BASE_SPEED;
  MAX_SPEED = d["ms"] | MAX_SPEED;
  MIN_SPEED = d["mn"] | MIN_SPEED;
  TURN_SHARPNESS = d["ts"] | TURN_SHARPNESS;
  TURN_BRAKE = d["tb"] | TURN_BRAKE;
  CURVE_EXP = d["ce"] | CURVE_EXP;
  TURN_SPEED_REDUCE = d["tsr"] | TURN_SPEED_REDUCE;
  PIVOT_THRESHOLD = d["pt"] | PIVOT_THRESHOLD;
  CORNER_SENSITIVITY = d["cs"] | CORNER_SENSITIVITY;
  CORNER_BOOST = d["cb"] | CORNER_BOOST;
  CORNER_DURATION_MS = d["cd"] | CORNER_DURATION_MS;
  CORNER_SPEED_PCT = d["csp"] | CORNER_SPEED_PCT;
  LINE_LOST_TIMEOUT = d["llt"] | LINE_LOST_TIMEOUT;
  battAlertThreshold = d["bat"] | battAlertThreshold;
  invertLine = d["inv"] | invertLine;
  mappingEnabled = d["me"] | mappingEnabled;
  speedRunEnabled = d["se"] | speedRunEnabled;
  speedRunMax = d["srm"] | speedRunMax;
  preBrakeMs = d["pbm"] | preBrakeMs;
  preBrakeSpeed = d["pbs"] | preBrakeSpeed;
  pidIntegral = 0;
  cornerActive = false;
}

void saveProfile(int slot, const String &name) {
  if (slot < 0 || slot >= MAX_PROFILES)
    return;
  prefs.begin("lfp", false);
  prefs.putString(("s" + String(slot)).c_str(), currentSettingsJson(name));
  prefs.end();
  wsLog("[PROF] Saved slot " + String(slot) + " \"" + name + "\"");
}

void loadProfile(int slot) {
  if (slot < 0 || slot >= MAX_PROFILES)
    return;
  prefs.begin("lfp", true);
  String key = "s" + String(slot);
  if (prefs.isKey(key.c_str())) {
    String json = prefs.getString(key.c_str(), "");
    prefs.end();
    applySettingsJson(json);
    wsLog("[PROF] Loaded slot " + String(slot));
  } else {
    prefs.end();
    wsLog("[PROF] Slot " + String(slot) + " empty");
  }
}

void deleteProfile(int slot) {
  if (slot < 0 || slot >= MAX_PROFILES)
    return;
  prefs.begin("lfp", false);
  prefs.remove(("s" + String(slot)).c_str());
  prefs.end();
  wsLog("[PROF] Deleted slot " + String(slot));
}

String listProfilesJson() {
  StaticJsonDocument<512> doc;
  doc["type"] = "profiles";
  JsonArray arr = doc.createNestedArray("list");
  prefs.begin("lfp", true);
  for (int i = 0; i < MAX_PROFILES; i++) {
    String key = "s" + String(i);
    if (prefs.isKey(key.c_str())) {
      String raw = prefs.getString(key.c_str(), "{}");
      StaticJsonDocument<512> pd;
      if (!deserializeJson(pd, raw)) {
        JsonObject o = arr.createNestedObject();
        o["slot"] = i;
        o["name"] = pd["name"] | String("Slot " + String(i));
      }
    }
  }
  prefs.end();
  String out;
  serializeJson(doc, out);
  return out;
}

// ══════════════════════════════════════════════════════════
//  Flash (save / load all)
// ══════════════════════════════════════════════════════════
void saveAll() {
  prefs.begin("lf", false);
  for (int i = 0; i < NUM_SENSORS; i++) {
    prefs.putInt(("n" + String(i)).c_str(), calMin[i]);
    prefs.putInt(("x" + String(i)).c_str(), calMax[i]);
  }
  prefs.putFloat("kp", Kp);
  prefs.putFloat("ki", Ki);
  prefs.putFloat("kd", Kd);
  prefs.putInt("bs", BASE_SPEED);
  prefs.putInt("ms", MAX_SPEED);
  prefs.putInt("mn", MIN_SPEED);
  prefs.putFloat("ts", TURN_SHARPNESS);
  prefs.putInt("tb", TURN_BRAKE);
  prefs.putFloat("ce", CURVE_EXP);
  prefs.putInt("tsr", TURN_SPEED_REDUCE);
  prefs.putInt("pt", PIVOT_THRESHOLD);
  prefs.putInt("cs", CORNER_SENSITIVITY);
  prefs.putFloat("cb", CORNER_BOOST);
  prefs.putInt("cd", CORNER_DURATION_MS);
  prefs.putInt("csp", CORNER_SPEED_PCT);
  prefs.putBool("inv", invertLine);
  prefs.putBool("eze", endZoneEnable);
  prefs.putInt("ezm", endZoneMinSensors);
  prefs.putInt("ezc", endZoneConfirmMs);
  prefs.putInt("ezt", endZoneThreshold);
  prefs.putInt("llt", LINE_LOST_TIMEOUT);
  prefs.putBool("lpm", lapTimerMode);
  prefs.putFloat("bat", battAlertThreshold);
  prefs.putBool("mpuA", driftAssistEnable);
  prefs.putFloat("mpuG", driftAssistGain);
  prefs.putBool("mpuT", gyroTurnEnable);
  prefs.putBool("me", mappingEnabled);
  prefs.putBool("se", speedRunEnabled);
  prefs.putInt("srm", speedRunMax);
  prefs.putInt("pbm", preBrakeMs);
  prefs.putInt("pbs", preBrakeSpeed);
  prefs.end();
  wsLog("[FLASH] Saved.");
}

void loadAll() {
  prefs.begin("lf", true);
  if (prefs.isKey("n0")) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      calMin[i] = prefs.getInt(("n" + String(i)).c_str(), 0);
      calMax[i] = prefs.getInt(("x" + String(i)).c_str(), 4095);
    }
    Kp = prefs.getFloat("kp", 0.4f);
    Ki = prefs.getFloat("ki", 0.0f);
    Kd = prefs.getFloat("kd", 1.2f);
    BASE_SPEED = prefs.getInt("bs", 150);
    MAX_SPEED = prefs.getInt("ms", 200);
    MIN_SPEED = prefs.getInt("mn", 40);
    TURN_SHARPNESS = prefs.getFloat("ts", 1.0f);
    TURN_BRAKE = prefs.getInt("tb", 0);
    CURVE_EXP = prefs.getFloat("ce", 1.0f);
    TURN_SPEED_REDUCE = prefs.getInt("tsr", 0);
    PIVOT_THRESHOLD = prefs.getInt("pt", 0);
    CORNER_SENSITIVITY = prefs.getInt("cs", 0);
    CORNER_BOOST = prefs.getFloat("cb", 2.0f);
    CORNER_DURATION_MS = prefs.getInt("cd", 200);
    CORNER_SPEED_PCT = prefs.getInt("csp", 50);
    invertLine = prefs.getBool("inv", false);
    endZoneEnable = prefs.getBool("eze", true);
    endZoneMinSensors = prefs.getInt("ezm", 5);
    endZoneConfirmMs = prefs.getInt("ezc", 100);
    endZoneThreshold = prefs.getInt("ezt", 500);
    LINE_LOST_TIMEOUT = prefs.getInt("llt", 1500);
    lapTimerMode = prefs.getBool("lpm", false);
    battAlertThreshold = prefs.getFloat("bat", 3.5f);
    driftAssistEnable = prefs.getBool("mpuA", false);
    driftAssistGain = prefs.getFloat("mpuG", 10.0f);
    gyroTurnEnable = prefs.getBool("mpuT", false);
    mappingEnabled = prefs.getBool("me", false);
    speedRunEnabled = prefs.getBool("se", false);
    speedRunMax = prefs.getInt("srm", 255);
    preBrakeMs = prefs.getInt("pbm", 150);
    preBrakeSpeed = prefs.getInt("pbs", 80);
    isCalibrated = true;
    evaluateSensors();
    wsLog("[FLASH] Loaded calibration.");
  }
  prefs.end();
}

// ══════════════════════════════════════════════════════════
//  WebSocket JSON builders
// ══════════════════════════════════════════════════════════
String buildStateJson() {
  StaticJsonDocument<900> doc;
  doc["type"] = "state";
  doc["running"] = robotRunning;
  doc["calibrating"] = calibrating;
  doc["calibrated"] = isCalibrated;
  doc["kp"] = Kp;
  doc["ki"] = Ki;
  doc["kd"] = Kd;
  doc["baseSpeed"] = BASE_SPEED;
  doc["maxSpeed"] = MAX_SPEED;
  doc["minSpeed"] = MIN_SPEED;
  doc["turnSharpness"] = TURN_SHARPNESS;
  doc["turnBrake"] = TURN_BRAKE;
  doc["curveExp"] = CURVE_EXP;
  doc["turnSpeedReduce"] = TURN_SPEED_REDUCE;
  doc["pivotThreshold"] = PIVOT_THRESHOLD;
  doc["cornerSensitivity"] = CORNER_SENSITIVITY;
  doc["cornerBoost"] = CORNER_BOOST;
  doc["cornerDuration"] = CORNER_DURATION_MS;
  doc["cornerSpeedPct"] = CORNER_SPEED_PCT;
  doc["cornerActive"] = cornerActive;
  doc["invertLine"] = invertLine;
  doc["endZoneEnable"] = endZoneEnable;
  doc["endZoneMinSensors"] = endZoneMinSensors;
  doc["endZoneConfirmMs"] = endZoneConfirmMs;
  doc["endZoneThreshold"] = endZoneThreshold;
  doc["endZoneTriggered"] = endZoneTriggered;
  // v5.1 additions
  doc["lineLost"] = lineLost;
  doc["lineLostTimeout"] = LINE_LOST_TIMEOUT;
  doc["lapTimerMode"] = lapTimerMode;
  doc["lapCount"] = lapCount;
  JsonArray lt = doc.createNestedArray("lapTimes");
  int n = min(lapCount, MAX_LAP_TIMES);
  for (int i = 0; i < n; i++)
    lt.add(lapTimes[(lapCount - n + i) % MAX_LAP_TIMES]);
  doc["battVoltage"] = battVoltage;
  doc["battAlertThreshold"] = battAlertThreshold;
  doc["battLow"] = battLow;
  doc["mpuPresent"] = mpuPresent;
  doc["driftAssistEnable"] = driftAssistEnable;
  doc["driftAssistGain"] = driftAssistGain;
  doc["gyroTurnEnable"] = gyroTurnEnable;
  doc["autoTuneActive"] = autoTuneActive;
  doc["mappingEnabled"] = mappingEnabled;
  doc["speedRunEnabled"] = speedRunEnabled;
  doc["speedRunMax"] = speedRunMax;
  doc["preBrakeMs"] = preBrakeMs;
  doc["preBrakeSpeed"] = preBrakeSpeed;
  doc["trackMapped"] = trackMapped;
  doc["segmentCount"] = segmentCount;
  String out;
  serializeJson(doc, out);
  return out;
}

String buildSensorJson() {
  StaticJsonDocument<512> doc;
  doc["type"] = "sensors";
  JsonArray norm = doc.createNestedArray("norm");
  JsonArray raw = doc.createNestedArray("raw");
  for (int i = 0; i < NUM_SENSORS; i++) {
    norm.add(normVals[i]);
    raw.add(rawVals[i]);
  }
  doc["position"] = lastPosition;
  doc["error"] = pidError;
  doc["pwmL"] = lastPwmL;
  doc["pwmR"] = lastPwmR;
  if (mpuPresent) {
    doc["gyroZ"] = gyroZ;
    doc["accelY"] = accelY;
    doc["yawAngle"] = yawAngle;
  }
  String out;
  serializeJson(doc, out);
  return out;
}

String buildLogJson() {
  StaticJsonDocument<2048> doc;
  doc["type"] = "log";
  JsonArray arr = doc.createNestedArray("lines");
  int start = (logHead - logCount + LOG_BUF) % LOG_BUF;
  for (int i = 0; i < logCount; i++)
    arr.add(logLines[(start + i) % LOG_BUF]);
  logCount = 0;
  String out;
  serializeJson(doc, out);
  return out;
}

// ══════════════════════════════════════════════════════════
//  End-zone / Lap detection
// ══════════════════════════════════════════════════════════
void resetEndZone() {
  endZonePending = false;
  endZoneTriggered = false;
  endZoneFirstSeen = 0;
  lapCount = 0;
  lapStartMs = millis();
  for (int i = 0; i < MAX_LAP_TIMES; i++)
    lapTimes[i] = 0;
}

bool detectEndZone() {
  if (!endZoneEnable || !robotRunning)
    return false;
  int active = 0;
  for (int i = 0; i < NUM_SENSORS; i++)
    if (sensorValid[i] && normVals[i] >= endZoneThreshold)
      active++;
  bool condMet = (active >= endZoneMinSensors);
  if (condMet) {
    if (!endZonePending) {
      endZonePending = true;
      endZoneFirstSeen = millis();
    } else if (millis() - endZoneFirstSeen >= (unsigned long)endZoneConfirmMs) {
      endZonePending = false;
      if (lapTimerMode) {
        // Count lap, don't stop
        unsigned long lapMs = millis() - lapStartMs;
        lapTimes[lapCount % MAX_LAP_TIMES] = lapMs;
        lapCount++;
        lapStartMs = millis();
        if (mappingEnabled) {
          trackMapped = true;
          mappingEnabled = false;
          wsLog("[MAP] Recorded " + String(segmentCount) + " segments");
        }
        if (speedRunEnabled && trackMapped) {
          currentSegmentIdx = 0;
          segmentStartMs = millis();
          wsLog("[MAP] Speed Run Lap!");
        }
        wsLog("[LAP] #" + String(lapCount) + " time=" + String(lapMs) + "ms");
        String s = buildStateJson();
        wsServer.broadcastTXT(s);
        // Brief pause to avoid counting same crossing twice
        delay(200);
      } else {
        robotRunning = false;
        endZoneTriggered = true;
        stopMotors();
        if (mappingEnabled) {
          trackMapped = true;
          mappingEnabled = false;
          wsLog("[MAP] Recorded " + String(segmentCount) + " segments");
        }
        wsLog("[END] Finish line! " + String(active) + " sensors. Stopped.");
        String s = buildStateJson();
        wsServer.broadcastTXT(s);
        return true;
      }
    }
  } else {
    endZonePending = false;
  }
  return false;
}

// ══════════════════════════════════════════════════════════
//  WebSocket event handler
// ══════════════════════════════════════════════════════════
void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) {
    String s1 = buildStateJson();
    wsServer.sendTXT(num, s1);
    String s2 = buildLogJson();
    wsServer.sendTXT(num, s2);
    String s3 = listProfilesJson();
    wsServer.sendTXT(num, s3);
    return;
  }
  if (type != WStype_TEXT)
    return;

  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload, length))
    return;
  const char *c = doc["cmd"];
  if (!c)
    return;

  if (!strcmp(c, "start")) {
    robotRunning = true;
    pidIntegral = 0;
    pidLastError = 0;
    lineLostStart = 0;
    resetEndZone();
    wsLog("[CMD] Start");
  } else if (!strcmp(c, "stop")) {
    robotRunning = false;
    calibrating = false;
    stopMotors();
    wsLog("[CMD] Stop");
  } else if (!strcmp(c, "calibrate")) {
    startCalibration();
  } else if (!strcmp(c, "save")) {
    saveAll();
  } else if (!strcmp(c, "pid")) {
    Kp = doc["kp"] | Kp;
    Ki = doc["ki"] | Ki;
    Kd = doc["kd"] | Kd;
    pidIntegral = 0;
    wsLog("[PID] Kp=" + String(Kp, 2) + " Ki=" + String(Ki, 3) +
          " Kd=" + String(Kd, 2));
  } else if (!strcmp(c, "speed")) {
    BASE_SPEED = doc["baseSpeed"] | BASE_SPEED;
    MAX_SPEED = doc["maxSpeed"] | MAX_SPEED;
    MIN_SPEED = doc["minSpeed"] | MIN_SPEED;
    wsLog("[SPD] base=" + String(BASE_SPEED) + " max=" + String(MAX_SPEED) +
          " min=" + String(MIN_SPEED));
  } else if (!strcmp(c, "turn")) {
    TURN_SHARPNESS = doc["sharpness"] | TURN_SHARPNESS;
    TURN_BRAKE = doc["brake"] | TURN_BRAKE;
    CURVE_EXP = doc["curveExp"] | CURVE_EXP;
    TURN_SPEED_REDUCE = doc["turnSpeedReduce"] | TURN_SPEED_REDUCE;
    PIVOT_THRESHOLD = doc["pivotThreshold"] | PIVOT_THRESHOLD;
    CORNER_SENSITIVITY = doc["cornerSensitivity"] | CORNER_SENSITIVITY;
    CORNER_BOOST = doc["cornerBoost"] | CORNER_BOOST;
    CORNER_DURATION_MS = doc["cornerDuration"] | CORNER_DURATION_MS;
    CORNER_SPEED_PCT = doc["cornerSpeedPct"] | CORNER_SPEED_PCT;
    TURN_SHARPNESS = constrain(TURN_SHARPNESS, 0.1f, 3.0f);
    TURN_BRAKE = constrain(TURN_BRAKE, 0, 100);
    CURVE_EXP = constrain(CURVE_EXP, 1.0f, 3.0f);
    TURN_SPEED_REDUCE = constrain(TURN_SPEED_REDUCE, 0, 100);
    PIVOT_THRESHOLD = constrain(PIVOT_THRESHOLD, 0, 2500);
    CORNER_SENSITIVITY = constrain(CORNER_SENSITIVITY, 0, 2000);
    CORNER_BOOST = constrain(CORNER_BOOST, 1.0f, 5.0f);
    CORNER_DURATION_MS = constrain(CORNER_DURATION_MS, 50, 1000);
    CORNER_SPEED_PCT = constrain(CORNER_SPEED_PCT, 10, 100);
    pidIntegral = 0;
    cornerActive = false;
    wsLog("[TURN] sharp=" + String(TURN_SHARPNESS, 2) +
          " exp=" + String(CURVE_EXP, 1));
  } else if (!strcmp(c, "endzone")) {
    endZoneEnable = doc["enable"] | endZoneEnable;
    endZoneMinSensors = doc["minSensors"] | endZoneMinSensors;
    endZoneConfirmMs = doc["confirmMs"] | endZoneConfirmMs;
    endZoneThreshold = doc["threshold"] | endZoneThreshold;
    endZoneMinSensors = constrain(endZoneMinSensors, 1, NUM_SENSORS);
    endZoneConfirmMs = constrain(endZoneConfirmMs, 10, 500);
    endZoneThreshold = constrain(endZoneThreshold, 100, 900);
    wsLog("[EZ] enable=" + String(endZoneEnable) +
          " minS=" + String(endZoneMinSensors));
  } else if (!strcmp(c, "linemode")) {
    bool ni = doc["invert"] | invertLine;
    if (ni != invertLine) {
      invertLine = ni;
      pidIntegral = 0;
      pidLastError = 0;
      wsLog("[MODE] " +
            (invertLine ? String("WHITE line") : String("BLACK line")));
    }
  } else if (!strcmp(c, "linelost")) {
    LINE_LOST_TIMEOUT = doc["timeout"] | LINE_LOST_TIMEOUT;
    wsLog("[LOST] timeout=" + String(LINE_LOST_TIMEOUT) + "ms");
  } else if (!strcmp(c, "mpu")) {
    if (doc.containsKey("driftAssistEnable"))
      driftAssistEnable = doc["driftAssistEnable"];
    if (doc.containsKey("driftAssistGain"))
      driftAssistGain = doc["driftAssistGain"];
    if (doc.containsKey("gyroTurnEnable"))
      gyroTurnEnable = doc["gyroTurnEnable"];
    if (doc.containsKey("autoTuneActive"))
      autoTuneActive = doc["autoTuneActive"];
    wsLog("[MPU] Settings updated");
  } else if (!strcmp(c, "trackmap")) {
    if (doc.containsKey("mappingEnabled"))
      mappingEnabled = doc["mappingEnabled"];
    if (doc.containsKey("speedRunEnabled"))
      speedRunEnabled = doc["speedRunEnabled"];
    if (doc.containsKey("speedRunMax"))
      speedRunMax = doc["speedRunMax"];
    if (doc.containsKey("preBrakeMs"))
      preBrakeMs = doc["preBrakeMs"];
    if (doc.containsKey("preBrakeSpeed"))
      preBrakeSpeed = doc["preBrakeSpeed"];
    if (doc.containsKey("clearMap") && doc["clearMap"]) {
      trackMapped = false;
      segmentCount = 0;
      wsLog("[MAP] Cleared");
    }
    wsLog("[MAP] Sync");
  } else if (!strcmp(c, "trackmap_get")) {
    DynamicJsonDocument md(4096);
    md["type"] = "map_data";
    JsonArray arr = md.createNestedArray("segs");
    for (int i = 0; i < segmentCount; i++) {
      JsonObject o = arr.createNestedObject();
      o["t"] = trackMap[i].type;
      o["d"] = trackMap[i].durationMs;
    }
    String out;
    serializeJson(md, out);
    wsServer.sendTXT(num, out);
  } else if (!strcmp(c, "laptimer")) {
    lapTimerMode = doc["enable"] | lapTimerMode;
    wsLog("[LAP] mode=" + String(lapTimerMode));
  } else if (!strcmp(c, "batt")) {
    battAlertThreshold = doc["threshold"] | battAlertThreshold;
    wsLog("[BATT] alert=" + String(battAlertThreshold, 2) + "V");
  } else if (!strcmp(c, "saveprofile")) {
    int slot = doc["slot"] | 0;
    String nm = doc["name"] | String("Slot " + String(slot));
    saveProfile(slot, nm);
    String s = listProfilesJson();
    wsServer.broadcastTXT(s);
  } else if (!strcmp(c, "loadprofile")) {
    int slot = doc["slot"] | 0;
    loadProfile(slot);
    String s = listProfilesJson();
    wsServer.broadcastTXT(s);
    wsLog("[PROF] Applied slot " + String(slot));
  } else if (!strcmp(c, "deleteprofile")) {
    int slot = doc["slot"] | 0;
    deleteProfile(slot);
    String s = listProfilesJson();
    wsServer.broadcastTXT(s);
  } else if (!strcmp(c, "getprofiles")) {
    String s = listProfilesJson();
    wsServer.sendTXT(num, s);
    return;
  }

  else if (!strcmp(c, "rescan_mpu")) {
    setupMPU();
  }

  String s = buildStateJson();
  wsServer.broadcastTXT(s);
}

// ══════════════════════════════════════════════════════════
//  HTTP setup (includes OTA /update endpoint)
// ══════════════════════════════════════════════════════════
void setupHTTP() {
  httpServer.on("/", []() { httpServer.send_P(200, "text/html", INDEX_HTML); });

  // OTA upload endpoint
  httpServer.on(
      "/update", HTTP_POST,
      []() {
        httpServer.send(200, "text/plain", Update.hasError() ? "FAIL" : "OK");
        ESP.restart();
      },
      []() {
        HTTPUpload &up = httpServer.upload();
        if (up.status == UPLOAD_FILE_START) {
          wsLog("[OTA] Start: " + String(up.filename.c_str()));
          if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            Update.printError(Serial);
          }
        } else if (up.status == UPLOAD_FILE_WRITE) {
          if (Update.write(up.buf, up.currentSize) != up.currentSize)
            Update.printError(Serial);
        } else if (up.status == UPLOAD_FILE_END) {
          if (Update.end(true))
            wsLog("[OTA] Success! " + String(up.totalSize) + " bytes");
          else
            Update.printError(Serial);
        }
      });

  httpServer.onNotFound(
      []() { httpServer.send(404, "text/plain", "Not found"); });
}

// ══════════════════════════════════════════════════════════
//  setup()
// ══════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(400);
  wsLog("===== Line Follower v5.2 =====");

  setupMPU();

  for (int i = 0; i < NUM_SENSORS; i++)
    sensorValid[i] = true;
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(IR_PINS[i], INPUT);
    analogSetPinAttenuation(IR_PINS[i], ADC_11db);
  }
  // Battery pin
  pinMode(BATT_PIN, INPUT);

  motorSetup();
  stopMotors();
  loadAll();

  wsLog("[WIFI] Connecting to " + String(SSID) + "...");
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  wsLog("[WIFI] IP: " + WiFi.localIP().toString());
  wsLog("[WEB]  http://" + WiFi.localIP().toString() + "/");
  wsLog("[OTA]  http://" + WiFi.localIP().toString() + "/update");
  wsLog("==============================");

  setupHTTP();
  httpServer.begin();
  wsServer.begin();
  wsServer.onEvent(onWsEvent);

  lapStartMs = millis();
}

// ══════════════════════════════════════════════════════════
//  loop()
// ══════════════════════════════════════════════════════════
void loop() {
  httpServer.handleClient();
  wsServer.loop();

  readSensors();
  normaliseSensors();
  calcPosition();
  readBattery();
  readMPU();

  if (calibrating) {
    updateCalibration();
  } else if (robotRunning) {
    detectEndZone();
    if (robotRunning)
      runPID();
  }

  unsigned long now = millis();
  if (now - lastWsBroadcast >= WS_INTERVAL) {
    lastWsBroadcast = now;
    String s1 = buildSensorJson();
    wsServer.broadcastTXT(s1);
    if (logCount > 0) {
      String s2 = buildLogJson();
      wsServer.broadcastTXT(s2);
    }
  }
}
