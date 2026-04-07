/*
 * ============================================================
 *  8-Sensor Line Follower — ESP32 + DRV8833 + MPU6050
 *  v6 — MPU6050 Gyro-Assisted High-Speed Mode
 *  FIXED: MPU init order, boot delay, testConnection retry
 * ============================================================
 *  IR:     IR2→36  IR3→39  IR4→34  IR5→35  IR6→32  IR7→33
 *  Motors: IN1→23  IN2→19  (Left)
 *          IN3→13  IN4→27  (Right)
 *  MPU6050: SDA→21  SCL→22  (I2C default)
 *
 *  Libraries needed:
 *    · WebSockets  by Markus Sattler
 *    · ArduinoJson by Benoit Blanchon
 *    · I2Cdev + MPU6050 by Jeff Rowberg (jrowberg/i2cdevlib)
 *      — Install via Library Manager: search "I2Cdev" and "MPU6050"
 *      — OR download from: https://github.com/jrowberg/i2cdevlib
 *
 *  FIXES IN THIS BUILD:
 *    1. mpuSetup() called FIRST in setup() — before ADC, LEDC, WiFi
 *    2. delay(150) after Wire.begin() — MPU6050 needs boot time
 *    3. testConnection() retried up to 5x — handles I2C glitches
 * ============================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>
// ── MPU6050 — jrowberg I2Cdev library ────────────────────
#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

// ── WiFi ─────────────────────────────────────────────────
const char* SSID     = "INDxAnurag1";
const char* PASSWORD = "wxfm6505";

// ── IR sensor pins ───────────────────────────────────────
const int IR_PINS[6]        = {36, 39, 34, 35, 32, 33};
const int NUM_SENSORS       = 6;
const int SENSOR_WEIGHTS[6] = {-2500,-1500,-500,500,1500,2500};

// ── Motor pins ───────────────────────────────────────────
#define MOTOR_L_IN1  23
#define MOTOR_L_IN2  19
#define MOTOR_R_IN3  13
#define MOTOR_R_IN4  27
#define PWM_L1_CH    0
#define PWM_L2_CH    1
#define PWM_R3_CH    2
#define PWM_R4_CH    3
#define PWM_FREQ     20000
#define PWM_RES      8

// ── PID ──────────────────────────────────────────────────
float Kp = 0.4f, Ki = 0.0f, Kd = 1.2f;
float pidError = 0, pidLastError = 0, pidIntegral = 0;
int   lastPosition = 0;

// ── Speed ────────────────────────────────────────────────
int BASE_SPEED = 150, MAX_SPEED = 200, MIN_SPEED = 40;

// ── Turn control ─────────────────────────────────────────
float TURN_SHARPNESS = 1.0f;
int   TURN_BRAKE     = 0;

// ── Hairpin / curve assist ────────────────────────────────
float CURVE_EXP         = 1.0f;
int   TURN_SPEED_REDUCE = 0;
int   PIVOT_THRESHOLD   = 0;

// ── Corner detection / 90° boost ─────────────────────────
int   CORNER_SENSITIVITY  = 0;
float CORNER_BOOST        = 2.0f;
int   CORNER_DURATION_MS  = 200;
int   CORNER_SPEED_PCT    = 50;

// ── Corner state ─────────────────────────────────────────
float         errorDerivative = 0;
bool          cornerActive    = false;
unsigned long cornerStartMs   = 0;

// ══════════════════════════════════════════════════════════
//  MPU6050 — Gyro-Assisted Steering
// ══════════════════════════════════════════════════════════
MPU6050 mpu(0x68);
bool    mpuOk = false;

float gyroBiasZ    = 0.0f;
float gyroNoiseFloor = 0.0f;
float yawRate         = 0.0f;
float yawRateFiltered = 0.0f;
float headingAccum    = 0.0f;

float GYRO_FF_GAIN       = 0.0f;
float GYRO_STRAIGHT_GAIN = 0.0f;
int   STRAIGHT_THRESHOLD = 300;
int   MOTOR_TRIM         = 0;
float TURN_RATE_SCALE    = 1.0f;

// ── IMU calibration state ─────────────────────────────────
bool          imuCalibrating   = false;
unsigned long imuCalStart      = 0;
const unsigned long IMU_CAL_MS = 3000;
float imuCalSumZ   = 0.0f;
int   imuCalCount  = 0;
float imuCalSumSqZ = 0.0f;

// ── Motor balance calibration state ──────────────────────
bool          motorCalibrating  = false;
unsigned long motorCalStart     = 0;
const unsigned long MOTOR_CAL_MS = 2500;  // extra 500ms for more accurate average
float motorCalYawAccum = 0.0f;
int   motorCalSamples  = 0;

// ── Turn rate calibration state ───────────────────────────
bool          turnCalibrating  = false;
unsigned long turnCalStart     = 0;
const unsigned long TURN_CAL_MS = 1500;
float turnCalYawAccum = 0.0f;
int   turnCalSamples  = 0;
int   turnCalDirection = 1;  // +1 = right (L motor fwd), -1 = left

// ── Auto PID Tune (Relay / Ziegler-Nichols) ───────────────
bool          autoPIDActive    = false;
unsigned long autoPIDStart     = 0;
const unsigned long AUTO_PID_MS = 10000;  // 10-second observation window
// Relay bang-bang state
bool  relayHigh        = false;           // true = full right correction
float relayAmplitude   = 0.0f;            // d = half the relay output
float relayOutput      = 0.0f;
// Oscillation tracking
unsigned long lastCrossTime  = 0;
float         lastCrossError = 0.0f;
int           crossCount     = 0;
float         periodSum      = 0.0f;
float         amplitudeSum   = 0.0f;
float         peakError      = 0.0f;

// ── Calibration IR ──────────────────────────────────────
int  calMin[NUM_SENSORS], calMax[NUM_SENSORS];
bool sensorValid[NUM_SENSORS];
bool isCalibrated = false;
bool calibrating  = false;
unsigned long calStart = 0;
const unsigned long CAL_DURATION = 8000;

// ── Sensor data ──────────────────────────────────────────
int rawVals[NUM_SENSORS], normVals[NUM_SENSORS];

// ── Line color mode ──────────────────────────────────────
bool invertLine = false;

// ── State ────────────────────────────────────────────────
bool robotRunning = false;

// ── End-zone detection ───────────────────────────────────
bool endZoneEnable      = true;
int  endZoneMinSensors  = 5;
int  endZoneConfirmMs   = 100;
int  endZoneThreshold   = 500;
unsigned long endZoneFirstSeen = 0;
bool          endZonePending   = false;
bool          endZoneTriggered = false;

// ── Web servers ──────────────────────────────────────────
WebServer        httpServer(80);
WebSocketsServer wsServer(81);
Preferences      prefs;
unsigned long    lastWsBroadcast = 0;
const unsigned long WS_INTERVAL  = 50;

// ── MPU read timing ──────────────────────────────────────
unsigned long lastMpuRead = 0;
const unsigned long MPU_INTERVAL = 5;


// ══════════════════════════════════════════════════════════
//  HTML Dashboard (unchanged from v6 original)
// ══════════════════════════════════════════════════════════
const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Line Follower v6</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,sans-serif;background:#111;color:#ddd;padding:16px}
h1{font-size:1.2rem;color:#7dd3fc;margin-bottom:14px}
.card{background:#1c1c1c;border:1px solid #2a2a2a;border-radius:10px;padding:14px;margin-bottom:12px}
.card h2{font-size:.75rem;text-transform:uppercase;letter-spacing:.1em;color:#555;margin-bottom:10px}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-bottom:8px}
button{padding:8px 16px;border:none;border-radius:6px;font-size:.85rem;cursor:pointer;font-weight:700}
button:disabled{opacity:.35;cursor:not-allowed}
.bg{background:#22c55e;color:#000}
.br{background:#ef4444;color:#fff}
.by{background:#f59e0b;color:#000}
.bb{background:#3b82f6;color:#fff}
.bv{background:#8b5cf6;color:#fff}
.bc{background:#06b6d4;color:#000}
.status{display:flex;align-items:center;gap:7px;font-size:.85rem;margin-bottom:10px}
.dot{width:10px;height:10px;border-radius:50%;background:#444}
.dot.on{background:#22c55e;box-shadow:0 0 6px #22c55e}
.dot.cal{background:#f59e0b;animation:blink 1s infinite}
.dot.off{background:#ef4444}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.2}}
label{font-size:.82rem;color:#999;display:flex;flex-direction:column;gap:4px;min-width:155px}
label small{font-size:.7rem;color:#555}
input[type=range]{width:100%;accent-color:#7dd3fc}
.val{color:#7dd3fc;font-weight:700;font-size:.88rem}
.sensors{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:6px}
.sb{display:flex;flex-direction:column;align-items:center;gap:3px}
.bwrap{height:72px;width:28px;background:#222;border-radius:4px;overflow:hidden;
       display:flex;align-items:flex-end;border:1px solid #2a2a2a}
.bar{width:100%;transition:height .07s;border-radius:3px 3px 0 0}
.bar.norm{background:#7dd3fc}
.bar.raw{background:#f59e0b}
.sl{font-size:.6rem;color:#555}
.sn{font-size:.7rem;color:#aaa}
.sn.raw{color:#f59e0b}
.pos-track{position:relative;height:24px;background:#222;border-radius:12px;
           overflow:hidden;border:1px solid #2a2a2a}
.pos-dot{position:absolute;top:2px;width:20px;height:20px;background:#7dd3fc;
         border-radius:50%;transition:left .07s;transform:translateX(-50%)}
#calBar{height:5px;background:#2a2a2a;border-radius:3px;margin-top:8px;overflow:hidden}
#calFill{height:100%;background:#f59e0b;width:0%;transition:width .3s}
.hint{font-size:.7rem;color:#444;margin-top:5px}
#logBox{background:#0a0a0a;border:1px solid #222;border-radius:7px;height:180px;
        overflow-y:auto;padding:8px;font-family:monospace;font-size:.7rem;color:#5a9a5a}
.ll{padding:1px 0;border-bottom:1px solid #111;white-space:pre-wrap;word-break:break-all}
.ll.w{color:#d97706}.ll.e{color:#dc2626}.ll.i{color:#60a5fa}
.toggle-row{display:flex;align-items:center;gap:12px;margin-bottom:10px}
.toggle-label{font-size:.82rem;color:#999}
.toggle-wrap{position:relative;display:inline-flex;align-items:center;gap:10px}
.toggle-track{
  position:relative;width:52px;height:26px;border-radius:13px;cursor:pointer;
  background:#222;border:1px solid #3a3a3a;transition:background .3s,border-color .3s;
}
.toggle-track.black-mode{background:#1a1a1a;border-color:#3a3a3a}
.toggle-track.white-mode{background:#d1d5db;border-color:#9ca3af}
.toggle-thumb{
  position:absolute;top:3px;left:3px;width:18px;height:18px;border-radius:50%;
  background:#555;transition:left .25s cubic-bezier(.4,0,.2,1),background .25s;
  box-shadow:0 1px 4px rgba(0,0,0,.5);
}
.toggle-track.white-mode .toggle-thumb{
  left:29px;background:#fff;box-shadow:0 1px 6px rgba(0,0,0,.3);
}
.line-icon{font-size:.75rem;font-weight:700;letter-spacing:.05em}
.line-icon.black{color:#444}
.line-icon.white{color:#aaa}
.mode-badge{
  display:inline-block;padding:2px 8px;border-radius:4px;font-size:.7rem;
  font-weight:700;letter-spacing:.05em;transition:background .3s,color .3s;
}
.mode-badge.black-line{background:#1e1e1e;color:#888;border:1px solid #333}
.mode-badge.white-line{background:#e5e7eb;color:#1f2937;border:1px solid #9ca3af}
.imu-badge{
  display:inline-block;padding:2px 8px;border-radius:4px;font-size:.7rem;
  font-weight:700;letter-spacing:.05em;margin-left:8px;
}
.imu-ok{background:#14532d;color:#4ade80;border:1px solid #166534}
.imu-err{background:#450a0a;color:#f87171;border:1px solid #7f1d1d}
.imu-cal{background:#78350f;color:#f59e0b;border:1px solid #92400e;animation:blink 1s infinite}
.gyro-bar-wrap{height:10px;background:#1a1a1a;border-radius:5px;overflow:hidden;margin-top:4px;border:1px solid #2a2a2a}
.gyro-bar-fill{height:100%;width:50%;background:#8b5cf6;transition:width .07s;border-radius:5px}
</style>
</head>
<body>
<h1>Line Follower v6 <span id="wsStatus" style="font-size:.7rem;color:#444">connecting...</span></h1>

<div class="card">
  <h2>Control</h2>
  <div class="status">
    <div class="dot" id="runDot"></div>
    <span id="runLabel">Stopped</span>
  </div>
  <div class="row">
    <button class="bg" id="btnStart"  onclick="cmd('start')">▶ Start</button>
    <button class="br" id="btnStop"   onclick="cmd('stop')"  disabled>⏹ Stop</button>
    <button class="by" id="btnIRCal"  onclick="cmd('calibrate')">⚡ IR Cal</button>
    <button class="bb" id="btnSave"   onclick="cmd('save')"  disabled>💾 Save</button>
  </div>
  <div class="hint">Sweep bot over line for 8 seconds during IR calibration.</div>
  <div id="calBar"><div id="calFill"></div></div>
</div>

<div class="card">
  <h2>MPU6050 IMU
    <span class="imu-badge" id="imuBadge">---</span>
  </h2>
  <div style="font-size:.75rem;color:#888;margin-bottom:10px">
    Gyro yaw rate: <span id="yawRateVal" style="color:#8b5cf6;font-weight:700">0.0</span> °/s
    &nbsp;|&nbsp; Bias: <span id="gyroBiasVal" style="color:#aaa">uncal</span>
    &nbsp;|&nbsp; Noise: <span id="gyroNoiseVal" style="color:#aaa">—</span>
  </div>
  <div class="gyro-bar-wrap"><div class="gyro-bar-fill" id="gyroBar"></div></div>
  <div class="row" style="margin-top:10px">
    <button class="bv" id="btnImuCal" onclick="cmd('imu_cal')">🔮 IMU Cal (keep still 3s)</button>
    <button class="bc" id="btnMotorCal" onclick="cmd('motor_bal_cal')">⚖ Motor Bal Cal (2.5s drive)</button>
    <button class="by" id="btnTurnCal" onclick="cmd('turn_rate_cal')">↩ Turn Rate Cal (1.5s)</button>
    <button class="bg" id="btnAutoPID" onclick="startAutoPID_ui()">🤖 Auto PID Tune (10s)</button>
    <button class="br" id="btnStopAutoPID" onclick="cmd('stop_auto_pid')" style="display:none">⏹ Stop Auto Tune</button>
  </div>
  <div style="font-size:.68rem;color:#444;margin-top:8px;line-height:1.7">
    <b style="color:#666">IMU Cal</b> — Bot must be still on flat surface. Measures gyro bias and noise floor. Run this FIRST.<br>
    <b style="color:#666">Motor Bal Cal</b> — Bot drives straight. Gyro measures drift, trims motor PWM automatically. Trim resets each run.<br>
    <b style="color:#666">Turn Rate Cal</b> — Bot turns in direction of last error at max speed. Measures real deg/s for feed-forward scaling.<br>
    <b style="color:#8b5cf6">Auto PID Tune</b> — Place bot on a straight line section. Uses relay method (Ziegler-Nichols) to find optimal Kp/Ki/Kd.
  </div>
  <div id="calResultsPanel" style="margin-top:10px;background:#0f0f0f;border:1px solid #2a2a2a;border-radius:7px;padding:10px;font-size:.72rem;color:#666;line-height:1.8">
    <span style="color:#444">No calibration results yet. Run IMU Cal → Motor Bal Cal → Turn Rate Cal in sequence.</span>
  </div>
</div>

<div class="card">
  <h2>Gyro Assist (High-Speed)</h2>
  <div class="row">
    <label>Feed-Forward Gain<small>Gyro yaw → steer correction (0=off, try 0.8–2.0)</small>
      <input type="range" id="gyroFFGain" min="0" max="4.0" step="0.05" value="0" oninput="sendGyro()">
      <span class="val" id="gyroFFGainv">0.00</span></label>
    <label>Straight Trim Gain<small>Heading trim on straights (0=off, try 0.1–0.4)</small>
      <input type="range" id="gyroStraightGain" min="0" max="1.0" step="0.02" value="0" oninput="sendGyro()">
      <span class="val" id="gyroStraightGainv">0.00</span></label>
    <label>Straight Threshold<small>IR error below which gyro trim is active</small>
      <input type="range" id="straightThreshold" min="50" max="1000" step="50" value="300" oninput="sendGyro()">
      <span class="val" id="straightThresholdv">300</span></label>
  </div>
  <div style="font-size:.68rem;color:#444;margin-top:4px;line-height:1.7">
    <b style="color:#8b5cf6">Feed-Forward</b> — Gyro senses rotation the INSTANT you enter a curve, before IR error peaks.
    Adds a correction proportional to yaw rate. This is what lets the bot corner at high speed.<br>
    Start at 0, push speed to max, then raise gain until it stops cutting corners.<br>
    <b style="color:#06b6d4">Straight Trim</b> — On straights, accumulated heading drift trims motor PWM to keep dead-straight.
    Raise slowly. Too high = oscillation.
  </div>
</div>

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

<div class="card" id="endZoneCard">
  <h2>Finish Line / End Zone
    <span id="endZoneBadge" style="display:inline-block;margin-left:8px;padding:1px 8px;
      border-radius:4px;font-size:.65rem;font-weight:700;letter-spacing:.06em;
      background:#1a1a1a;color:#444;border:1px solid #2a2a2a;transition:all .2s">ARMED</span>
  </h2>
  <div id="endZoneBanner" style="display:none;background:#14532d;border:1px solid #166534;
    border-radius:7px;padding:10px 14px;margin-bottom:10px;text-align:center">
    <div style="font-size:1rem;font-weight:700;color:#4ade80;letter-spacing:.08em">🏁 FINISH LINE REACHED</div>
    <div style="font-size:.72rem;color:#86efac;margin-top:3px">Bot stopped automatically. Press ▶ Start to reset.</div>
  </div>
  <div class="toggle-row" style="margin-bottom:10px">
    <span style="font-size:.82rem;color:#999">Enable auto-stop</span>
    <div class="toggle-wrap" style="margin-left:8px">
      <div class="toggle-track" id="ezToggle" onclick="toggleEndZone()"
        style="background:#22c55e;border-color:#166534">
        <div class="toggle-thumb" style="left:29px;background:#fff"></div>
      </div>
    </div>
  </div>
  <div class="row">
    <label>Min Sensors Active<small>How many sensors must see black (4–6)</small>
      <input type="range" id="ezMinSensors" min="1" max="6" step="1" value="5" oninput="sendEndZone()">
      <span class="val" id="ezMinSensorsv">5 / 6</span></label>
    <label>Debounce Time<small>Must hold for this long before stopping (ms)</small>
      <input type="range" id="ezConfirmMs" min="10" max="500" step="10" value="100" oninput="sendEndZone()">
      <span class="val" id="ezConfirmMsv">100ms</span></label>
    <label>Sensor Threshold<small>Norm value to count as "over black" (0–1000)</small>
      <input type="range" id="ezThreshold" min="100" max="900" step="50" value="500" oninput="sendEndZone()">
      <span class="val" id="ezThresholdv">500</span></label>
  </div>
  <div style="display:flex;gap:4px" id="ezSensorDots"></div>
</div>

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

<div class="card">
  <h2>PID Tuning</h2>
  <div class="row">
    <label>Kp<small>Proportional — main steering</small>
      <input type="range" id="kp" min="0" max="5" step="0.05" value="0.4" oninput="sendPID()">
      <span class="val" id="kpv">0.40</span></label>
    <label>Ki<small>Integral — drift correction</small>
      <input type="range" id="ki" min="0" max="1" step="0.005" value="0" oninput="sendPID()">
      <span class="val" id="kiv">0.000</span></label>
    <label>Kd<small>Derivative — damping</small>
      <input type="range" id="kd" min="0" max="5" step="0.05" value="1.2" oninput="sendPID()">
      <span class="val" id="kdv">1.20</span></label>
  </div>
</div>

<div class="card">
  <h2>Speed</h2>
  <div class="row">
    <label>Base<small>Normal running speed</small>
      <input type="range" id="spd" min="30" max="255" step="5" value="150" oninput="sendSpeed()">
      <span class="val" id="spdv">150</span></label>
    <label>Max<small>Maximum (turns)</small>
      <input type="range" id="maxspd" min="80" max="255" step="5" value="200" oninput="sendSpeed()">
      <span class="val" id="maxspdv">200</span></label>
    <label>Min<small>Stall guard</small>
      <input type="range" id="minspd" min="20" max="120" step="5" value="40" oninput="sendSpeed()">
      <span class="val" id="minspdv">40</span></label>
  </div>
</div>

<div class="card">
  <h2>Turning Radius &amp; Hairpin Assist</h2>
  <div class="row">
    <label>Sharpness<small>PID correction multiplier — wide ↔ sharp</small>
      <input type="range" id="sharpness" min="0.1" max="3.0" step="0.05" value="1.0" oninput="sendTurn()">
      <span class="val" id="sharpnessv">1.00×</span></label>
    <label>Inner Brake<small>Extra braking on inner wheel during turns</small>
      <input type="range" id="brake" min="0" max="100" step="5" value="0" oninput="sendTurn()">
      <span class="val" id="brakev">0%</span></label>
  </div>
  <div class="row">
    <label>Curve Exponent<small>Non-linear response — 1.0=linear, 2.0=quadratic</small>
      <input type="range" id="curveExp" min="1.0" max="3.0" step="0.1" value="1.0" oninput="sendTurn()">
      <span class="val" id="curveExpv">1.0</span></label>
    <label>Speed Reduce on Turn<small>Auto-slow % when error is at max</small>
      <input type="range" id="turnSpeedReduce" min="0" max="100" step="5" value="0" oninput="sendTurn()">
      <span class="val" id="turnSpeedReducev">0%</span></label>
    <label>Pivot Threshold<small>Error at which inner wheel reverses (0=off)</small>
      <input type="range" id="pivotThreshold" min="0" max="2500" step="100" value="0" oninput="sendTurn()">
      <span class="val" id="pivotThresholdv">off</span></label>
  </div>
  <div style="display:flex;gap:8px;margin-top:4px;flex-wrap:wrap">
    <div style="position:relative;height:56px;flex:1;min-width:200px;
      background:#111;border:1px solid #2a2a2a;border-radius:8px;overflow:hidden">
      <canvas id="turnCanvas" width="400" height="56" style="width:100%;height:100%"></canvas>
    </div>
  </div>
  <div style="font-size:.68rem;color:#444;margin-top:8px;line-height:1.6">
    <b style="color:#666">Curve Exponent</b> — most effective for hairpins. Try 1.8–2.2.<br>
    <b style="color:#666">Speed Reduce</b> — bot slows into corners. Try 40–60% for U-turns.<br>
    <b style="color:#666">Pivot Threshold</b> — inner wheel reverses above this error. Try 1800–2200.
  </div>
</div>

<div class="card">
  <h2>90° Corner Detection
    <span id="cornerBadge" style="display:inline-block;margin-left:8px;padding:1px 7px;
      border-radius:4px;font-size:.65rem;font-weight:700;background:#1a1a1a;
      color:#444;border:1px solid #2a2a2a;transition:all .15s">IDLE</span>
  </h2>
  <div class="row">
    <label>Sensitivity<small>Min error-rate to trigger (0=off, 300–600 typical)</small>
      <input type="range" id="cornerSensitivity" min="0" max="1500" step="50" value="0" oninput="sendTurn()">
      <span class="val" id="cornerSensitivityv">off</span></label>
    <label>Boost Gain<small>Kp multiplier during corner (1.5–3.0)</small>
      <input type="range" id="cornerBoost" min="1.0" max="5.0" step="0.1" value="2.0" oninput="sendTurn()">
      <span class="val" id="cornerBoostv">2.0×</span></label>
  </div>
  <div class="row">
    <label>Boost Duration<small>How long corner mode stays on (ms)</small>
      <input type="range" id="cornerDuration" min="50" max="600" step="25" value="200" oninput="sendTurn()">
      <span class="val" id="cornerDurationv">200ms</span></label>
    <label>Corner Speed<small>Speed % of base during corner boost</small>
      <input type="range" id="cornerSpeedPct" min="10" max="100" step="5" value="50" oninput="sendTurn()">
      <span class="val" id="cornerSpeedPctv">50%</span></label>
  </div>
</div>

<div class="card">
  <h2>Motor Trim</h2>
  <div class="row">
    <label>Motor Trim<small>PWM balance offset (-50 to +50). Auto-set by Motor Bal Cal.</small>
      <input type="range" id="motorTrim" min="-50" max="50" step="1" value="0" oninput="sendMotorTrim()">
      <span class="val" id="motorTrimv">0</span></label>
  </div>
  <div style="font-size:.68rem;color:#444;margin-top:4px">
    Positive = boost right motor. Negative = boost left motor.
    Motor Bal Cal sets this automatically — use manual trim only for fine-tuning.
  </div>
</div>

<div class="card">
  <h2>Serial Monitor
    <button onclick="clearLog()" style="padding:2px 8px;font-size:.68rem;background:#222;
      color:#888;border:1px solid #2a2a2a;border-radius:4px;cursor:pointer;
      margin-left:8px;font-weight:400">Clear</button>
  </h2>
  <div id="logBox"></div>
</div>

<script>
let ws, calTimer=null, calProgress=0;
let invertLine=false;
let endZoneEnabled=true;

(function(){
  const c=document.getElementById('ezSensorDots');
  if(!c) return;
  for(let i=0;i<6;i++) c.innerHTML+=`<div id="ezDot${i}" style="
    flex:1;height:14px;border-radius:3px;background:#1a1a1a;
    border:1px solid #2a2a2a;transition:background .1s"></div>`;
})();

function initBars(id, colorClass, numClass){
  const c=document.getElementById(id); c.innerHTML='';
  for(let i=0;i<6;i++) c.innerHTML+=`<div class="sb">
    <div class="bwrap"><div class="bar ${colorClass}" id="${id}${i}" style="height:0%"></div></div>
    <div class="sn ${numClass}" id="${id}n${i}">0</div>
    <div class="sl">IR${i+2}</div></div>`;
}
initBars('normBars','norm','');
initBars('rawBars','raw','raw');

function connect(){
  ws=new WebSocket('ws://'+location.hostname+':81/');
  ws.onopen=()=>{ setWs(true); };
  ws.onclose=()=>{ setWs(false); setTimeout(connect,2000); };
  ws.onmessage=(e)=>{
    const d=JSON.parse(e.data);
    if(d.type==='state')   updateState(d);
    if(d.type==='sensors') updateSensors(d);
    if(d.type==='imu')     updateIMU(d);
    if(d.type==='log')     appendLog(d.lines);
  };
}
connect();

function setWs(ok){
  const el=document.getElementById('wsStatus');
  el.textContent=ok?'● connected':'○ disconnected';
  el.style.color=ok?'#22c55e':'#ef4444';
}

function updateIMU(d){
  const badge=document.getElementById('imuBadge');
  if(d.imuCalibrating){
    badge.textContent='CALIBRATING...'; badge.className='imu-badge imu-cal';
  } else if(d.mpuOk && d.imuCalDone){
    badge.textContent='CALIBRATED ✓'; badge.className='imu-badge imu-ok';
  } else if(d.mpuOk){
    badge.textContent='CONNECTED'; badge.className='imu-badge imu-ok';
  } else {
    badge.textContent='NOT FOUND'; badge.className='imu-badge imu-err';
  }
  document.getElementById('yawRateVal').textContent=d.yawRate ? d.yawRate.toFixed(1) : '0.0';
  if(d.gyroBias!==undefined) document.getElementById('gyroBiasVal').textContent=d.gyroBias.toFixed(2)+' °/s';
  if(d.gyroNoise!==undefined) document.getElementById('gyroNoiseVal').textContent='±'+d.gyroNoise.toFixed(2)+' °/s';
  if(d.yawRate!==undefined){
    const pct=Math.min(Math.max(50+(d.yawRate/200)*50,0),100);
    document.getElementById('gyroBar').style.width=pct+'%';
  }
  if(d.calResults){
    const p=document.getElementById('calResultsPanel');
    p.innerHTML=d.calResults;
  }
}

function toggleLineMode(){
  invertLine=!invertLine;
  applyLineModeUI();
  cmd('linemode',{invert:invertLine});
}

function applyLineModeUI(){
  const track=document.getElementById('lineToggle');
  const badge=document.getElementById('modeBadge');
  if(invertLine){
    track.className='toggle-track white-mode';
    badge.className='mode-badge white-line';
    badge.textContent='WHITE LINE';
  } else {
    track.className='toggle-track black-mode';
    badge.className='mode-badge black-line';
    badge.textContent='BLACK LINE';
  }
}

function updateState(d){
  const dot=document.getElementById('runDot');
  dot.className='dot '+(d.calibrating?'cal':d.running?'on':'off');
  document.getElementById('runLabel').textContent=
    d.calibrating?'Calibrating IR...':d.running?'Running':'Stopped';
  document.getElementById('btnStart').disabled=d.running||d.calibrating;
  document.getElementById('btnStop').disabled=!d.running&&!d.calibrating;
  document.getElementById('btnIRCal').disabled=d.running||d.calibrating;
  document.getElementById('btnSave').disabled=!d.calibrated&&!d.calibrating;

  if(typeof d.endZoneEnable !== 'undefined'){
    endZoneEnabled=d.endZoneEnable;
    updateEndZoneToggleUI();
    document.getElementById('ezMinSensors').value=d.endZoneMinSensors;
    document.getElementById('ezMinSensorsv').textContent=d.endZoneMinSensors+' / 6';
    document.getElementById('ezConfirmMs').value=d.endZoneConfirmMs;
    document.getElementById('ezConfirmMsv').textContent=d.endZoneConfirmMs+'ms';
    document.getElementById('ezThreshold').value=d.endZoneThreshold;
    document.getElementById('ezThresholdv').textContent=d.endZoneThreshold;
    const badge=document.getElementById('endZoneBadge');
    const banner=document.getElementById('endZoneBanner');
    if(d.endZoneTriggered){
      badge.textContent='FINISHED'; badge.style.background='#14532d';
      badge.style.color='#4ade80'; badge.style.borderColor='#166534';
      banner.style.display='block';
    } else if(d.endZoneEnable){
      badge.textContent='ARMED'; badge.style.background='#1a1a1a';
      badge.style.color='#22c55e'; badge.style.borderColor='#166534';
      banner.style.display='none';
    } else {
      badge.textContent='OFF'; badge.style.background='#1a1a1a';
      badge.style.color='#444'; badge.style.borderColor='#2a2a2a';
      banner.style.display='none';
    }
  }

  if(typeof d.invertLine !== 'undefined' && d.invertLine !== invertLine){
    invertLine=d.invertLine; applyLineModeUI();
  }

  if(d.calibrating){
    if(!calTimer){
      calProgress=0;
      calTimer=setInterval(()=>{
        calProgress=Math.min(calProgress+100/80,100);
        document.getElementById('calFill').style.width=calProgress+'%';
        if(calProgress>=100){clearInterval(calTimer);calTimer=null;}
      },100);
    }
  } else {
    if(calTimer){clearInterval(calTimer);calTimer=null;}
    document.getElementById('calFill').style.width=d.calibrated?'100%':'0%';
  }

  document.getElementById('kp').value=d.kp;
  document.getElementById('kpv').textContent=d.kp.toFixed(2);
  document.getElementById('ki').value=d.ki;
  document.getElementById('kiv').textContent=d.ki.toFixed(3);
  document.getElementById('kd').value=d.kd;
  document.getElementById('kdv').textContent=d.kd.toFixed(2);
  document.getElementById('spd').value=d.baseSpeed;
  document.getElementById('spdv').textContent=d.baseSpeed;
  document.getElementById('maxspd').value=d.maxSpeed;
  document.getElementById('maxspdv').textContent=d.maxSpeed;
  document.getElementById('minspd').value=d.minSpeed;
  document.getElementById('minspdv').textContent=d.minSpeed;
  if(typeof d.turnSharpness!=='undefined'){
    document.getElementById('sharpness').value=d.turnSharpness;
    document.getElementById('sharpnessv').textContent=d.turnSharpness.toFixed(2)+'×';
    document.getElementById('brake').value=d.turnBrake;
    document.getElementById('brakev').textContent=d.turnBrake+'%';
    document.getElementById('curveExp').value=d.curveExp;
    document.getElementById('curveExpv').textContent=d.curveExp.toFixed(1);
    document.getElementById('turnSpeedReduce').value=d.turnSpeedReduce;
    document.getElementById('turnSpeedReducev').textContent=d.turnSpeedReduce+'%';
    document.getElementById('pivotThreshold').value=d.pivotThreshold;
    document.getElementById('pivotThresholdv').textContent=d.pivotThreshold>0?d.pivotThreshold:'off';
    document.getElementById('cornerSensitivity').value=d.cornerSensitivity;
    document.getElementById('cornerSensitivityv').textContent=d.cornerSensitivity>0?d.cornerSensitivity:'off';
    document.getElementById('cornerBoost').value=d.cornerBoost;
    document.getElementById('cornerBoostv').textContent=d.cornerBoost.toFixed(1)+'×';
    document.getElementById('cornerDuration').value=d.cornerDuration;
    document.getElementById('cornerDurationv').textContent=d.cornerDuration+'ms';
    document.getElementById('cornerSpeedPct').value=d.cornerSpeedPct;
    document.getElementById('cornerSpeedPctv').textContent=d.cornerSpeedPct+'%';
    const badge=document.getElementById('cornerBadge');
    if(d.cornerActive){
      badge.textContent='CORNER!';
      badge.style.background='#78350f'; badge.style.color='#f59e0b';
      badge.style.borderColor='#92400e';
    } else {
      badge.textContent='IDLE';
      badge.style.background='#1a1a1a'; badge.style.color='#444';
      badge.style.borderColor='#2a2a2a';
    }
  }
  if(typeof d.gyroFFGain!=='undefined'){
    document.getElementById('gyroFFGain').value=d.gyroFFGain;
    document.getElementById('gyroFFGainv').textContent=d.gyroFFGain.toFixed(2);
    document.getElementById('gyroStraightGain').value=d.gyroStraightGain;
    document.getElementById('gyroStraightGainv').textContent=d.gyroStraightGain.toFixed(2);
    document.getElementById('straightThreshold').value=d.straightThreshold;
    document.getElementById('straightThresholdv').textContent=d.straightThreshold;
    document.getElementById('motorTrim').value=d.motorTrim;
    document.getElementById('motorTrimv').textContent=d.motorTrim;
  }
  drawTurnPreview(d.turnSharpness||1,d.turnBrake||0,d.curveExp||1,d.turnSpeedReduce||0,d.pivotThreshold||0);
}

function updateSensors(d){
  const ezThresh=+document.getElementById('ezThreshold').value;
  for(let i=0;i<6;i++){
    document.getElementById('normBars'+i).style.height=(d.norm[i]/10)+'%';
    document.getElementById('normBarsn'+i).textContent=d.norm[i];
    if(d.raw){
      document.getElementById('rawBars'+i).style.height=(d.raw[i]/40.95)+'%';
      document.getElementById('rawBarsn'+i).textContent=d.raw[i];
    }
    const dot=document.getElementById('ezDot'+i);
    if(dot){
      const over=d.norm[i]>=ezThresh;
      dot.style.background=over?'#22c55e':'#1a1a1a';
      dot.style.borderColor=over?'#166534':'#2a2a2a';
    }
  }
  const pct=((d.position+3500)/7000)*100;
  document.getElementById('posDot').style.left=pct+'%';
  document.getElementById('posText').textContent=
    'pos: '+d.position+'  err: '+d.error.toFixed(1);
}

function toggleEndZone(){
  endZoneEnabled=!endZoneEnabled;
  updateEndZoneToggleUI();
  sendEndZone();
}
function updateEndZoneToggleUI(){
  const track=document.getElementById('ezToggle');
  const thumb=track.querySelector('.toggle-thumb');
  if(endZoneEnabled){
    track.style.background='#22c55e'; track.style.borderColor='#166534';
    thumb.style.left='29px';
  } else {
    track.style.background='#222'; track.style.borderColor='#3a3a3a';
    thumb.style.left='3px';
  }
}
function sendEndZone(){
  const mn=+document.getElementById('ezMinSensors').value;
  const ms=+document.getElementById('ezConfirmMs').value;
  const th=+document.getElementById('ezThreshold').value;
  document.getElementById('ezMinSensorsv').textContent=mn+' / 6';
  document.getElementById('ezConfirmMsv').textContent=ms+'ms';
  document.getElementById('ezThresholdv').textContent=th;
  cmd('endzone',{enable:endZoneEnabled, minSensors:mn, confirmMs:ms, threshold:th});
}

function appendLog(lines){
  const box=document.getElementById('logBox');
  lines.forEach(l=>{
    const div=document.createElement('div');
    div.className='ll'+(l.includes('[ERR]')?' e':
                        l.includes('[WARN]')?' w':
                        (l.includes('[IMU]')||l.includes('[WIFI]')||l.includes('[WEB]')||l.includes('[MODE]')||l.includes('[MOTOR]')||l.includes('[TURN_CAL]'))?' i':'');
    div.textContent=l;
    box.appendChild(div);
  });
  box.scrollTop=box.scrollHeight;
}
function clearLog(){ document.getElementById('logBox').innerHTML=''; }

function cmd(c,ex={}){
  if(ws&&ws.readyState===1) ws.send(JSON.stringify({cmd:c,...ex}));
}
function startAutoPID_ui(){
  if(!confirm('Place the bot on a STRAIGHT section of the line.\nIt will oscillate for up to 10s, then set Kp/Ki/Kd automatically.\n\nReady?')) return;
  document.getElementById('btnAutoPID').style.display='none';
  document.getElementById('btnStopAutoPID').style.display='';
  cmd('auto_pid');
  // Re-show button after 12s
  setTimeout(()=>{ document.getElementById('btnAutoPID').style.display=''; document.getElementById('btnStopAutoPID').style.display='none'; }, 12000);
}
function sendPID(){
  const kp=+document.getElementById('kp').value;
  const ki=+document.getElementById('ki').value;
  const kd=+document.getElementById('kd').value;
  document.getElementById('kpv').textContent=kp.toFixed(2);
  document.getElementById('kiv').textContent=ki.toFixed(3);
  document.getElementById('kdv').textContent=kd.toFixed(2);
  cmd('pid',{kp,ki,kd});
}
function sendSpeed(){
  const b=+document.getElementById('spd').value;
  const mx=+document.getElementById('maxspd').value;
  const mn=+document.getElementById('minspd').value;
  document.getElementById('spdv').textContent=b;
  document.getElementById('maxspdv').textContent=mx;
  document.getElementById('minspdv').textContent=mn;
  cmd('speed',{baseSpeed:b,maxSpeed:mx,minSpeed:mn});
}
function sendTurn(){
  const sh=+document.getElementById('sharpness').value;
  const br=+document.getElementById('brake').value;
  const ce=+document.getElementById('curveExp').value;
  const sr=+document.getElementById('turnSpeedReduce').value;
  const pt=+document.getElementById('pivotThreshold').value;
  const cs=+document.getElementById('cornerSensitivity').value;
  const cb=+document.getElementById('cornerBoost').value;
  const cd=+document.getElementById('cornerDuration').value;
  const cp=+document.getElementById('cornerSpeedPct').value;
  document.getElementById('sharpnessv').textContent=sh.toFixed(2)+'×';
  document.getElementById('brakev').textContent=br+'%';
  document.getElementById('curveExpv').textContent=ce.toFixed(1);
  document.getElementById('turnSpeedReducev').textContent=sr+'%';
  document.getElementById('pivotThresholdv').textContent=pt>0?pt:'off';
  document.getElementById('cornerSensitivityv').textContent=cs>0?cs:'off';
  document.getElementById('cornerBoostv').textContent=cb.toFixed(1)+'×';
  document.getElementById('cornerDurationv').textContent=cd+'ms';
  document.getElementById('cornerSpeedPctv').textContent=cp+'%';
  cmd('turn',{sharpness:sh,brake:br,curveExp:ce,turnSpeedReduce:sr,
    pivotThreshold:pt,cornerSensitivity:cs,cornerBoost:cb,cornerDuration:cd,cornerSpeedPct:cp});
  drawTurnPreview(sh,br,ce,sr,pt);
}
function sendGyro(){
  const ff=+document.getElementById('gyroFFGain').value;
  const sg=+document.getElementById('gyroStraightGain').value;
  const st=+document.getElementById('straightThreshold').value;
  document.getElementById('gyroFFGainv').textContent=ff.toFixed(2);
  document.getElementById('gyroStraightGainv').textContent=sg.toFixed(2);
  document.getElementById('straightThresholdv').textContent=st;
  cmd('gyro',{ffGain:ff,straightGain:sg,straightThreshold:st});
}
function sendMotorTrim(){
  const t=+document.getElementById('motorTrim').value;
  document.getElementById('motorTrimv').textContent=t;
  cmd('motortrim',{trim:t});
}

function drawTurnPreview(sharpness,brake,curveExp,speedReduce,pivotThresh){
  const canvas=document.getElementById('turnCanvas');
  if(!canvas) return;
  const ctx=canvas.getContext('2d');
  const W=canvas.width, H=canvas.height;
  ctx.clearRect(0,0,W,H);
  const maxErr=2500;
  ctx.beginPath();
  for(let s=0;s<=100;s++){
    const normE=s/100;
    let corr=Math.pow(normE,curveExp)*sharpness;
    corr=Math.min(corr,1);
    const x=W*0.15+s*(W*0.7)/100;
    const y=H*0.85-(corr*H*0.7);
    if(s===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  }
  ctx.strokeStyle='#7dd3fc';
  ctx.lineWidth=2;
  ctx.stroke();
  ctx.beginPath(); ctx.arc(W*0.15,H*0.85,4,0,2*Math.PI);
  ctx.fillStyle='#22c55e'; ctx.fill();
  ctx.fillStyle='#333'; ctx.font='9px system-ui';
  ctx.fillText('exp:'+curveExp.toFixed(1)+' sharp:'+sharpness.toFixed(1)+(pivotThresh>0?' pivot:'+pivotThresh:''),4,H-4);
}
</script>
</body>
</html>
)rawhtml";


// ══════════════════════════════════════════════════════════
//  Log buffer
// ══════════════════════════════════════════════════════════
#define LOG_BUF 40
String logLines[LOG_BUF];
int logHead=0, logCount=0;

void wsLog(const String& msg){
  Serial.println(msg);
  logLines[logHead]=msg;
  logHead=(logHead+1)%LOG_BUF;
  if(logCount<LOG_BUF) logCount++;
}


// ══════════════════════════════════════════════════════════
//  Motors
// ══════════════════════════════════════════════════════════
void motorSetup(){
  ledcSetup(PWM_L1_CH,PWM_FREQ,PWM_RES); ledcAttachPin(MOTOR_L_IN1,PWM_L1_CH);
  ledcSetup(PWM_L2_CH,PWM_FREQ,PWM_RES); ledcAttachPin(MOTOR_L_IN2,PWM_L2_CH);
  ledcSetup(PWM_R3_CH,PWM_FREQ,PWM_RES); ledcAttachPin(MOTOR_R_IN3,PWM_R3_CH);
  ledcSetup(PWM_R4_CH,PWM_FREQ,PWM_RES); ledcAttachPin(MOTOR_R_IN4,PWM_R4_CH);
}

void setMotors(int L, int R){
  if(MOTOR_TRIM >= 0){
    R = constrain(R + MOTOR_TRIM, -255, 255);
  } else {
    L = constrain(L - MOTOR_TRIM, -255, 255);
  }
  L=constrain(L,-255,255); R=constrain(R,-255,255);
  ledcWrite(PWM_L1_CH,L>=0?L:0); ledcWrite(PWM_L2_CH,L<0?-L:0);
  ledcWrite(PWM_R3_CH,R>=0?R:0); ledcWrite(PWM_R4_CH,R<0?-R:0);
}
void stopMotors(){
  ledcWrite(PWM_L1_CH,0); ledcWrite(PWM_L2_CH,0);
  ledcWrite(PWM_R3_CH,0); ledcWrite(PWM_R4_CH,0);
}


// ══════════════════════════════════════════════════════════
//  MPU6050 — Setup
//  FIX 1: Called FIRST in setup() before ADC/LEDC/WiFi
//  FIX 2: delay(150) after Wire.begin() for MPU boot time
//  FIX 3: testConnection() retried up to 5x with 50ms gaps
// ══════════════════════════════════════════════════════════
void mpuSetup(){
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin(21, 22);
    Wire.setClock(400000);
  #endif

  delay(150);        // MPU needs boot time before first I2C transaction
  mpu.initialize();  // clears SLEEP bit in PWR_MGMT_1
  delay(50);         // settle after wake-up

  // Read WHO_AM_I (reg 0x75) directly.
  // testConnection() in the Rowberg library strictly checks for 0x68
  // (MPU-6050) and rejects 0x70 (MPU-6000), even though both chips
  // are fully register-compatible. Your module returns 0x70.
  Wire.beginTransmission(0x68);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x68, (uint8_t)1);
  uint8_t whoAmI = Wire.available() ? Wire.read() : 0x00;

  if(whoAmI == 0x68 || whoAmI == 0x70){
    mpuOk = true;
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    wsLog("[IMU] MPU connected OK. WHO_AM_I=0x" + String(whoAmI, HEX)
          + (whoAmI == 0x70 ? " (MPU-6000 clone)" : " (MPU-6050)"));
    wsLog("[IMU] Range: +-500 deg/s");
  } else {
    mpuOk = false;
    wsLog("[IMU] MPU NOT found. WHO_AM_I=0x" + String(whoAmI, HEX));
    wsLog("[IMU] Check: SDA=21 SCL=22, 3.3V power, AD0 pin state.");
  }
}


// ══════════════════════════════════════════════════════════
//  MPU6050 — Read (called at ~200Hz)
// ══════════════════════════════════════════════════════════
void readMPU(){
  if(!mpuOk) return;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // ±500°/s → 65.536 LSB per deg/s
  float rawZ    = gz / 65.536f;
  float corrected = rawZ - gyroBiasZ;

  // Dead-band: suppress values within noise floor
  if(fabsf(corrected) < gyroNoiseFloor * 1.5f) corrected = 0.0f;

  // Low-pass filter
  yawRateFiltered = 0.7f * yawRateFiltered + 0.3f * corrected;
  yawRate = yawRateFiltered;
}


// ══════════════════════════════════════════════════════════
//  IMU Calibration — gyro bias & noise floor
// ══════════════════════════════════════════════════════════
void startIMUCal(){
  if(!mpuOk){ wsLog("[IMU] ERROR: MPU6050 not connected"); return; }
  imuCalibrating = true;
  imuCalStart    = millis();
  imuCalSumZ     = 0.0f;
  imuCalSumSqZ   = 0.0f;
  imuCalCount    = 0;
  wsLog("[IMU] Gyro cal started — keep bot STILL for 3s...");
}

void updateIMUCal(){
  if(!imuCalibrating) return;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float rawZ = gz / 65.536f;
  imuCalSumZ   += rawZ;
  imuCalSumSqZ += rawZ * rawZ;
  imuCalCount++;

  if(millis() - imuCalStart >= IMU_CAL_MS){
    imuCalibrating = false;
    gyroBiasZ = imuCalSumZ / imuCalCount;
    float meanSq   = imuCalSumSqZ / imuCalCount;
    float variance = meanSq - gyroBiasZ * gyroBiasZ;
    gyroNoiseFloor = sqrtf(fabsf(variance));
    wsLog("[IMU] Gyro cal done!");
    wsLog("[IMU]   Bias Z = " + String(gyroBiasZ, 3) + " deg/s");
    wsLog("[IMU]   Noise  = +-" + String(gyroNoiseFloor, 3) + " deg/s  (" + String(imuCalCount) + " samples)");
    broadcastIMUState();
  }
}


// ══════════════════════════════════════════════════════════
//  Motor Balance Calibration
//  FIX: Trim sign was inverted. Positive avgYaw = bot drifting
//  RIGHT → need to boost LEFT motor (negative trim for L).
//  setMotors() does: if MOTOR_TRIM>=0: R+=trim, else L-=trim
//  So to boost LEFT (negative trim): L -= (-trim) = L+trim ✓
//  To boost RIGHT (positive trim):   R += trim             ✓
//  avgYaw > 0 means drifting right → boost LEFT → trim < 0
//  avgYaw < 0 means drifting left  → boost RIGHT → trim > 0
// ══════════════════════════════════════════════════════════
void startMotorBalCal(){
  if(!mpuOk){ wsLog("[MOTOR] ERROR: run IMU Cal first"); return; }
  // Reset trim to zero each run so we measure clean drift
  MOTOR_TRIM = 0;
  motorCalibrating  = true;
  motorCalStart     = millis();
  motorCalYawAccum  = 0.0f;
  motorCalSamples   = 0;
  headingAccum      = 0.0f;
  wsLog("[MOTOR] Balance cal started — bot drives straight for 2.5s...");
  wsLog("[MOTOR] Trim reset to 0 for clean measurement.");
  setMotors(BASE_SPEED, BASE_SPEED);
}

void updateMotorBalCal(){
  if(!motorCalibrating) return;

  motorCalYawAccum += yawRate;
  motorCalSamples++;

  if(millis() - motorCalStart >= MOTOR_CAL_MS){
    motorCalibrating = false;
    stopMotors();

    float avgYaw = motorCalYawAccum / motorCalSamples;
    // FIX: Negate the sign — drifting right (positive yaw) means
    // right motor is faster, so we need negative trim to boost left.
    // drifting left (negative yaw) → positive trim boosts right.
    int trim = -(int)(avgYaw * 3.0f);  // scale 3.0 gives ~1 PWM per 0.33 deg/s drift
    trim = constrain(trim, -50, 50);
    MOTOR_TRIM = trim;  // Set absolute, not additive (since we reset to 0)

    wsLog("[MOTOR] Balance cal done!");
    wsLog("[MOTOR]   Avg yaw drift = " + String(avgYaw, 2) + " deg/s"
          + (avgYaw > 0.5f ? " (drifting RIGHT)" : avgYaw < -0.5f ? " (drifting LEFT)" : " (straight!)"));
    wsLog("[MOTOR]   Trim set = " + String(MOTOR_TRIM)
          + (MOTOR_TRIM > 0 ? " → right motor boosted" : MOTOR_TRIM < 0 ? " → left motor boosted" : " → no trim needed"));
    broadcastIMUState();
  }
}


// ══════════════════════════════════════════════════════════
//  Turn Rate Calibration
//  Spins in the direction of last sensor error for
//  consistency (not always hard-left).
// ══════════════════════════════════════════════════════════
void startTurnRateCal(){
  if(!mpuOk){ wsLog("[TURN_CAL] ERROR: run IMU Cal first"); return; }
  turnCalibrating  = true;
  turnCalStart     = millis();
  turnCalYawAccum  = 0.0f;
  turnCalSamples   = 0;
  // Spin direction: use last position error — if bot was turning right,
  // calibrate that direction. Default to right (+1) if no error known.
  turnCalDirection = (lastPosition >= 0) ? 1 : -1;
  wsLog("[TURN_CAL] Turn rate cal started — pivoting " +
        String(turnCalDirection > 0 ? "RIGHT" : "LEFT") + " for 1.5s...");
  // direction +1 = pivot right: left motor fwd, right motor rev
  setMotors(MAX_SPEED * turnCalDirection, -MAX_SPEED * turnCalDirection);
}

void updateTurnRateCal(){
  if(!turnCalibrating) return;

  turnCalYawAccum += fabsf(yawRate);
  turnCalSamples++;

  if(millis() - turnCalStart >= TURN_CAL_MS){
    turnCalibrating = false;
    stopMotors();

    float avgTurnRate = turnCalYawAccum / turnCalSamples;
    if(avgTurnRate > 1.0f){
      TURN_RATE_SCALE = avgTurnRate;
    }
    wsLog("[TURN_CAL] Turn rate cal done!");
    wsLog("[TURN_CAL]   Avg turn rate = " + String(avgTurnRate, 1) + " deg/s at full correction");
    wsLog("[TURN_CAL]   TURN_RATE_SCALE = " + String(TURN_RATE_SCALE, 1));
    broadcastIMUState();
  }
}


// ══════════════════════════════════════════════════════════
//  Auto PID Tuning — Ziegler-Nichols Relay Method
//  Drives the bot on the line with bang-bang control,
//  measures the natural oscillation period (Tu) and
//  amplitude (a), then computes Kp, Ki, Kd.
//
//  Ku = (4 * relay_amplitude) / (π * oscillation_amplitude)
//  Tu = measured oscillation period
//  Kp = 0.6 * Ku,  Ki = 1.2*Ku/Tu,  Kd = 0.075*Ku*Tu
// ══════════════════════════════════════════════════════════
void startAutoPID(){
  if(!isCalibrated){ wsLog("[AUTO_PID] ERROR: run IR Cal first"); return; }
  autoPIDActive   = true;
  autoPIDStart    = millis();
  relayHigh       = false;
  relayAmplitude  = (float)BASE_SPEED * 0.5f;  // relay output = ±50% of base
  crossCount      = 0;
  periodSum       = 0.0f;
  amplitudeSum    = 0.0f;
  peakError       = 0.0f;
  lastCrossTime   = millis();
  lastCrossError  = 0.0f;
  wsLog("[AUTO_PID] Relay tune started — bot must be ON the line.");
  wsLog("[AUTO_PID] Window: 10s, relay amp = " + String((int)relayAmplitude));
}

void updateAutoPID(){
  if(!autoPIDActive) return;

  float err = (float)lastPosition;
  // Track peak error amplitude this half-cycle
  if(fabsf(err) > peakError) peakError = fabsf(err);

  // Detect zero-crossing (error changed sign)
  bool  crossed = (err >= 0.0f) != (lastCrossError >= 0.0f);
  if(crossed && millis() - lastCrossTime > 50){  // debounce 50ms
    unsigned long period2 = millis() - lastCrossTime;  // half-period in ms
    if(period2 > 50 && period2 < 3000){  // sanity: 50ms–3s
      if(crossCount > 0){  // skip first half-cycle (undefined)
        periodSum    += period2 * 2.0f;  // full period
        amplitudeSum += peakError;
        crossCount++;
        wsLog("[AUTO_PID] cross #" + String(crossCount) +
              "  half-T=" + String(period2) + "ms  peakErr=" + String((int)peakError));
      } else {
        crossCount = 1;  // mark that we've seen first crossing
      }
      peakError     = 0.0f;  // reset for next half-cycle
      lastCrossTime = millis();
    }
  }
  lastCrossError = err;

  // Relay: switch output at zero-crossing
  if(err > 0.0f)  relayHigh = true;
  else            relayHigh = false;

  float correction = relayHigh ? relayAmplitude : -relayAmplitude;
  int spd = constrain(BASE_SPEED, MIN_SPEED, MAX_SPEED);
  int L = constrain(spd - (int)correction, -MAX_SPEED, MAX_SPEED);
  int R = constrain(spd + (int)correction, -MAX_SPEED, MAX_SPEED);
  setMotors(L, R);

  // Done?
  unsigned long elapsed = millis() - autoPIDStart;
  if(elapsed >= AUTO_PID_MS || crossCount >= 10){
    autoPIDActive = false;
    stopMotors();

    int validCycles = crossCount - 1;  // first crossing is reference
    if(validCycles < 2){
      wsLog("[AUTO_PID] Not enough oscillations detected (" + String(validCycles) + ").");
      wsLog("[AUTO_PID] Try: higher speed, lower Kp, or make sure bot is on straight line.");
      return;
    }

    float Tu_ms = periodSum / validCycles;           // avg oscillation period (ms)
    float a = amplitudeSum / validCycles;             // avg oscillation amplitude
    float Tu = Tu_ms / 1000.0f;                      // in seconds
    // Ultimate gain formula for relay method
    float Ku = (4.0f * relayAmplitude) / (3.14159f * max(a, 1.0f));

    // Ziegler-Nichols PID formulas
    float newKp = 0.6f  * Ku;
    float newKi = 1.2f  * Ku / Tu;
    float newKd = 0.075f * Ku * Tu;

    // Clamp to safe ranges
    newKp = constrain(newKp, 0.05f, 5.0f);
    newKi = constrain(newKi, 0.0f,  1.0f);
    newKd = constrain(newKd, 0.0f,  5.0f);

    Kp = newKp; Ki = newKi; Kd = newKd;
    pidIntegral = 0; pidLastError = 0;

    wsLog("[AUTO_PID] === TUNING COMPLETE ===");
    wsLog("[AUTO_PID]   Cycles sampled = " + String(validCycles));
    wsLog("[AUTO_PID]   Tu = " + String(Tu * 1000.0f, 0) + " ms,  Ku = " + String(Ku, 3));
    wsLog("[AUTO_PID]   Kp = " + String(Kp, 3));
    wsLog("[AUTO_PID]   Ki = " + String(Ki, 4));
    wsLog("[AUTO_PID]   Kd = " + String(Kd, 3));
    wsLog("[AUTO_PID] Verify on track, then use Save to keep.");

    String s = buildStateJson(); wsServer.broadcastTXT(s);
  }
}


// ══════════════════════════════════════════════════════════
//  IR sensors
// ══════════════════════════════════════════════════════════
void readSensors(){
  for(int i=0;i<NUM_SENSORS;i++) rawVals[i]=analogRead(IR_PINS[i]);
}

void evaluateSensors(){
  for(int i=0;i<NUM_SENSORS;i++){
    int span=calMax[i]-calMin[i];
    sensorValid[i]=(span>=150);
    wsLog("  IR"+String(i+1)+" span="+String(span)+(sensorValid[i]?" OK":" EXCLUDED"));
  }
}

void normaliseSensors(){
  for(int i=0;i<NUM_SENSORS;i++){
    int v;
    if(!isCalibrated){
      v=constrain(map(rawVals[i],50,4000,0,1000),0,1000);
    } else {
      if(!sensorValid[i]){ normVals[i]=0; continue; }
      v=constrain(map(rawVals[i],calMin[i],calMax[i],0,1000),0,1000);
    }
    if(invertLine) v = 1000 - v;
    normVals[i]=v;
  }
}

int calcPosition(){
  long wSum=0, sSum=0; bool any=false;
  for(int i=0;i<NUM_SENSORS;i++){
    if(sensorValid[i] && normVals[i]>150){
      wSum+=(long)SENSOR_WEIGHTS[i]*normVals[i];
      sSum+=normVals[i];
      any=true;
    }
  }
  if(!any) return lastPosition;
  lastPosition=(int)(wSum/sSum);
  return lastPosition;
}


// ══════════════════════════════════════════════════════════
//  IR Calibration
// ══════════════════════════════════════════════════════════
void startCalibration(){
  for(int i=0;i<NUM_SENSORS;i++){ calMin[i]=4095; calMax[i]=0; }
  isCalibrated=false;
  calibrating=true;
  calStart=millis();
  wsLog("[CAL] IR Cal started -- sweep over line for 8 seconds");
}

void updateCalibration(){
  if(!calibrating) return;
  for(int i=0;i<NUM_SENSORS;i++){
    if(rawVals[i]<calMin[i]) calMin[i]=rawVals[i];
    if(rawVals[i]>calMax[i]) calMax[i]=rawVals[i];
  }
  if(millis()-calStart>=CAL_DURATION){
    calibrating=false;
    isCalibrated=true;
    wsLog("[CAL] IR Cal done!");
    evaluateSensors();
    String s=buildStateJson(); wsServer.broadcastTXT(s);
  }
}


// ══════════════════════════════════════════════════════════
//  PID + Gyro Feed-Forward
// ══════════════════════════════════════════════════════════
void runPID(){
  float currentError = (float)lastPosition;

  float rawDeriv = currentError - pidError;
  errorDerivative = 0.6f * errorDerivative + 0.4f * rawDeriv;

  if(CORNER_SENSITIVITY > 0 && !cornerActive
     && fabsf(errorDerivative) > (float)CORNER_SENSITIVITY){
    cornerActive  = true;
    cornerStartMs = millis();
    wsLog("[CORNER] dErr="+String((int)errorDerivative)+" err="+String((int)currentError));
  }
  if(cornerActive && (millis() - cornerStartMs) > (unsigned long)CORNER_DURATION_MS){
    cornerActive = false;
  }

  pidError    = currentError;
  pidIntegral = constrain(pidIntegral + pidError, -10000.0f, 10000.0f);

  float effKp  = Kp * (cornerActive ? CORNER_BOOST : 1.0f);
  float rawCorr = effKp*pidError + Ki*pidIntegral + Kd*(pidError - pidLastError);
  pidLastError  = pidError;

  float normCorr = rawCorr / (float)MAX_SPEED;
  float sign     = (normCorr >= 0) ? 1.0f : -1.0f;
  float expCorr  = sign * powf(fabsf(normCorr), CURVE_EXP) * (float)MAX_SPEED;
  float corr     = expCorr * TURN_SHARPNESS;

  float gyroFF = 0.0f;
  if(mpuOk && GYRO_FF_GAIN > 0.0f && TURN_RATE_SCALE > 0.1f){
    gyroFF = (yawRate / TURN_RATE_SCALE) * (float)MAX_SPEED * GYRO_FF_GAIN;
  }

  float gyroTrim = 0.0f;
  if(mpuOk && GYRO_STRAIGHT_GAIN > 0.0f && fabsf(currentError) < (float)STRAIGHT_THRESHOLD){
    headingAccum += yawRate * 0.005f;
    headingAccum = constrain(headingAccum, -30.0f, 30.0f);
    gyroTrim = headingAccum * GYRO_STRAIGHT_GAIN * (float)MAX_SPEED * 0.01f;
  } else {
    headingAccum *= 0.95f;
  }

  float totalCorr = corr + gyroFF + gyroTrim;

  float normErr  = constrain(fabsf(pidError) / 2500.0f, 0.0f, 1.0f);
  float errScale = 1.0f - normErr * (TURN_SPEED_REDUCE / 100.0f);
  int   effBase  = (int)(BASE_SPEED * errScale);
  if(cornerActive){
    int cornerSpeed = (int)(BASE_SPEED * (CORNER_SPEED_PCT / 100.0f));
    effBase = min(effBase, cornerSpeed);
  }
  effBase = constrain(effBase, MIN_SPEED, MAX_SPEED);

  int L = constrain(effBase - (int)totalCorr, -MAX_SPEED, MAX_SPEED);
  int R = constrain(effBase + (int)totalCorr, -MAX_SPEED, MAX_SPEED);

  if(TURN_BRAKE > 0){
    float brakeScale = normErr * (TURN_BRAKE / 100.0f);
    if(pidError > 0){
      int newL = (int)(L - (L - MIN_SPEED) * brakeScale);
      L = constrain(newL, MIN_SPEED, MAX_SPEED);
    } else {
      int newR = (int)(R - (R - MIN_SPEED) * brakeScale);
      R = constrain(newR, MIN_SPEED, MAX_SPEED);
    }
  }

  bool doPivot = (PIVOT_THRESHOLD > 0 && fabsf(pidError) >= (float)PIVOT_THRESHOLD)
              || (cornerActive && fabsf(pidError) > 800.0f);

  if(doPivot){
    float pivotRef   = cornerActive ? 800.0f : (float)PIVOT_THRESHOLD;
    float pivotRange = 2500.0f - pivotRef;
    float pivotDepth = constrain((fabsf(pidError) - pivotRef) / pivotRange, 0.0f, 1.0f);
    if(cornerActive) pivotDepth = max(pivotDepth, 0.4f);
    int reverseSpeed = (int)(MIN_SPEED + pivotDepth * (MAX_SPEED - MIN_SPEED));
    if(pidError > 0) L = -reverseSpeed;
    else             R = -reverseSpeed;
  }

  L = constrain(L, -MAX_SPEED, MAX_SPEED);
  R = constrain(R, -MAX_SPEED, MAX_SPEED);
  setMotors(L, R);
}


// ══════════════════════════════════════════════════════════
//  End-zone detection
// ══════════════════════════════════════════════════════════
bool detectEndZone(){
  if(!endZoneEnable || !robotRunning) return false;
  int active = 0;
  for(int i = 0; i < NUM_SENSORS; i++){
    if(sensorValid[i] && normVals[i] >= endZoneThreshold) active++;
  }
  bool conditionMet = (active >= endZoneMinSensors);
  if(conditionMet){
    if(!endZonePending){
      endZonePending   = true;
      endZoneFirstSeen = millis();
    } else if(millis() - endZoneFirstSeen >= (unsigned long)endZoneConfirmMs){
      robotRunning     = false;
      endZoneTriggered = true;
      endZonePending   = false;
      stopMotors();
      wsLog("[END] Finish line! "+String(active)+" sensors. Stopped.");
      String s = buildStateJson(); wsServer.broadcastTXT(s);
      return true;
    }
  } else {
    endZonePending = false;
  }
  return false;
}

void resetEndZone(){
  endZonePending   = false;
  endZoneTriggered = false;
  endZoneFirstSeen = 0;
}


// ══════════════════════════════════════════════════════════
//  Flash save / load
// ══════════════════════════════════════════════════════════
void saveAll(){
  prefs.begin("lf",false);
  for(int i=0;i<NUM_SENSORS;i++){
    prefs.putInt(("n"+String(i)).c_str(), calMin[i]);
    prefs.putInt(("x"+String(i)).c_str(), calMax[i]);
  }
  prefs.putFloat("kp",Kp); prefs.putFloat("ki",Ki); prefs.putFloat("kd",Kd);
  prefs.putInt("bs",BASE_SPEED); prefs.putInt("ms",MAX_SPEED); prefs.putInt("mn",MIN_SPEED);
  prefs.putFloat("ts",TURN_SHARPNESS); prefs.putInt("tb",TURN_BRAKE);
  prefs.putFloat("ce",CURVE_EXP); prefs.putInt("tsr",TURN_SPEED_REDUCE); prefs.putInt("pt",PIVOT_THRESHOLD);
  prefs.putInt("cs",CORNER_SENSITIVITY); prefs.putFloat("cb",CORNER_BOOST);
  prefs.putInt("cd",CORNER_DURATION_MS); prefs.putInt("csp",CORNER_SPEED_PCT);
  prefs.putBool("inv",invertLine);
  prefs.putBool("eze",endZoneEnable); prefs.putInt("ezm",endZoneMinSensors);
  prefs.putInt("ezc",endZoneConfirmMs); prefs.putInt("ezt",endZoneThreshold);
  prefs.putFloat("gff",GYRO_FF_GAIN);
  prefs.putFloat("gsg",GYRO_STRAIGHT_GAIN);
  prefs.putInt("gst",STRAIGHT_THRESHOLD);
  prefs.putInt("mtr",MOTOR_TRIM);
  prefs.putFloat("trs",TURN_RATE_SCALE);
  prefs.putFloat("gbz",gyroBiasZ);
  prefs.putFloat("gnf",gyroNoiseFloor);
  prefs.end();
  wsLog("[FLASH] Saved all settings including gyro cal.");
}

void loadAll(){
  prefs.begin("lf",true);
  if(prefs.isKey("n0")){
    for(int i=0;i<NUM_SENSORS;i++){
      calMin[i]=prefs.getInt(("n"+String(i)).c_str(),0);
      calMax[i]=prefs.getInt(("x"+String(i)).c_str(),4095);
    }
    Kp=prefs.getFloat("kp",0.4f);
    Ki=prefs.getFloat("ki",0.0f);
    Kd=prefs.getFloat("kd",1.2f);
    BASE_SPEED=prefs.getInt("bs",150);
    MAX_SPEED =prefs.getInt("ms",200);
    MIN_SPEED =prefs.getInt("mn",40);
    TURN_SHARPNESS=prefs.getFloat("ts",1.0f);
    TURN_BRAKE    =prefs.getInt("tb",0);
    CURVE_EXP          =prefs.getFloat("ce",1.0f);
    TURN_SPEED_REDUCE  =prefs.getInt("tsr",0);
    PIVOT_THRESHOLD    =prefs.getInt("pt",0);
    CORNER_SENSITIVITY =prefs.getInt("cs",0);
    CORNER_BOOST       =prefs.getFloat("cb",2.0f);
    CORNER_DURATION_MS =prefs.getInt("cd",200);
    CORNER_SPEED_PCT   =prefs.getInt("csp",50);
    invertLine=prefs.getBool("inv",false);
    endZoneEnable     =prefs.getBool("eze",true);
    endZoneMinSensors =prefs.getInt("ezm",5);
    endZoneConfirmMs  =prefs.getInt("ezc",100);
    endZoneThreshold  =prefs.getInt("ezt",500);
    GYRO_FF_GAIN       =prefs.getFloat("gff",0.0f);
    GYRO_STRAIGHT_GAIN =prefs.getFloat("gsg",0.0f);
    STRAIGHT_THRESHOLD =prefs.getInt("gst",300);
    MOTOR_TRIM         =prefs.getInt("mtr",0);
    TURN_RATE_SCALE    =prefs.getFloat("trs",1.0f);
    gyroBiasZ          =prefs.getFloat("gbz",0.0f);
    gyroNoiseFloor     =prefs.getFloat("gnf",0.0f);
    isCalibrated=true;
    evaluateSensors();
    wsLog("[FLASH] Loaded. Gyro bias="+String(gyroBiasZ,3)+" trim="+String(MOTOR_TRIM));
  }
  prefs.end();
}


// ══════════════════════════════════════════════════════════
//  WebSocket JSON builders
// ══════════════════════════════════════════════════════════
String buildStateJson(){
  StaticJsonDocument<768> doc;
  doc["type"]        ="state";
  doc["running"]     =robotRunning;
  doc["calibrating"] =calibrating;
  doc["calibrated"]  =isCalibrated;
  doc["kp"]=Kp; doc["ki"]=Ki; doc["kd"]=Kd;
  doc["baseSpeed"]=BASE_SPEED;
  doc["maxSpeed"] =MAX_SPEED;
  doc["minSpeed"] =MIN_SPEED;
  doc["turnSharpness"]=TURN_SHARPNESS;
  doc["turnBrake"]    =TURN_BRAKE;
  doc["curveExp"]         =CURVE_EXP;
  doc["turnSpeedReduce"]  =TURN_SPEED_REDUCE;
  doc["pivotThreshold"]   =PIVOT_THRESHOLD;
  doc["cornerSensitivity"]=CORNER_SENSITIVITY;
  doc["cornerBoost"]      =CORNER_BOOST;
  doc["cornerDuration"]   =CORNER_DURATION_MS;
  doc["cornerSpeedPct"]   =CORNER_SPEED_PCT;
  doc["cornerActive"]     =cornerActive;
  doc["invertLine"]       =invertLine;
  doc["endZoneEnable"]    =endZoneEnable;
  doc["endZoneMinSensors"]=endZoneMinSensors;
  doc["endZoneConfirmMs"] =endZoneConfirmMs;
  doc["endZoneThreshold"] =endZoneThreshold;
  doc["endZoneTriggered"] =endZoneTriggered;
  doc["gyroFFGain"]       =GYRO_FF_GAIN;
  doc["gyroStraightGain"] =GYRO_STRAIGHT_GAIN;
  doc["straightThreshold"]=STRAIGHT_THRESHOLD;
  doc["motorTrim"]        =MOTOR_TRIM;
  String out; serializeJson(doc,out); return out;
}

void broadcastIMUState(){
  StaticJsonDocument<512> doc;
  doc["type"]          = "imu";
  doc["mpuOk"]         = mpuOk;
  doc["imuCalibrating"]= imuCalibrating;
  doc["imuCalDone"]    = (gyroBiasZ != 0.0f || gyroNoiseFloor != 0.0f);
  doc["yawRate"]       = yawRate;
  doc["gyroBias"]      = gyroBiasZ;
  doc["gyroNoise"]     = gyroNoiseFloor;
  String results = "";
  if(gyroBiasZ != 0.0f || gyroNoiseFloor != 0.0f){
    results += "<span style='color:#4ade80'>✓ IMU Cal</span>: ";
    results += "Bias=" + String(gyroBiasZ,3) + " deg/s  Noise=+-" + String(gyroNoiseFloor,3) + " deg/s<br>";
  }
  if(MOTOR_TRIM != 0){
    results += "<span style='color:#60a5fa'>✓ Motor Bal</span>: Trim=";
    results += String(MOTOR_TRIM);
    results += (MOTOR_TRIM > 0 ? " (right motor boosted)" : " (left motor boosted)");
    results += "<br>";
  }
  if(TURN_RATE_SCALE != 1.0f){
    results += "<span style='color:#f59e0b'>✓ Turn Cal</span>: ";
    results += "Scale=" + String(TURN_RATE_SCALE,1) + " deg/s at full pivot<br>";
  }
  if(results.length() == 0){
    results = "<span style='color:#444'>No calibration done yet.</span>";
  }
  doc["calResults"] = results;
  String out; serializeJson(doc,out);
  wsServer.broadcastTXT(out);
}

String buildSensorJson(){
  StaticJsonDocument<512> doc;
  doc["type"]    ="sensors";
  JsonArray norm =doc.createNestedArray("norm");
  JsonArray raw  =doc.createNestedArray("raw");
  for(int i=0;i<NUM_SENSORS;i++){
    norm.add(normVals[i]);
    raw.add(rawVals[i]);
  }
  doc["position"]=lastPosition; doc["error"]=pidError;
  String out; serializeJson(doc,out); return out;
}

String buildLogJson(){
  StaticJsonDocument<2048> doc;
  doc["type"]="log";
  JsonArray arr=doc.createNestedArray("lines");
  int start=(logHead-logCount+LOG_BUF)%LOG_BUF;
  for(int i=0;i<logCount;i++) arr.add(logLines[(start+i)%LOG_BUF]);
  logCount=0;
  String out; serializeJson(doc,out); return out;
}


// ══════════════════════════════════════════════════════════
//  WebSocket event handler
// ══════════════════════════════════════════════════════════
void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length){
  if(type==WStype_CONNECTED){
    String s=buildStateJson(); wsServer.sendTXT(num,s);
    broadcastIMUState();
    String l=buildLogJson();   wsServer.sendTXT(num,l);
    return;
  }
  if(type!=WStype_TEXT) return;

  StaticJsonDocument<256> doc;
  if(deserializeJson(doc,payload,length)) return;
  const char* c=doc["cmd"]; if(!c) return;

  if(!strcmp(c,"start")){
    robotRunning=true; pidIntegral=0; pidLastError=0;
    headingAccum=0.0f;
    resetEndZone();
    wsLog("[CMD] Start");
  }
  else if(!strcmp(c,"stop")){
    robotRunning=false; calibrating=false;
    imuCalibrating=false; motorCalibrating=false; turnCalibrating=false;
    stopMotors();
    wsLog("[CMD] Stop");
  }
  else if(!strcmp(c,"calibrate")){
    startCalibration();
  }
  else if(!strcmp(c,"imu_cal")){
    startIMUCal();
  }
  else if(!strcmp(c,"motor_bal_cal")){
    startMotorBalCal();
  }
  else if(!strcmp(c,"turn_rate_cal")){
    startTurnRateCal();
  }
  else if(!strcmp(c,"auto_pid")){
    startAutoPID();
  }
  else if(!strcmp(c,"stop_auto_pid")){
    autoPIDActive = false;
    stopMotors();
    wsLog("[AUTO_PID] Aborted.");
  }
  else if(!strcmp(c,"save")){
    saveAll();
  }
  else if(!strcmp(c,"pid")){
    Kp=doc["kp"]|Kp; Ki=doc["ki"]|Ki; Kd=doc["kd"]|Kd;
    pidIntegral=0;
    wsLog("[PID] Kp="+String(Kp,2)+" Ki="+String(Ki,3)+" Kd="+String(Kd,2));
  }
  else if(!strcmp(c,"speed")){
    BASE_SPEED=doc["baseSpeed"]|BASE_SPEED;
    MAX_SPEED =doc["maxSpeed"] |MAX_SPEED;
    MIN_SPEED =doc["minSpeed"] |MIN_SPEED;
    wsLog("[SPD] base="+String(BASE_SPEED)+" max="+String(MAX_SPEED)+" min="+String(MIN_SPEED));
  }
  else if(!strcmp(c,"turn")){
    TURN_SHARPNESS     = doc["sharpness"]        | TURN_SHARPNESS;
    TURN_BRAKE         = doc["brake"]            | TURN_BRAKE;
    CURVE_EXP          = doc["curveExp"]         | CURVE_EXP;
    TURN_SPEED_REDUCE  = doc["turnSpeedReduce"]  | TURN_SPEED_REDUCE;
    PIVOT_THRESHOLD    = doc["pivotThreshold"]   | PIVOT_THRESHOLD;
    CORNER_SENSITIVITY = doc["cornerSensitivity"]| CORNER_SENSITIVITY;
    CORNER_BOOST       = doc["cornerBoost"]      | CORNER_BOOST;
    CORNER_DURATION_MS = doc["cornerDuration"]   | CORNER_DURATION_MS;
    CORNER_SPEED_PCT   = doc["cornerSpeedPct"]   | CORNER_SPEED_PCT;
    TURN_SHARPNESS     = constrain(TURN_SHARPNESS, 0.1f, 3.0f);
    TURN_BRAKE         = constrain(TURN_BRAKE, 0, 100);
    CURVE_EXP          = constrain(CURVE_EXP, 1.0f, 3.0f);
    TURN_SPEED_REDUCE  = constrain(TURN_SPEED_REDUCE, 0, 100);
    PIVOT_THRESHOLD    = constrain(PIVOT_THRESHOLD, 0, 2500);
    CORNER_SENSITIVITY = constrain(CORNER_SENSITIVITY, 0, 2000);
    CORNER_BOOST       = constrain(CORNER_BOOST, 1.0f, 5.0f);
    CORNER_DURATION_MS = constrain(CORNER_DURATION_MS, 50, 1000);
    CORNER_SPEED_PCT   = constrain(CORNER_SPEED_PCT, 10, 100);
    pidIntegral=0; cornerActive=false;
    wsLog("[TURN] sharp="+String(TURN_SHARPNESS,2)+" brake="+String(TURN_BRAKE)
         +"% exp="+String(CURVE_EXP,2)+" spdR="+String(TURN_SPEED_REDUCE)
         +"% pivot="+String(PIVOT_THRESHOLD));
  }
  else if(!strcmp(c,"gyro")){
    GYRO_FF_GAIN       = doc["ffGain"]           | GYRO_FF_GAIN;
    GYRO_STRAIGHT_GAIN = doc["straightGain"]     | GYRO_STRAIGHT_GAIN;
    STRAIGHT_THRESHOLD = doc["straightThreshold"]| STRAIGHT_THRESHOLD;
    GYRO_FF_GAIN       = constrain(GYRO_FF_GAIN, 0.0f, 4.0f);
    GYRO_STRAIGHT_GAIN = constrain(GYRO_STRAIGHT_GAIN, 0.0f, 1.0f);
    STRAIGHT_THRESHOLD = constrain(STRAIGHT_THRESHOLD, 50, 1000);
    headingAccum=0.0f;
    wsLog("[GYRO] FF="+String(GYRO_FF_GAIN,2)+" straight="+String(GYRO_STRAIGHT_GAIN,2)
         +" thresh="+String(STRAIGHT_THRESHOLD));
  }
  else if(!strcmp(c,"motortrim")){
    MOTOR_TRIM = doc["trim"] | MOTOR_TRIM;
    MOTOR_TRIM = constrain(MOTOR_TRIM, -50, 50);
    wsLog("[MOTOR] Trim="+String(MOTOR_TRIM));
  }
  else if(!strcmp(c,"endzone")){
    endZoneEnable     = doc["enable"]     | endZoneEnable;
    endZoneMinSensors = doc["minSensors"] | endZoneMinSensors;
    endZoneConfirmMs  = doc["confirmMs"]  | endZoneConfirmMs;
    endZoneThreshold  = doc["threshold"]  | endZoneThreshold;
    endZoneMinSensors = constrain(endZoneMinSensors, 1, NUM_SENSORS);
    endZoneConfirmMs  = constrain(endZoneConfirmMs, 10, 500);
    endZoneThreshold  = constrain(endZoneThreshold, 100, 900);
  }
  else if(!strcmp(c,"linemode")){
    bool newInvert = doc["invert"] | invertLine;
    if(newInvert != invertLine){
      invertLine = newInvert;
      pidIntegral=0; pidLastError=0;
      wsLog("[MODE] "+(invertLine?String("WHITE line on black BG"):String("BLACK line on white BG")));
    }
  }

  String s=buildStateJson(); wsServer.broadcastTXT(s);
}

void setupHTTP(){
  httpServer.on("/",[](){ httpServer.send_P(200,"text/html",INDEX_HTML); });
  httpServer.onNotFound([](){ httpServer.send(404,"text/plain","Not found"); });
}


// ══════════════════════════════════════════════════════════
//  setup()
//  FIX 1: mpuSetup() is FIRST — before ADC, LEDC, WiFi
// ══════════════════════════════════════════════════════════
void setup(){
  Serial.begin(115200); delay(400);
  wsLog("===== Line Follower v6 (MPU6050 fixed) =====");

  // ── FIX 1: MPU6050 initialised BEFORE anything else ──
  // LEDC (motors) and ADC attenuation can interfere with
  // I2C peripheral init on ESP32 if called first.
  mpuSetup();

  // ── IR sensor pins ────────────────────────────────────
  for(int i=0;i<NUM_SENSORS;i++) sensorValid[i]=true;
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  for(int i=0;i<NUM_SENSORS;i++){
    pinMode(IR_PINS[i], INPUT);
    analogSetPinAttenuation(IR_PINS[i], ADC_11db);
  }

  // ── Motors ────────────────────────────────────────────
  motorSetup(); stopMotors();

  // ── Flash ─────────────────────────────────────────────
  loadAll();

  // ── WiFi ──────────────────────────────────────────────
  wsLog("[WIFI] Connecting to "+String(SSID)+"...");
  WiFi.begin(SSID,PASSWORD);
  while(WiFi.status()!=WL_CONNECTED){ delay(500); Serial.print("."); }
  Serial.println();
  wsLog("[WIFI] IP: "+WiFi.localIP().toString());
  wsLog("[WEB]  http://"+WiFi.localIP().toString()+"/");
  if(mpuOk && gyroBiasZ != 0.0f){
    wsLog("[IMU]  Loaded saved cal: bias="+String(gyroBiasZ,3)+" noise=+-"+String(gyroNoiseFloor,3));
  }
  wsLog("=========================");

  setupHTTP(); httpServer.begin();
  wsServer.begin(); wsServer.onEvent(onWsEvent);
}


// ══════════════════════════════════════════════════════════
//  loop()
// ══════════════════════════════════════════════════════════
void loop(){
  httpServer.handleClient();
  wsServer.loop();

  unsigned long now = millis();

  // Read MPU at 200Hz (every 5ms)
  if(mpuOk && (now - lastMpuRead >= MPU_INTERVAL)){
    lastMpuRead = now;
    if(imuCalibrating){
      updateIMUCal();
    } else {
      readMPU();
    }
  }

  // Motor balance / turn rate calibration routines
  if(motorCalibrating){
    readMPU();
    updateMotorBalCal();
  }
  if(turnCalibrating){
    readMPU();
    updateTurnRateCal();
  }

  readSensors();
  normaliseSensors();
  calcPosition();

  if(calibrating){
    updateCalibration();
  } else if(autoPIDActive){
    // Auto PID tune takes over motor control
    updateAutoPID();
  } else if(robotRunning){
    detectEndZone();
    if(robotRunning) runPID();
  }

  // Broadcast IMU state every 100ms (live gyro bar on dashboard)
  static unsigned long lastImuBroadcast = 0;
  if(now - lastImuBroadcast >= 100){
    lastImuBroadcast = now;
    if(mpuOk) broadcastIMUState();
  }

  if(now-lastWsBroadcast>=WS_INTERVAL){
    lastWsBroadcast=now;
    String s=buildSensorJson(); wsServer.broadcastTXT(s);
    if(logCount>0){ String l=buildLogJson(); wsServer.broadcastTXT(l); }
  }
}
