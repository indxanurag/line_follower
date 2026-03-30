/*
 * ============================================================
 *  8-Sensor Line Follower — ESP32 + DRV8833
 * ============================================================
 *  IR:     IR2→36  IR3→39  IR4→34  IR5→35  IR6→32  IR7→33
 *          (IR1 and IR8 dropped — all 6 on ADC1)
 *  Motors: IN1→23  IN2→19  (Left)
 *          IN3→13  IN4→27  (Right)
 *
 *  Libraries needed:
 *    · WebSockets  by Markus Sattler
 *    · ArduinoJson by Benoit Blanchon
 *
 *  HOW TO USE:
 *    1. Flash, open Serial at 115200, get the IP
 *    2. Browse to http://<IP>/ 
 *    3. Click Calibrate — sweep bot over line for 8 s
 *    4. Click Save
 *    5. Toggle "Line Color" switch: BLACK line on white BG (default)
 *       or WHITE line on black BG
 *    6. Click Start
 *    7. Tune Kp / Kd live with sliders
 * ============================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>

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
// TURN_SHARPNESS  (0.1 – 2.0): multiplier on the PID correction value.
//   < 1.0 → gentler / wider turns   (good for fast straight tracks)
//   = 1.0 → neutral (default)
//   > 1.0 → sharper / tighter turns (good for tight corners)
//
// TURN_BRAKE  (0 – 100 %): extra braking applied to the INNER wheel
//   during a turn.  0 = no brake (both wheels just change speed),
//   100 = inner wheel is actively slowed to MIN_SPEED on max error.
//   This is the most direct way to shrink the turning radius.
float TURN_SHARPNESS = 1.0f;
int   TURN_BRAKE     = 0;        // percent, 0-100

// ── Hairpin / curve assist ────────────────────────────────
// CURVE_EXP: non-linear correction exponent (1.0=linear, 2.0=quadratic).
//   Larger = gentle on straights, dramatically harder on big errors.
//   Good range for hairpins: 1.6 – 2.2
float CURVE_EXP = 1.0f;

// TURN_SPEED_REDUCE: auto-slow on high error. 0=off, 100=stop at max error.
//   50 = runs at 50% BASE_SPEED when fully off-centre entering a hairpin.
int   TURN_SPEED_REDUCE = 0;     // percent, 0-100

// PIVOT_THRESHOLD: error above which inner wheel is REVERSED (pivot turn).
//   0 = disabled. 1800-2200 = activates only on near-lost-line hairpin errors.
int   PIVOT_THRESHOLD = 0;

// ── Calibration ──────────────────────────────────────────
int  calMin[NUM_SENSORS], calMax[NUM_SENSORS];
bool sensorValid[NUM_SENSORS];
bool isCalibrated = false;
bool calibrating  = false;
unsigned long calStart = 0;
const unsigned long CAL_DURATION = 8000;

// ── Sensor data ──────────────────────────────────────────
int rawVals[NUM_SENSORS], normVals[NUM_SENSORS];

// ── Line color mode ──────────────────────────────────────
// false = black line on white background (IR reads HIGH on line)
// true  = white line on black background (IR reads LOW on line)
bool invertLine = false;

// ── State ────────────────────────────────────────────────
bool robotRunning = false;

// ── Web servers ──────────────────────────────────────────
WebServer        httpServer(80);
WebSocketsServer wsServer(81);
Preferences      prefs;
unsigned long    lastWsBroadcast = 0;
const unsigned long WS_INTERVAL  = 50;

// ══════════════════════════════════════════════════════════
//  HTML Dashboard
// ══════════════════════════════════════════════════════════
const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Line Follower</title>
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

/* ── Line Color Toggle ── */
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
</style>
</head>
<body>
<h1>Line Follower <span id="wsStatus" style="font-size:.7rem;color:#444">connecting...</span></h1>

<!-- CONTROL -->
<div class="card">
  <h2>Control</h2>
  <div class="status">
    <div class="dot" id="runDot"></div>
    <span id="runLabel">Stopped</span>
  </div>
  <div class="row">
    <button class="bg" id="btnStart"  onclick="cmd('start')">▶ Start</button>
    <button class="br" id="btnStop"   onclick="cmd('stop')"  disabled>⏹ Stop</button>
    <button class="by" id="btnCal"    onclick="cmd('calibrate')">⚡ Calibrate</button>
    <button class="bb" id="btnSave"   onclick="cmd('save')"  disabled>💾 Save</button>
  </div>
  <div class="hint">Sweep bot over line for 8 seconds during calibration.</div>
  <div id="calBar"><div id="calFill"></div></div>
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
  <div style="font-size:.7rem;color:#444">
    <b style="color:#666">Black line</b> on white background — IR sensor reads <b style="color:#7dd3fc">HIGH</b> over line.<br>
    <b style="color:#666">White line</b> on black background — sensor values are <b style="color:#f59e0b">inverted</b> before PID.
  </div>
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

<!-- SPEED -->
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

<!-- TURN RADIUS -->
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
    <b style="color:#666">Curve Exponent</b> — most effective for hairpins. Try 1.8–2.2. Big errors get disproportionately more correction, small errors stay smooth.<br>
    <b style="color:#666">Speed Reduce</b> — bot slows into the corner automatically. Try 40–60% for tight U-turns.<br>
    <b style="color:#666">Pivot Threshold</b> — inner wheel goes <em>backward</em> above this error. Try 1800–2200 for U-turns only.
  </div>
</div>

<!-- SERIAL MONITOR -->
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
    if(d.type==='log')     appendLog(d.lines);
  };
}
connect();

function setWs(ok){
  const el=document.getElementById('wsStatus');
  el.textContent=ok?'● connected':'○ disconnected';
  el.style.color=ok?'#22c55e':'#ef4444';
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
    d.calibrating?'Calibrating...':d.running?'Running':'Stopped';
  document.getElementById('btnStart').disabled=d.running||d.calibrating;
  document.getElementById('btnStop').disabled=!d.running&&!d.calibrating;
  document.getElementById('btnCal').disabled=d.running||d.calibrating;
  document.getElementById('btnSave').disabled=!d.calibrated&&!d.calibrating;

  // Sync invert toggle from server
  if(typeof d.invertLine !== 'undefined' && d.invertLine !== invertLine){
    invertLine=d.invertLine;
    applyLineModeUI();
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
    drawTurnPreview(d.turnSharpness, d.turnBrake, d.curveExp, d.turnSpeedReduce, d.pivotThreshold);
  }
}

function updateSensors(d){
  for(let i=0;i<6;i++){
    document.getElementById('normBars'+i).style.height=(d.norm[i]/10)+'%';
    document.getElementById('normBarsn'+i).textContent=d.norm[i];
    if(d.raw){
      document.getElementById('rawBars'+i).style.height=(d.raw[i]/40.95)+'%';
      document.getElementById('rawBarsn'+i).textContent=d.raw[i];
    }
  }
  const pct=((d.position+3500)/7000)*100;
  document.getElementById('posDot').style.left=pct+'%';
  document.getElementById('posText').textContent=
    'pos: '+d.position+'  err: '+d.error.toFixed(1);
}

function appendLog(lines){
  const box=document.getElementById('logBox');
  lines.forEach(l=>{
    const div=document.createElement('div');
    div.className='ll'+(l.includes('[ERR]')?' e':
                        l.includes('[WARN]')?' w':
                        (l.includes('[IMU]')||l.includes('[WIFI]')||l.includes('[WEB]')||l.includes('[MODE]'))?' i':'');
    div.textContent=l;
    box.appendChild(div);
  });
  box.scrollTop=box.scrollHeight;
}
function clearLog(){ document.getElementById('logBox').innerHTML=''; }

function cmd(c,ex={}){
  if(ws&&ws.readyState===1) ws.send(JSON.stringify({cmd:c,...ex}));
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
  document.getElementById('sharpnessv').textContent=sh.toFixed(2)+'×';
  document.getElementById('brakev').textContent=br+'%';
  document.getElementById('curveExpv').textContent=ce.toFixed(1);
  document.getElementById('turnSpeedReducev').textContent=sr+'%';
  document.getElementById('pivotThresholdv').textContent=pt>0?pt:'off';
  drawTurnPreview(sh, br, ce, sr, pt);
  cmd('turn',{sharpness:sh, brake:br, curveExp:ce, turnSpeedReduce:sr, pivotThreshold:pt});
}
function drawTurnPreview(sharpness, brake, curveExp, speedReduce, pivotThresh){
  const canvas=document.getElementById('turnCanvas');
  if(!canvas) return;
  const ctx=canvas.getContext('2d');
  const W=canvas.width, H=canvas.height;
  ctx.clearRect(0,0,W,H);

  // Simulate left and right wheel speeds across error range -2500..2500
  // and draw the resulting curve path
  const steps=80;
  ctx.beginPath();
  let x=W*0.15, y=H*0.85, angle=-Math.PI/2; // start pointing up
  for(let s=0;s<=steps;s++){
    const err=(s/steps)*2500; // 0..2500
    const normErr=err/2500;
    const speedScale=1-(normErr*(speedReduce/100));
    const base=Math.max(0.2, speedScale);
    // non-linear correction magnitude
    const corrMag=Math.pow(normErr, curveExp)*sharpness;
    let L=base-corrMag, R=base+corrMag;
    // brake
    if(brake>0) L=L-(L-0.1)*(normErr*(brake/100));
    // pivot
    if(pivotThresh>0 && err>=pivotThresh){
      const pd=(err-pivotThresh)/(2500-pivotThresh);
      L=-0.1-pd*0.4;
    }
    // angular change proportional to (R-L)
    const dAngle=(R-L)*0.04;
    angle+=dAngle;
    x+=Math.cos(angle)*3.5;
    y+=Math.sin(angle)*3.5;
    if(s===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  }
  ctx.strokeStyle='#7dd3fc';
  ctx.lineWidth=2;
  ctx.stroke();
  // Start dot
  ctx.beginPath(); ctx.arc(W*0.15,H*0.85,4,0,2*Math.PI);
  ctx.fillStyle='#22c55e'; ctx.fill();
  // Labels
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
  L=constrain(L,-255,255); R=constrain(R,-255,255);
  ledcWrite(PWM_L1_CH,L>=0?L:0); ledcWrite(PWM_L2_CH,L<0?-L:0);
  ledcWrite(PWM_R3_CH,R>=0?R:0); ledcWrite(PWM_R4_CH,R<0?-R:0);
}
void stopMotors(){
  ledcWrite(PWM_L1_CH,0); ledcWrite(PWM_L2_CH,0);
  ledcWrite(PWM_R3_CH,0); ledcWrite(PWM_R4_CH,0);
}


// ══════════════════════════════════════════════════════════
//  IR sensors
// ══════════════════════════════════════════════════════════
void readSensors(){
  for(int i=0;i<NUM_SENSORS;i++) rawVals[i]=analogRead(IR_PINS[i]);
}

// Called once after calibration — marks sensors valid if span >= 150
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
    // ── Invert for white-line-on-black-background ──────────
    // In default (black line) mode: high raw ADC → high norm (sensor over line).
    // In white-line mode: the contrast is reversed, so we flip the normalised value
    // so that 1000 still means "sensor is over the line".
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
//  Calibration — 8-second manual sweep
// ══════════════════════════════════════════════════════════
void startCalibration(){
  for(int i=0;i<NUM_SENSORS;i++){ calMin[i]=4095; calMax[i]=0; }
  isCalibrated=false;
  calibrating=true;
  calStart=millis();
  wsLog("[CAL] Started -- sweep bot over line for 8 seconds");
  wsLog("[CAL] Mode: "+(invertLine?String("WHITE line on black BG"):String("BLACK line on white BG")));
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
    wsLog("[CAL] Done!");
    evaluateSensors();
    String s=buildStateJson(); wsServer.broadcastTXT(s);
  }
}


// ══════════════════════════════════════════════════════════
//  PID  (with hairpin / U-turn assist)
// ══════════════════════════════════════════════════════════
void runPID(){
  pidError    = (float)lastPosition;
  pidIntegral = constrain(pidIntegral + pidError, -10000.0f, 10000.0f);
  float rawCorr = Kp*pidError + Ki*pidIntegral + Kd*(pidError - pidLastError);
  pidLastError  = pidError;

  // ── 1. Non-linear (exponential) correction ───────────────
  // Raise |rawCorr| to CURVE_EXP power, preserving sign.
  // CURVE_EXP=1.0 → no change (linear, original behaviour).
  // CURVE_EXP=2.0 → small errors barely change, big errors get squared.
  // Normalise to MAX_SPEED range so we don't blow up the value.
  float normCorr = rawCorr / (float)MAX_SPEED;          // approx -1..1 range
  float sign     = (normCorr >= 0) ? 1.0f : -1.0f;
  float expCorr  = sign * powf(fabsf(normCorr), CURVE_EXP) * (float)MAX_SPEED;
  float corr     = expCorr * TURN_SHARPNESS;

  // ── 2. Auto speed reduction on high error ────────────────
  // As |error| grows toward 2500 (max), reduce effective base speed.
  // TURN_SPEED_REDUCE=50 → at max error, speed = 50% of BASE_SPEED.
  float normErr   = constrain(fabsf(pidError) / 2500.0f, 0.0f, 1.0f);
  float speedScale = 1.0f - normErr * (TURN_SPEED_REDUCE / 100.0f);
  int   effBase   = (int)(BASE_SPEED * speedScale);
  effBase         = constrain(effBase, MIN_SPEED, MAX_SPEED);

  int L = constrain(effBase - (int)corr, -MAX_SPEED, MAX_SPEED);
  int R = constrain(effBase + (int)corr, -MAX_SPEED, MAX_SPEED);

  // ── 3. Turn brake (existing) ─────────────────────────────
  // Additional braking on inner wheel proportional to error magnitude.
  if(TURN_BRAKE > 0){
    float brakeScale = normErr * (TURN_BRAKE / 100.0f);
    if(pidError > 0){
      // turning right → left (inner) wheel braked
      int newL = (int)(L - (L - MIN_SPEED) * brakeScale);
      L = constrain(newL, MIN_SPEED, MAX_SPEED);
    } else {
      // turning left → right (inner) wheel braked
      int newR = (int)(R - (R - MIN_SPEED) * brakeScale);
      R = constrain(newR, MIN_SPEED, MAX_SPEED);
    }
  }

  // ── 4. Pivot assist — inner wheel REVERSE on extreme error ─
  // When error exceeds PIVOT_THRESHOLD the inner wheel is driven backward.
  // This converts a wide arc into a near-pivot turn for hairpins/U-turns.
  // Only active when PIVOT_THRESHOLD > 0.
  if(PIVOT_THRESHOLD > 0 && fabsf(pidError) >= (float)PIVOT_THRESHOLD){
    // How far past threshold are we? 0..1
    float pivotDepth = constrain(
      (fabsf(pidError) - PIVOT_THRESHOLD) / (2500.0f - PIVOT_THRESHOLD),
      0.0f, 1.0f);
    // Reverse the inner wheel proportionally to pivotDepth
    int reverseSpeed = (int)(MIN_SPEED + pivotDepth * (MAX_SPEED - MIN_SPEED));
    if(pidError > 0){
      L = -reverseSpeed;   // inner wheel backward → tight right pivot
    } else {
      R = -reverseSpeed;   // inner wheel backward → tight left pivot
    }
  }

  // Final safety clamp — allow reverse only for pivot assist
  L = constrain(L, -MAX_SPEED, MAX_SPEED);
  R = constrain(R, -MAX_SPEED, MAX_SPEED);

  setMotors(L, R);
}


// ══════════════════════════════════════════════════════════
//  Flash
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
  prefs.putBool("inv",invertLine);   // ← save line mode
  prefs.end();
  wsLog("[FLASH] Saved. Mode: "+(invertLine?String("WHITE line"):String("BLACK line")));
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
    invertLine=prefs.getBool("inv",false);  // ← load line mode
    isCalibrated=true;
    evaluateSensors();
    wsLog("[FLASH] Loaded calibration.");
    wsLog("[FLASH] Line mode: "+(invertLine?String("WHITE line"):String("BLACK line")));
  }
  prefs.end();
}


// ══════════════════════════════════════════════════════════
//  WebSocket JSON builders
// ══════════════════════════════════════════════════════════
String buildStateJson(){
  StaticJsonDocument<384> doc;
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
  doc["invertLine"]=invertLine;   // ← send mode to UI
  String out; serializeJson(doc,out); return out;
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
    String l=buildLogJson();   wsServer.sendTXT(num,l);
    return;
  }
  if(type!=WStype_TEXT) return;

  StaticJsonDocument<256> doc;
  if(deserializeJson(doc,payload,length)) return;
  const char* c=doc["cmd"]; if(!c) return;

  if(!strcmp(c,"start")){
    robotRunning=true; pidIntegral=0; pidLastError=0;
    wsLog("[CMD] Start");
  }
  else if(!strcmp(c,"stop")){
    robotRunning=false; calibrating=false; stopMotors();
    wsLog("[CMD] Stop");
  }
  else if(!strcmp(c,"calibrate")){
    startCalibration();
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
  // ── Turn radius control ───────────────────────────────────
  else if(!strcmp(c,"turn")){
    TURN_SHARPNESS     = doc["sharpness"]        | TURN_SHARPNESS;
    TURN_BRAKE         = doc["brake"]            | TURN_BRAKE;
    CURVE_EXP          = doc["curveExp"]         | CURVE_EXP;
    TURN_SPEED_REDUCE  = doc["turnSpeedReduce"]  | TURN_SPEED_REDUCE;
    PIVOT_THRESHOLD    = doc["pivotThreshold"]   | PIVOT_THRESHOLD;
    TURN_SHARPNESS     = constrain(TURN_SHARPNESS, 0.1f, 3.0f);
    TURN_BRAKE         = constrain(TURN_BRAKE, 0, 100);
    CURVE_EXP          = constrain(CURVE_EXP, 1.0f, 3.0f);
    TURN_SPEED_REDUCE  = constrain(TURN_SPEED_REDUCE, 0, 100);
    PIVOT_THRESHOLD    = constrain(PIVOT_THRESHOLD, 0, 2500);
    pidIntegral=0;
    wsLog("[TURN] sharp="+String(TURN_SHARPNESS,2)
         +" brake="+String(TURN_BRAKE)+"%"
         +" exp="+String(CURVE_EXP,2)
         +" spdReduce="+String(TURN_SPEED_REDUCE)+"%"
         +" pivot="+String(PIVOT_THRESHOLD));
  }
  // ── NEW: line color mode toggle ──────────────────────────
  else if(!strcmp(c,"linemode")){
    bool newInvert = doc["invert"] | invertLine;
    if(newInvert != invertLine){
      invertLine = newInvert;
      pidIntegral=0; pidLastError=0; // reset PID to avoid jerk on flip
      wsLog("[MODE] Line color: "+(invertLine?String("WHITE line on black BG"):String("BLACK line on white BG")));
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
// ══════════════════════════════════════════════════════════
void setup(){
  Serial.begin(115200); delay(400);
  wsLog("===== Line Follower =====");

  for(int i=0;i<NUM_SENSORS;i++) sensorValid[i]=true;

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  for(int i=0;i<NUM_SENSORS;i++){
    pinMode(IR_PINS[i], INPUT);
    analogSetPinAttenuation(IR_PINS[i], ADC_11db);
  }
  motorSetup(); stopMotors();

  loadAll();

  wsLog("[WIFI] Connecting to "+String(SSID)+"...");
  WiFi.begin(SSID,PASSWORD);
  while(WiFi.status()!=WL_CONNECTED){ delay(500); Serial.print("."); }
  Serial.println();
  wsLog("[WIFI] IP: "+WiFi.localIP().toString());
  wsLog("[WEB]  http://"+WiFi.localIP().toString()+"/");
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

  readSensors();
  normaliseSensors();
  calcPosition();

  if(calibrating){
    updateCalibration();
  } else if(robotRunning){
    runPID();
  }

  unsigned long now=millis();
  if(now-lastWsBroadcast>=WS_INTERVAL){
    lastWsBroadcast=now;
    String s=buildSensorJson(); wsServer.broadcastTXT(s);
    if(logCount>0){ String l=buildLogJson(); wsServer.broadcastTXT(l); }
  }
}
