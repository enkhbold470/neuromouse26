// include/WifiDebug.h — DORMANT HTTP telemetry / live-tuning dashboard.
//
// NOT included by [env:main]. Historical wall-follow / web-debugger side path.
// Do not commit real WiFi credentials — set them locally (or via build flags)
// before flashing any sketch that includes this header.
#pragma once
#include <WiFi.h>
#include <WebServer.h>

#ifndef WIFI_SSID
#define WIFI_SSID "YOUR_WIFI_SSID"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "YOUR_WIFI_PASSWORD"
#endif
#define WIFI_DEBUG_PORT 80

// ── Tuning Config ─────────────────────────────────────────────────────────────
struct TuningConfig {
    int   basePwm     = 320,         turnPwm    = TURN_PWM;
    float wallKp      = WALL_KP,     wallKi     = WALL_KI,   wallKd    = WALL_KD;
    int   wallMaxCorr = WALL_MAX_CORR;
    float encKp       = ENC_KP,      encKi      = ENC_KI,    encKd     = ENC_KD;
    int   encMaxCorr  = ENC_MAX_CORR;
    int   l45Center   = L_CENTER,  r45Center  = R_CENTER;
    int   l45Thresh   = L_THRESH,  r45Thresh  = R_THRESH;
    int   lfThresh    = LF_THRESH,   rfThresh   = RF_THRESH;
    int   ticksPerCell = TICKS_PER_CELL, cellPauseMs = CELL_PAUSE_MS;
    int   mazeRows    = 6,           mazeCols   = 3;
    int   goalRow     = 5,           goalCol    = 1;
};

// ── Debug Snapshot ────────────────────────────────────────────────────────────
struct DebugSnapshot {
    int     ir[4];
    long    encL, encR;
    float   vBat;
    int     state;    // 0=IDLE 1=RUN 2=GOAL 3=STOP
    int     row, col, heading;
    bool    wallF, wallL, wallR;
    uint8_t mazeWalls  [MAZE_SIZE][MAZE_SIZE];
    uint8_t mazeFlood  [MAZE_SIZE][MAZE_SIZE];
    uint8_t mazeVisited[MAZE_SIZE][MAZE_SIZE];
    int     mazeRows, mazeCols;
};

static DebugSnapshot _dbgSnap;
static WebServer     _dbgServer(WIFI_DEBUG_PORT);
static bool          _wifiReady     = false;
static TuningConfig* _tuning        = nullptr;
static bool          _configChanged = false;
static int         (*_irReadFn)(int) = nullptr;  // set via wifiDebugSetIRReader()

// ── HTML Dashboard ────────────────────────────────────────────────────────────
static const char _DBG_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html><html><head><meta charset='utf-8'>
<title>Micromouse Debug</title>
<style>
*{box-sizing:border-box}
body{font-family:monospace;background:#111;color:#0f0;padding:16px;max-width:960px;margin:0 auto}
h2,h3{color:#0ff;margin:8px 0}
.val{color:#ff0}
.wall{color:#f44;font-weight:bold}
.clear{color:#0f0}
table{border-collapse:collapse;width:100%;margin-bottom:10px}
td,th{padding:4px 8px;border:1px solid #333}
th{background:#222;color:#0ff;text-align:left}
input[type=number]{background:#1a1a1a;color:#ff0;border:1px solid #444;padding:2px 6px;width:88px;font-family:monospace}
button{background:#030;color:#0f0;border:1px solid #0f0;padding:6px 20px;cursor:pointer;font-family:monospace;margin-top:8px}
button:hover{background:#050}
#status{position:fixed;top:10px;right:10px;padding:4px 10px;border-radius:4px;font-size:12px}
.ok{background:#0a0}.err{background:#600}
.grid{display:grid;grid-template-columns:1fr auto;gap:20px;align-items:start}
canvas{display:block;border:2px solid #0f0;background:#0d0d0d;image-rendering:pixelated}
fieldset{border:1px solid #333;margin:6px 0;padding:8px 12px}
legend{color:#0ff;padding:0 4px}
.row{display:flex;gap:8px;align-items:center;margin:3px 0}
label{color:#888;min-width:130px;font-size:13px}
.cfg-grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:0 16px}
</style></head><body>
<div id='status' class='ok'>LIVE</div>
<h2>&#x1F42D; Micromouse Live Debug</h2>

<div class='grid'>
<div id='sensors'>Loading...</div>
<div>
<h3>Maze</h3>
<canvas id='maze'></canvas>
<div id='mazeinfo' style='color:#555;font-size:11px;margin-top:4px'></div>
</div>
</div>

<div style='margin-top:20px'>
<h3>Config <span style='color:#555;font-size:13px'>(submit applies instantly, rebuilds PIDs + maze)</span></h3>
<form method='POST' action='/config' id='cfg'>
<div class='cfg-grid'>
<fieldset><legend>Drive</legend>
<div class='row'><label>Base PWM (0-255)</label><input name='basePwm' type='number' min='0' max='255'></div>
<div class='row'><label>Turn PWM (0-255)</label><input name='turnPwm' type='number' min='0' max='255'></div>
<div class='row'><label>Ticks / Cell</label><input name='ticksPerCell' type='number' min='1' max='9999'></div>
<div class='row'><label>Cell Pause ms</label><input name='cellPauseMs' type='number' min='0' max='5000'></div>
</fieldset>
<fieldset><legend>Wall PID</legend>
<div class='row'><label>Kp</label><input name='wallKp' type='number' step='0.001'></div>
<div class='row'><label>Ki</label><input name='wallKi' type='number' step='0.001'></div>
<div class='row'><label>Kd</label><input name='wallKd' type='number' step='0.001'></div>
<div class='row'><label>Max Corr</label><input name='wallMaxCorr' type='number' min='0' max='255'></div>
</fieldset>
<fieldset><legend>Encoder PID</legend>
<div class='row'><label>Kp</label><input name='encKp' type='number' step='0.001'></div>
<div class='row'><label>Ki</label><input name='encKi' type='number' step='0.001'></div>
<div class='row'><label>Kd</label><input name='encKd' type='number' step='0.001'></div>
<div class='row'><label>Max Corr</label><input name='encMaxCorr' type='number' min='0' max='255'></div>
</fieldset>
<fieldset><legend>IR Sensors</legend>
<div class='row'><label>L45 Center</label><input name='l45Center' type='number'></div>
<div class='row'><label>R45 Center</label><input name='r45Center' type='number'></div>
<div class='row'><label>L45 Thresh</label><input name='l45Thresh' type='number'></div>
<div class='row'><label>R45 Thresh</label><input name='r45Thresh' type='number'></div>
<div class='row'><label>LF Thresh</label><input name='lfThresh' type='number'></div>
<div class='row'><label>RF Thresh</label><input name='rfThresh' type='number'></div>
</fieldset>
<fieldset><legend>Maze / Goal</legend>
<div class='row'><label>Rows (1-16)</label><input name='mazeRows' type='number' min='1' max='16'></div>
<div class='row'><label>Cols (1-16)</label><input name='mazeCols' type='number' min='1' max='16'></div>
<div class='row'><label>Goal Row</label><input name='goalRow' type='number' min='0' max='15'></div>
<div class='row'><label>Goal Col</label><input name='goalCol' type='number' min='0' max='15'></div>
</fieldset>
</div>
<button type='submit'>&#x25B6; Apply Config</button>
</form>
</div>

<script>
const STATES=['IDLE','RUN','GOAL','STOP'];
const DIRS=['N','E','S','W'];
function w(v){return v?'<span class="wall">WALL</span>':'<span class="clear">---</span>';}

function pollSensors(){
  fetch('/data').then(r=>r.json()).then(d=>{
    document.getElementById('status').className='ok';
    document.getElementById('status').textContent='LIVE';
    const pct=Math.max(0,Math.min(100,((d.vbat-6.8)/(8.2-6.8)*100))).toFixed(0);
    document.getElementById('sensors').innerHTML=
      '<table><th colspan=2>STATE</th>'+
      '<tr><td>State</td><td class="val">'+STATES[d.state]+'</td></tr>'+
      '<tr><td>Position</td><td class="val">Row '+d.row+' &nbsp;Col '+d.col+' &nbsp;'+DIRS[d.heading]+'</td></tr>'+
      '<tr><td>Battery</td><td class="val">'+d.vbat.toFixed(2)+'V &nbsp;'+pct+'%</td></tr>'+
      '</table>'+
      '<table><th>Sensor</th><th>Raw</th><th>Wall?</th>'+
      '<tr><td>LF</td><td class="val">'+d.ir0+'</td><td>'+w(d.wallF)+'</td></tr>'+
      '<tr><td>L45</td><td class="val">'+d.ir1+'</td><td>'+w(d.wallL)+'</td></tr>'+
      '<tr><td>R45</td><td class="val">'+d.ir2+'</td><td>'+w(d.wallR)+'</td></tr>'+
      '<tr><td>RF</td><td class="val">'+d.ir3+'</td><td>'+w(d.wallF)+'</td></tr>'+
      '</table>'+
      '<table><th colspan=2>ENCODERS</th>'+
      '<tr><td>Left</td><td class="val">'+d.encL+'</td></tr>'+
      '<tr><td>Right</td><td class="val">'+d.encR+'</td></tr>'+
      '<tr><td>Drift L&#x2212;R</td><td class="val">'+(d.encL-d.encR)+'</td></tr>'+
      '</table>';
  }).catch(()=>{
    document.getElementById('status').className='err';
    document.getElementById('status').textContent='LOST';
  });
}

function pollMaze(){
  fetch('/maze').then(r=>r.json()).then(drawMaze).catch(()=>{});
}

function drawMaze(d){
  const rows=d.rows, cols=d.cols;
  const cellPx=Math.min(40,Math.floor(480/Math.max(rows,cols)));
  const cv=document.getElementById('maze');
  cv.width=cols*cellPx; cv.height=rows*cellPx;
  const ctx=cv.getContext('2d');
  ctx.clearRect(0,0,cv.width,cv.height);
  for(let r=0;r<rows;r++){
    for(let c=0;c<cols;c++){
      const i=r*cols+c;
      const wx=c*cellPx;
      const wy=(rows-1-r)*cellPx; // row 0 at bottom
      ctx.fillStyle=d.v[i]?'#1a2a1a':'#0d0d0d';
      ctx.fillRect(wx,wy,cellPx,cellPx);
      if(d.f[i]===0){
        ctx.fillStyle='rgba(255,200,0,0.18)';
        ctx.fillRect(wx+1,wy+1,cellPx-2,cellPx-2);
      }
      if(d.f[i]<255&&cellPx>=14){
        ctx.fillStyle='#445';
        ctx.font=Math.floor(cellPx*0.3)+'px monospace';
        ctx.textAlign='center';
        ctx.fillText(d.f[i],wx+cellPx/2,wy+cellPx*0.66);
      }
      ctx.strokeStyle='#0f0'; ctx.lineWidth=2;
      const wl=d.w[i];
      // NORTH=0x01 → top edge (row flip makes north = visually up)
      if(wl&1){ctx.beginPath();ctx.moveTo(wx,wy);ctx.lineTo(wx+cellPx,wy);ctx.stroke();}
      if(wl&2){ctx.beginPath();ctx.moveTo(wx+cellPx,wy);ctx.lineTo(wx+cellPx,wy+cellPx);ctx.stroke();}
      if(wl&4){ctx.beginPath();ctx.moveTo(wx,wy+cellPx);ctx.lineTo(wx+cellPx,wy+cellPx);ctx.stroke();}
      if(wl&8){ctx.beginPath();ctx.moveTo(wx,wy);ctx.lineTo(wx,wy+cellPx);ctx.stroke();}
    }
  }
  // robot triangle
  const rx=d.rc*cellPx+cellPx/2;
  const ry=(rows-1-d.rr)*cellPx+cellPx/2;
  const rad=cellPx*0.38;
  const angs=[-Math.PI/2,0,Math.PI/2,Math.PI];
  const a=angs[d.rh]||0;
  ctx.fillStyle='#0ff';
  ctx.beginPath();
  ctx.moveTo(rx+rad*Math.cos(a),ry+rad*Math.sin(a));
  ctx.lineTo(rx+rad*0.5*Math.cos(a+2.5),ry+rad*0.5*Math.sin(a+2.5));
  ctx.lineTo(rx+rad*0.5*Math.cos(a-2.5),ry+rad*0.5*Math.sin(a-2.5));
  ctx.closePath(); ctx.fill();
  document.getElementById('mazeinfo').textContent=rows+'x'+cols+' grid  cell='+cellPx+'px  robot@('+d.rr+','+d.rc+') '+['N','E','S','W'][d.rh];
}

function loadConfig(){
  fetch('/config').then(r=>r.json()).then(d=>{
    Object.keys(d).forEach(k=>{
      const el=document.querySelector('[name='+k+']');
      if(el) el.value=d[k];
    });
  }).catch(()=>{});
}

// ── Calibration ───────────────────────────────────────────────────────────────
let _calNoWall = null, _calCenter = null, _calFront = null;
const SNAMES = ['LF','L45','R45','RF'];

function calRead(step, btn) {
  btn.disabled = true; btn.textContent = 'Reading...';
  fetch('/calibrate').then(r=>r.json()).then(d=>{
    btn.disabled = false; btn.textContent = 'Read Again';
    const avg = d.avg;
    let ok, msg;
    if (step === 'nowall') {
      _calNoWall = d;
      ok = avg.every(v=>v<30);
      msg = SNAMES.map((n,i)=>`${n}:${avg[i]}`).join(' &nbsp;');
      document.getElementById('cal-nowall-res').innerHTML = msg +
        (ok ? ' &nbsp;<span style="color:#0f0">&#10003; clear</span>'
             : ' &nbsp;<span style="color:#f44">&#9888; values high &mdash; check ambient light / wiring</span>');
    } else if (step === 'center') {
      _calCenter = d;
      ok = avg[1] > 50 && avg[2] > 50;
      msg = `L45:${avg[1]} &nbsp; R45:${avg[2]}`;
      document.getElementById('cal-center-res').innerHTML = msg +
        (ok ? ' &nbsp;<span style="color:#0f0">&#10003; walls detected</span>'
             : ' &nbsp;<span style="color:#f44">&#9888; low readings &mdash; position closer to walls</span>');
    } else if (step === 'front') {
      _calFront = d;
      ok = avg[0] > 50 && avg[3] > 50;
      msg = `LF:${avg[0]} &nbsp; RF:${avg[3]}`;
      document.getElementById('cal-front-res').innerHTML = msg +
        (ok ? ' &nbsp;<span style="color:#0f0">&#10003; wall detected</span>'
             : ' &nbsp;<span style="color:#f44">&#9888; low &mdash; move closer to front wall</span>');
    }
    document.getElementById('cal-apply').disabled = !(_calNoWall && _calCenter);
  }).catch(e=>{
    btn.disabled=false; btn.textContent='Retry';
    document.getElementById('cal-'+step+'-res').innerHTML='<span style="color:#f44">Error: '+e+'</span>';
  });
}

function applyCalibration() {
  if (!_calCenter) return;
  const l45c = _calCenter.avg[1];
  const r45c = _calCenter.avg[2];
  // Threshold: midpoint between max no-wall and min wall (with 20 headroom)
  let thr = 50;
  if (_calNoWall) {
    const noWallMax = Math.max(..._calNoWall.max);
    const wallMin   = Math.min(_calCenter.min[1], _calCenter.min[2]);
    thr = Math.round((noWallMax + wallMin) / 2);
    thr = Math.max(20, Math.min(300, thr));
  }
  const params = new URLSearchParams({
    l45Center: l45c, r45Center: r45c,
    l45Thresh: thr,  r45Thresh: thr,
    lfThresh:  thr,  rfThresh:  thr
  });
  fetch('/config',{method:'POST',body:params}).then(()=>{
    document.getElementById('cal-status').innerHTML=
      `<span style="color:#0f0">&#10003; Applied &mdash; L_CENTER=${l45c} R_CENTER=${r45c} THRESH=${thr}</span>`;
    loadConfig();
  }).catch(e=>{
    document.getElementById('cal-status').innerHTML='<span style="color:#f44">POST failed: '+e+'</span>';
  });
}

setInterval(pollSensors,300);
setInterval(pollMaze,1000);
pollSensors(); pollMaze(); loadConfig();
</script>

<div style='margin-top:24px;border:1px solid #0ff;padding:16px'>
<h3 style='color:#0ff;margin:0 0 8px'>IR Sensor Calibration</h3>
<p style='color:#888;margin:0 0 12px;font-size:13px'>Do this before pressing Button 1. Robot must be in IDLE state.</p>

<div style='margin-bottom:14px;padding:10px;border:1px solid #333'>
<b style='color:#ff0'>Step 1 &mdash; Open Space</b><br>
<span style='color:#888;font-size:13px'>Place robot in open area. No walls within 15 cm of any sensor.</span><br>
<button style='margin-top:6px' onclick='calRead("nowall",this)'>Read No-Wall</button>
<div id='cal-nowall-res' style='margin-top:4px;font-size:13px'></div>
</div>

<div style='margin-bottom:14px;padding:10px;border:1px solid #333'>
<b style='color:#ff0'>Step 2 &mdash; Side Wall Centers</b><br>
<span style='color:#888;font-size:13px'>Place robot centered in a corridor &mdash; walls on LEFT and RIGHT sides, open front.</span><br>
<button style='margin-top:6px' onclick='calRead("center",this)'>Read Centers</button>
<div id='cal-center-res' style='margin-top:4px;font-size:13px'></div>
</div>

<div style='margin-bottom:14px;padding:10px;border:1px solid #333'>
<b style='color:#ff0'>Step 3 &mdash; Front Wall (optional)</b><br>
<span style='color:#888;font-size:13px'>Place robot ~5 cm from a front wall. Verifies LF and RF sensors.</span><br>
<button style='margin-top:6px' onclick='calRead("front",this)'>Read Front</button>
<div id='cal-front-res' style='margin-top:4px;font-size:13px'></div>
</div>

<button id='cal-apply' onclick='applyCalibration()' disabled
  style='background:#003;border-color:#0ff;color:#0ff;padding:8px 24px'>
&#9654; Apply Calibration &rarr; Updates Centers + Thresholds
</button>
<div id='cal-status' style='margin-top:8px;font-size:13px'></div>
</div>

</body></html>
)rawhtml";

// ── Public API ────────────────────────────────────────────────────────────────
inline void wifiDebugSetIRReader(int (*fn)(int)) { _irReadFn = fn; }
inline void wifiDebugUpdate(const DebugSnapshot& snap) { _dbgSnap = snap; }

inline bool wifiDebugConfigChanged() {
    if (!_configChanged) return false;
    _configChanged = false;
    return true;
}

inline void wifiDebugInit(TuningConfig* t) {
    _tuning = t;
    Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
        delay(300); Serial.print('.');
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\n[WiFi] FAILED — no debug server");
        return;
    }
    _wifiReady = true;
    Serial.printf("\n[WiFi] Connected! http://%s\n", WiFi.localIP().toString().c_str());

    _dbgServer.on("/", []() {
        _dbgServer.send_P(200, "text/html", _DBG_HTML);
    });

    _dbgServer.on("/data", []() {
        char buf[256];
        snprintf(buf, sizeof(buf),
            "{\"ir0\":%d,\"ir1\":%d,\"ir2\":%d,\"ir3\":%d,"
            "\"encL\":%ld,\"encR\":%ld,\"vbat\":%.2f,"
            "\"state\":%d,\"row\":%d,\"col\":%d,\"heading\":%d,"
            "\"wallF\":%s,\"wallL\":%s,\"wallR\":%s}",
            _dbgSnap.ir[0], _dbgSnap.ir[1], _dbgSnap.ir[2], _dbgSnap.ir[3],
            _dbgSnap.encL, _dbgSnap.encR, _dbgSnap.vBat,
            _dbgSnap.state, _dbgSnap.row, _dbgSnap.col, _dbgSnap.heading,
            _dbgSnap.wallF?"true":"false",
            _dbgSnap.wallL?"true":"false",
            _dbgSnap.wallR?"true":"false");
        _dbgServer.sendHeader("Access-Control-Allow-Origin", "*");
        _dbgServer.send(200, "application/json", buf);
    });

    _dbgServer.on("/maze", []() {
        int rows = _dbgSnap.mazeRows, cols = _dbgSnap.mazeCols;
        String s;
        s.reserve(rows * cols * 10 + 80);
        s = "{\"rows\":"; s += rows;
        s += ",\"cols\":"; s += cols;
        s += ",\"w\":[";
        for (int r = 0; r < rows; r++) for (int c = 0; c < cols; c++) {
            if (r || c) s += ',';
            s += (int)_dbgSnap.mazeWalls[r][c];
        }
        s += "],\"f\":[";
        for (int r = 0; r < rows; r++) for (int c = 0; c < cols; c++) {
            if (r || c) s += ',';
            s += (int)_dbgSnap.mazeFlood[r][c];
        }
        s += "],\"v\":[";
        for (int r = 0; r < rows; r++) for (int c = 0; c < cols; c++) {
            if (r || c) s += ',';
            s += (int)_dbgSnap.mazeVisited[r][c];
        }
        s += "],\"rr\":"; s += _dbgSnap.row;
        s += ",\"rc\":"; s += _dbgSnap.col;
        s += ",\"rh\":"; s += _dbgSnap.heading;
        s += "}";
        _dbgServer.sendHeader("Access-Control-Allow-Origin", "*");
        _dbgServer.send(200, "application/json", s);
    });

    _dbgServer.on("/calibrate", []() {
        if (!_irReadFn) { _dbgServer.send(503, "text/plain", "no IR reader set"); return; }
        const int N = 20;
        float sum[4] = {};
        int   mn[4]  = {9999,9999,9999,9999};
        int   mx[4]  = {};
        for (int i = 0; i < N; i++) {
            for (int s = 0; s < 4; s++) {
                int v = _irReadFn(s);
                sum[s] += v;
                if (v < mn[s]) mn[s] = v;
                if (v > mx[s]) mx[s] = v;
            }
            delay(15);
        }
        char buf[256];
        snprintf(buf, sizeof(buf),
            "{\"avg\":[%d,%d,%d,%d],\"min\":[%d,%d,%d,%d],\"max\":[%d,%d,%d,%d]}",
            (int)(sum[0]/N),(int)(sum[1]/N),(int)(sum[2]/N),(int)(sum[3]/N),
            mn[0],mn[1],mn[2],mn[3],
            mx[0],mx[1],mx[2],mx[3]);
        _dbgServer.sendHeader("Access-Control-Allow-Origin","*");
        _dbgServer.send(200,"application/json",buf);
    });

    _dbgServer.on("/config", HTTP_GET, []() {
        if (!_tuning) { _dbgServer.send(500, "text/plain", "no tuning"); return; }
        String s = "{";
        s += "\"basePwm\":"      + String(_tuning->basePwm)      + ",";
        s += "\"turnPwm\":"      + String(_tuning->turnPwm)      + ",";
        s += "\"wallKp\":"       + String(_tuning->wallKp,  4)   + ",";
        s += "\"wallKi\":"       + String(_tuning->wallKi,  4)   + ",";
        s += "\"wallKd\":"       + String(_tuning->wallKd,  4)   + ",";
        s += "\"wallMaxCorr\":"  + String(_tuning->wallMaxCorr)  + ",";
        s += "\"encKp\":"        + String(_tuning->encKp,   4)   + ",";
        s += "\"encKi\":"        + String(_tuning->encKi,   4)   + ",";
        s += "\"encKd\":"        + String(_tuning->encKd,   4)   + ",";
        s += "\"encMaxCorr\":"   + String(_tuning->encMaxCorr)   + ",";
        s += "\"l45Center\":"    + String(_tuning->l45Center)    + ",";
        s += "\"r45Center\":"    + String(_tuning->r45Center)    + ",";
        s += "\"l45Thresh\":"    + String(_tuning->l45Thresh)    + ",";
        s += "\"r45Thresh\":"    + String(_tuning->r45Thresh)    + ",";
        s += "\"lfThresh\":"     + String(_tuning->lfThresh)     + ",";
        s += "\"rfThresh\":"     + String(_tuning->rfThresh)     + ",";
        s += "\"ticksPerCell\":" + String(_tuning->ticksPerCell) + ",";
        s += "\"cellPauseMs\":"  + String(_tuning->cellPauseMs)  + ",";
        s += "\"mazeRows\":"     + String(_tuning->mazeRows)     + ",";
        s += "\"mazeCols\":"     + String(_tuning->mazeCols)     + ",";
        s += "\"goalRow\":"      + String(_tuning->goalRow)      + ",";
        s += "\"goalCol\":"      + String(_tuning->goalCol);
        s += "}";
        _dbgServer.sendHeader("Access-Control-Allow-Origin", "*");
        _dbgServer.send(200, "application/json", s);
    });

    _dbgServer.on("/config", HTTP_POST, []() {
        if (!_tuning) { _dbgServer.send(500, "text/plain", "no tuning"); return; }
        #define _AI(n,f) if (_dbgServer.hasArg(n)) _tuning->f = _dbgServer.arg(n).toInt()
        #define _AF(n,f) if (_dbgServer.hasArg(n)) _tuning->f = _dbgServer.arg(n).toFloat()
        _AI("basePwm",     basePwm);
        _AI("turnPwm",     turnPwm);
        _AF("wallKp",      wallKp);
        _AF("wallKi",      wallKi);
        _AF("wallKd",      wallKd);
        _AI("wallMaxCorr", wallMaxCorr);
        _AF("encKp",       encKp);
        _AF("encKi",       encKi);
        _AF("encKd",       encKd);
        _AI("encMaxCorr",  encMaxCorr);
        _AI("l45Center",   l45Center);
        _AI("r45Center",   r45Center);
        _AI("l45Thresh",   l45Thresh);
        _AI("r45Thresh",   r45Thresh);
        _AI("lfThresh",    lfThresh);
        _AI("rfThresh",    rfThresh);
        _AI("ticksPerCell",ticksPerCell);
        _AI("cellPauseMs", cellPauseMs);
        _AI("mazeRows",    mazeRows);
        _AI("mazeCols",    mazeCols);
        _AI("goalRow",     goalRow);
        _AI("goalCol",     goalCol);
        #undef _AI
        #undef _AF
        _configChanged = true;
        _dbgServer.sendHeader("Location", "/");
        _dbgServer.send(303, "text/plain", "Redirecting...");
    });

    _dbgServer.begin();
    Serial.printf("[WiFi] Debug server on port %d\n", WIFI_DEBUG_PORT);
}

inline void wifiDebugHandle() {
    if (_wifiReady) _dbgServer.handleClient();
}
