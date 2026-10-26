#include <WiFi.h>
#include <WebServer.h>
#include <HX711.h>

// Hardware pins from the supplied wiring.
constexpr uint8_t HX711_DOUT_PIN = 32;
constexpr uint8_t HX711_SCK_PIN = 33;
constexpr uint8_t MOTOR_PWM_PIN = 25;

// Optional tachometer input. Connect an RPM sensor output to GPIO 27 and GND.
// Set TACH_ENABLED to false if no sensor is installed.
constexpr bool TACH_ENABLED = true;
constexpr uint8_t TACH_PIN = 27;
constexpr uint16_t TACH_PULSES_PER_REV = 1;

// This output is for a PWM motor driver. For a hobby ESC, configure the
// controller's required pulse protocol instead of using this generic PWM.
constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
constexpr uint8_t PWM_RESOLUTION_BITS = 8;
constexpr uint16_t SAMPLE_PERIOD_MS = 250;
constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 2500;
// Readings are median-filtered across a short window, then lightly smoothed.
constexpr uint8_t THRUST_FILTER_WINDOW = 5;
constexpr float THRUST_EMA_ALPHA = 0.25f;

const char *AP_SSID = "Thrust-Rig";
const char *AP_PASSWORD = "thrust1234"; // Change before use.

WebServer server(80);
HX711 loadCell;

portMUX_TYPE tachMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t tachPulses = 0;
uint32_t previousTachPulses = 0;
uint32_t previousRpmMillis = 0;
float rpm = 0;
float scaleFactor = 1.0f;
float thrustGrams = 0;
float latestRaw = 0;
float thrustSamples[THRUST_FILTER_WINDOW] = {0};
uint8_t thrustSampleCount = 0;
uint8_t thrustSampleIndex = 0;
bool thrustFilterInitialized = false;
uint32_t lastSampleMillis = 0;
uint32_t lastHeartbeatMillis = 0;
uint32_t runStartedMillis = 0;
uint32_t runDurationMillis = 0;
float runStartThrottle = 0;
float runEndThrottle = 0;
float throttlePercent = 0;
String runMode = "idle";
bool loadCellEnabled = true;
bool emergencyLatched = false;

const char DASHBOARD_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Thrust Rig</title>
<style>
:root{color-scheme:dark;--bg:#0b1118;--panel:#111a24;--line:#233243;--muted:#8b9aad;--text:#e8eff7;--cyan:#42d5c5;--blue:#6a9cff;--red:#ff5964;--amber:#ffbf5e}*{box-sizing:border-box}body{margin:0;background:radial-gradient(ellipse at 15% 0,#142634 0,transparent 45%),var(--bg);color:var(--text);font:14px/1.5 Inter,ui-sans-serif,system-ui,-apple-system,Segoe UI,sans-serif}.wrap{max-width:1180px;margin:auto;padding:28px 22px 50px}.top{display:flex;align-items:center;justify-content:space-between;gap:20px;margin-bottom:24px}.brand{display:flex;align-items:center;gap:12px}.mark{width:38px;height:38px;border:1px solid #326e76;border-radius:12px;display:grid;place-items:center;color:var(--cyan);font-size:20px;background:#102a30}.eyebrow,.label{font-size:11px;letter-spacing:.12em;text-transform:uppercase;color:var(--muted);font-weight:700}.title{font-size:20px;font-weight:700;letter-spacing:-.02em}.status{display:flex;align-items:center;gap:8px;border:1px solid var(--line);background:var(--panel);padding:8px 12px;border-radius:99px;color:#b9c6d5;font-size:12px}.dot{width:8px;height:8px;border-radius:50%;background:var(--cyan);box-shadow:0 0 12px #42d5c580}.dot.red{background:var(--red);box-shadow:0 0 12px #ff596480}.grid{display:grid;grid-template-columns:repeat(12,1fr);gap:14px}.card{background:linear-gradient(145deg,#131e29,#101821);border:1px solid var(--line);border-radius:16px;padding:18px;min-width:0}.metric{grid-column:span 3;min-height:130px}.metric .value{font-size:34px;letter-spacing:-.04em;font-weight:700;margin:9px 0 0}.unit{font-size:14px;color:var(--muted);font-weight:500}.sub{font-size:12px;color:var(--muted)}.main{grid-column:span 8}.side{grid-column:span 4}.half{grid-column:span 6}.cardhead{display:flex;justify-content:space-between;align-items:center;margin-bottom:14px}.cardtitle{font-weight:650;font-size:14px}.badge{border:1px solid var(--line);border-radius:8px;padding:4px 8px;color:var(--muted);font-size:11px}canvas{display:block;width:100%;height:240px}.smallchart canvas{height:190px}.row{display:flex;gap:10px;align-items:center}.row>*{flex:1}label{display:block;color:#aab8c8;font-size:12px;margin:13px 0 6px}input,select{width:100%;background:#0b121a;border:1px solid #2a3a4c;border-radius:9px;color:var(--text);padding:10px 11px;font:inherit;outline:none}input:focus,select:focus{border-color:#4d9c9b}.range{padding:0;accent-color:var(--cyan)}.throttle-line{display:flex;align-items:baseline;justify-content:space-between;margin:10px 0}.throttle-line strong{font-size:28px}.btn{border:1px solid #34485c;border-radius:9px;background:#1a2734;color:var(--text);font:600 13px inherit;padding:10px 12px;cursor:pointer;transition:.15s}.btn:hover{filter:brightness(1.18);transform:translateY(-1px)}.btn.primary{background:#164442;border-color:#287c75;color:#bffaf1}.btn.stop{background:#451c24;border-color:#923440;color:#ffd8dc}.btn.full{width:100%;margin-top:10px}.btn.small{padding:7px 10px;font-size:12px}.controls{display:grid;grid-template-columns:1fr 1fr;gap:8px}.note{font-size:11px;color:var(--muted);margin-top:12px;line-height:1.5}.divider{height:1px;background:var(--line);margin:16px 0}.switchrow{display:flex;align-items:center;justify-content:space-between;color:#bdc9d6;font-size:13px;margin:10px 0}.switch{appearance:none;width:38px;height:22px;background:#344353;border-radius:22px;position:relative;cursor:pointer;transition:.2s}.switch:checked{background:#23877e}.switch:before{content:"";position:absolute;width:16px;height:16px;left:3px;top:3px;border-radius:50%;background:white;transition:.2s}.switch:checked:before{left:19px}.toast{position:fixed;bottom:18px;left:50%;transform:translate(-50%,20px);opacity:0;background:#1b2a37;border:1px solid #34485c;border-radius:10px;padding:10px 16px;transition:.2s;pointer-events:none}.toast.show{opacity:1;transform:translate(-50%,0)}.footer{margin-top:18px;color:#65778a;font-size:11px;display:flex;justify-content:space-between}@media(max-width:760px){.metric{grid-column:span 6}.main,.side,.half{grid-column:span 12}.wrap{padding:18px 14px}.top{align-items:flex-start}.metric .value{font-size:29px}}
</style></head><body><main class="wrap">
<header class="top"><div class="brand"><div class="mark">↗</div><div><div class="eyebrow">Bench instrumentation</div><div class="title">Thrust Rig <span style="color:#7c91a6;font-weight:500">/ ESP32</span></div></div></div><div class="status"><i class="dot" id="dot"></i><span id="state">Connecting…</span></div></header>
<section class="grid">
<article class="card metric"><div class="label">Thrust</div><div class="value"><span id="thrust">—</span> <span class="unit">g</span></div><div class="sub" id="loadState">Load cell active</div></article>
<article class="card metric"><div class="label">Motor output</div><div class="value"><span id="throttle">0</span><span class="unit"> %</span></div><div class="sub" id="modeLabel">Idle</div></article>
<article class="card metric"><div class="label">Shaft speed</div><div class="value"><span id="rpm">—</span> <span class="unit">RPM</span></div><div class="sub" id="rpmState">Tachometer enabled</div></article>
<article class="card metric"><div class="label">Load cell</div><div class="value" style="font-size:21px;margin-top:15px" id="calStatus">Ready</div><div class="sub" id="calSub">Scale factor: —</div></article>
<article class="card main"><div class="cardhead"><div class="cardtitle">Thrust vs RPM</div><span class="badge">Live · last 120 samples</span></div><canvas id="scatter"></canvas></article>
<article class="card side"><div class="cardhead"><div class="cardtitle">Test control</div><span class="badge" id="runBadge">IDLE</span></div>
<div class="throttle-line"><span class="label">Manual throttle</span><strong><span id="sliderVal">0</span><span class="unit">%</span></strong></div><input id="slider" class="range" type="range" min="0" max="100" value="0">
<div class="controls"><button class="btn primary" id="setManual">Apply throttle</button><button class="btn" id="cancel">Stop test</button></div>
<div class="divider"></div><label for="holdPct">Hold throttle</label><div class="row"><input id="holdPct" type="number" min="0" max="100" value="30" aria-label="Hold throttle percentage"><input id="holdSec" type="number" min="1" max="3600" value="10" aria-label="Hold duration in seconds"><button class="btn primary" id="hold">Run</button></div><div class="sub" style="margin-top:5px">Throttle % · duration seconds</div>
<label for="sweepSec">Automatic sweep · 0 → 100%</label><div class="row"><input id="sweepSec" type="number" min="1" max="3600" value="60" aria-label="Sweep duration in seconds"><button class="btn primary" id="sweep">Start sweep</button></div>
<button class="btn stop full" id="estop">EMERGENCY STOP</button><button class="btn full" id="resetEstop">Reset emergency stop</button></article>
<article class="card half smallchart"><div class="cardhead"><div class="cardtitle">Thrust over time</div><span class="badge">Live</span></div><canvas id="timeline"></canvas></article>
<article class="card half"><div class="cardhead"><div class="cardtitle">Sensor setup</div><span class="badge" id="sensorBadge">HX711</span></div>
<div class="switchrow"><span>Use load cell readings</span><input type="checkbox" class="switch" id="loadToggle" checked></div>
<div class="controls"><button class="btn primary" id="tare">Zero / tare</button><button class="btn" id="clearGraph">Clear graphs</button></div>
<div class="divider"></div><label for="knownWeight">Known calibration mass</label><div class="row"><input type="number" id="knownWeight" min="0.1" step="0.1" value="100" aria-label="Known calibration mass in grams"><span class="badge">grams</span><button class="btn primary" id="calibrate">Calibrate</button></div><div class="note">Place the entered mass on the load cell, then select Calibrate. Zero the unloaded sensor first. Calibration is stored in RAM and resets when the ESP32 restarts.</div>
</article></section><div class="footer"><span>Local dashboard · connect to Wi-Fi “Thrust-Rig”</span><span>Safety: keep clear of propellers during tests</span></div></main><div class="toast" id="toast"></div>
<script>
const $=id=>document.getElementById(id);let samples=[],timeSamples=[],busy=false;
function toast(s){let t=$('toast');t.textContent=s;t.classList.add('show');setTimeout(()=>t.classList.remove('show'),2400)}
async function api(path,body){let opt=body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams(body)};let r=await fetch(path,opt);let j=await r.json();if(!r.ok)throw Error(j.error||'Request failed');return j}
async function action(path,body,msg){try{await api(path,body);if(msg)toast(msg)}catch(e){toast(e.message)}}
function axes(ctx,w,h,xLabel,yLabel){ctx.strokeStyle='#2a3948';ctx.lineWidth=1;ctx.fillStyle='#8393a5';ctx.font='10px system-ui';for(let i=0;i<=4;i++){let y=10+(h-36)*i/4;ctx.beginPath();ctx.moveTo(38,y);ctx.lineTo(w-8,y);ctx.stroke();ctx.fillText(String(Math.round((4-i)*100)),4,y+3)}ctx.fillText(yLabel,5,9);ctx.fillText(xLabel,w-75,h-5)}
function chart(id,data,mode){let c=$(id),dpr=window.devicePixelRatio||1,rect=c.getBoundingClientRect();c.width=rect.width*dpr;c.height=rect.height*dpr;let x=c.getContext('2d');x.scale(dpr,dpr);let w=rect.width,h=rect.height;axes(x,w,h,mode==='rpm'?'RPM':'Time','g');if(!data.length)return;let maxX=mode==='rpm'?Math.max(100,...data.map(p=>p.x)):Math.max(1,data.length-1),maxY=Math.max(10,...data.map(p=>p.y))*1.15;x.strokeStyle='#42d5c5';x.lineWidth=2;x.beginPath();data.forEach((p,i)=>{let px=38+(w-48)*Math.max(0,p.x)/maxX,py=10+(h-46)*(1-p.y/maxY);if(i)x.lineTo(px,py);else x.moveTo(px,py)});x.stroke();x.fillStyle='#64e2d2';data.forEach(p=>{let px=38+(w-48)*Math.max(0,p.x)/maxX,py=10+(h-46)*(1-p.y/maxY);x.beginPath();x.arc(px,py,2.6,0,Math.PI*2);x.fill()})}
function draw(){chart('scatter',samples,'rpm');chart('timeline',timeSamples,'time')}
async function refresh(){try{let s=await api('/api/state');$('state').textContent=s.estop?'E-STOP LATCHED':s.mode==='idle'?'Ready':s.mode.toUpperCase()+' RUNNING';$('dot').classList.toggle('red',s.estop);$('thrust').textContent=s.loadEnabled?Number(s.thrust).toFixed(1):'OFF';$('loadState').textContent=s.loadEnabled?'HX711 active':'Load cell ignored';$('throttle').textContent=Number(s.throttle).toFixed(0);$('modeLabel').textContent=s.mode==='idle'?'Idle':s.mode; $('rpm').textContent=s.tachEnabled?Math.round(s.rpm).toLocaleString():'—';$('rpmState').textContent=s.tachEnabled?'Tachometer enabled · GPIO 27':'Tachometer disabled';$('runBadge').textContent=s.estop?'E-STOP':s.mode.toUpperCase();$('calStatus').textContent=s.calibrated?'Calibrated':'Needs calibration';$('calSub').textContent='Scale factor: '+Number(s.scale).toPrecision(4);$('loadToggle').checked=s.loadEnabled;$('slider').value=s.throttle;$('sliderVal').textContent=s.throttle;if(s.loadEnabled){if(s.tareCount!==undefined){samples.push({x:s.rpm,y:s.thrust});timeSamples.push({x:timeSamples.length,y:s.thrust});if(samples.length>120)samples.shift();if(timeSamples.length>120)timeSamples.shift();draw()}}}catch(e){$('state').textContent='Disconnected';$('dot').classList.add('red')}}
$('slider').oninput=e=>$('sliderVal').textContent=e.target.value;$('setManual').onclick=()=>action('/api/manual',{throttle:$('slider').value},'Manual throttle applied');$('hold').onclick=()=>action('/api/hold',{throttle:$('holdPct').value,seconds:$('holdSec').value},'Hold test started');$('sweep').onclick=()=>action('/api/sweep',{seconds:$('sweepSec').value},$('sweepSec').value+'-second sweep started');$('cancel').onclick=()=>action('/api/stop',{},'Test stopped');$('estop').onclick=()=>action('/api/estop',{},'Emergency stop latched');$('resetEstop').onclick=()=>action('/api/reset-estop',{},'Emergency stop reset');$('tare').onclick=()=>action('/api/tare',{},'Load cell zeroed');$('calibrate').onclick=()=>action('/api/calibrate',{grams:$('knownWeight').value},'Calibration updated');$('loadToggle').onchange=e=>action('/api/loadcell',{enabled:e.target.checked?1:0});$('clearGraph').onclick=()=>{samples=[];timeSamples=[];draw()};window.onresize=draw;setInterval(refresh,250);refresh();
</script></body></html>
)HTML";

void IRAM_ATTR onTachPulse() {
  portENTER_CRITICAL_ISR(&tachMux);
  ++tachPulses;
  portEXIT_CRITICAL_ISR(&tachMux);
}

void setThrottle(float value) {
  throttlePercent = constrain(value, 0.0f, 100.0f);
  uint32_t duty = (uint32_t)roundf(throttlePercent * 255.0f / 100.0f);
  ledcWrite(MOTOR_PWM_PIN, duty);
}

void resetThrustFilter() {
  thrustSampleCount = 0;
  thrustSampleIndex = 0;
  thrustFilterInitialized = false;
  thrustGrams = 0;
  latestRaw = 0;
}

float medianThrustSample() {
  float sorted[THRUST_FILTER_WINDOW];
  for (uint8_t i = 0; i < thrustSampleCount; ++i) sorted[i] = thrustSamples[i];
  for (uint8_t i = 1; i < thrustSampleCount; ++i) {
    float value = sorted[i];
    int8_t j = i - 1;
    while (j >= 0 && sorted[j] > value) {
      sorted[j + 1] = sorted[j];
      --j;
    }
    sorted[j + 1] = value;
  }
  if (thrustSampleCount & 1) return sorted[thrustSampleCount / 2];
  return (sorted[thrustSampleCount / 2 - 1] + sorted[thrustSampleCount / 2]) * 0.5f;
}

void updateThrustFilter(float sample) {
  thrustSamples[thrustSampleIndex] = sample;
  thrustSampleIndex = (thrustSampleIndex + 1) % THRUST_FILTER_WINDOW;
  if (thrustSampleCount < THRUST_FILTER_WINDOW) ++thrustSampleCount;

  float median = medianThrustSample();
  if (!thrustFilterInitialized) {
    thrustGrams = median;
    thrustFilterInitialized = true;
  } else {
    thrustGrams += THRUST_EMA_ALPHA * (median - thrustGrams);
  }
}

void stopRun(bool latch) {
  setThrottle(0);
  runMode = "idle";
  if (latch) emergencyLatched = true;
}

String jsonState() {
  String s = "{\"thrust\":" + String(thrustGrams, 2) +
    ",\"raw\":" + String(latestRaw, 1) +
    ",\"rpm\":" + String(rpm, 1) +
    ",\"throttle\":" + String(throttlePercent, 1) +
    ",\"mode\":\"" + runMode + "\",\"estop\":" + String(emergencyLatched ? "true" : "false") +
    ",\"loadEnabled\":" + String(loadCellEnabled ? "true" : "false") +
    ",\"tachEnabled\":" + String(TACH_ENABLED ? "true" : "false") +
    ",\"scale\":" + String(scaleFactor, 6) +
    ",\"calibrated\":" + String(fabsf(scaleFactor - 1.0f) > 0.0001f ? "true" : "false") +
    ",\"tareCount\":" + String((uint32_t)millis()) + "}";
  return s;
}

void sendJson(int status, const String &body) {
  server.send(status, "application/json", body);
}

bool parseNumber(const char *key, float &out) {
  if (!server.hasArg(key)) return false;
  out = server.arg(key).toFloat();
  return isfinite(out);
}

bool rejectIfLatched() {
  if (!emergencyLatched) return false;
  sendJson(423, "{\"error\":\"Emergency stop is latched. Reset it before running.\"}");
  return true;
}

void setupRoutes() {
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", DASHBOARD_HTML); });
  server.on("/api/state", HTTP_GET, []() {
    lastHeartbeatMillis = millis();
    sendJson(200, jsonState());
  });
  server.on("/api/manual", HTTP_POST, []() {
    if (rejectIfLatched()) return;
    float p;
    if (!parseNumber("throttle", p) || p < 0 || p > 100) { sendJson(400, "{\"error\":\"Throttle must be 0–100%.\"}"); return; }
    runMode = "manual"; runDurationMillis = 0; setThrottle(p);
    sendJson(200, "{\"ok\":true}");
  });
  server.on("/api/hold", HTTP_POST, []() {
    if (rejectIfLatched()) return;
    float p, sec;
    if (!parseNumber("throttle", p) || !parseNumber("seconds", sec) || p < 0 || p > 100 || sec < 1 || sec > 3600) { sendJson(400, "{\"error\":\"Enter throttle 0–100% and duration 1–3600 seconds.\"}"); return; }
    runMode = "hold"; runStartedMillis = millis(); runDurationMillis = (uint32_t)(sec * 1000); runEndThrottle = p; setThrottle(p); lastHeartbeatMillis = millis();
    sendJson(200, "{\"ok\":true}");
  });
  server.on("/api/sweep", HTTP_POST, []() {
    if (rejectIfLatched()) return;
    float sec;
    if (!parseNumber("seconds", sec) || sec < 1 || sec > 3600) { sendJson(400, "{\"error\":\"Duration must be 1–3600 seconds.\"}"); return; }
    runMode = "sweep"; runStartedMillis = millis(); runDurationMillis = (uint32_t)(sec * 1000); runStartThrottle = 0; runEndThrottle = 100; setThrottle(0); lastHeartbeatMillis = millis();
    sendJson(200, "{\"ok\":true}");
  });
  server.on("/api/stop", HTTP_POST, []() { stopRun(false); sendJson(200, "{\"ok\":true}"); });
  server.on("/api/estop", HTTP_POST, []() { stopRun(true); sendJson(200, "{\"ok\":true}"); });
  server.on("/api/reset-estop", HTTP_POST, []() { emergencyLatched = false; sendJson(200, "{\"ok\":true}"); });
  server.on("/api/tare", HTTP_POST, []() {
    if (!loadCell.is_ready()) { sendJson(503, "{\"error\":\"HX711 is not ready. Check power and wiring.\"}"); return; }
    loadCell.tare(10); resetThrustFilter(); sendJson(200, "{\"ok\":true}");
  });
  server.on("/api/calibrate", HTTP_POST, []() {
    float grams;
    if (!parseNumber("grams", grams) || grams <= 0 || grams > 100000) { sendJson(400, "{\"error\":\"Known mass must be greater than zero grams.\"}"); return; }
    if (!loadCell.is_ready()) { sendJson(503, "{\"error\":\"HX711 is not ready. Check power and wiring.\"}"); return; }
    float counts = (float)loadCell.get_value(12);
    if (fabsf(counts) < 1.0f) { sendJson(400, "{\"error\":\"No load change detected. Place the known mass on the sensor.\"}"); return; }
    scaleFactor = counts / grams;
    loadCell.set_scale(scaleFactor);
    resetThrustFilter();
    sendJson(200, "{\"ok\":true}");
  });
  server.on("/api/loadcell", HTTP_POST, []() {
    loadCellEnabled = server.arg("enabled") == "1";
    if (!loadCellEnabled) thrustGrams = 0;
    sendJson(200, "{\"ok\":true}");
  });
  server.onNotFound([]() { sendJson(404, "{\"error\":\"Not found\"}"); });
}

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  ledcAttach(MOTOR_PWM_PIN, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS); // Arduino-ESP32 3.x
  setThrottle(0);

  loadCell.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
  loadCell.set_scale(scaleFactor);
  if (loadCell.wait_ready_timeout(2000)) loadCell.tare(10);
  else Serial.println("HX711 not ready; dashboard will report the sensor error when tare/calibrate is requested.");

  if (TACH_ENABLED) {
    pinMode(TACH_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(TACH_PIN), onTachPulse, FALLING);
  }
  previousRpmMillis = millis();
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("Dashboard: http://"); Serial.println(WiFi.softAPIP());
  Serial.print("Wi-Fi: "); Serial.println(AP_SSID);
  setupRoutes();
  server.begin();
  lastHeartbeatMillis = millis();
}

void loop() {
  server.handleClient();
  uint32_t now = millis();

  if (TACH_ENABLED && now - previousRpmMillis >= 1000) {
    uint32_t count;
    portENTER_CRITICAL(&tachMux); count = tachPulses; portEXIT_CRITICAL(&tachMux);
    uint32_t delta = count - previousTachPulses;
    rpm = (delta * 60000.0f) / (TACH_PULSES_PER_REV * (now - previousRpmMillis));
    previousTachPulses = count; previousRpmMillis = now;
  }

  if (runMode != "idle") {
    if (now - lastHeartbeatMillis > HEARTBEAT_TIMEOUT_MS) {
      stopRun(true); // Browser connection lost while a motor command was active.
    } else if (runMode == "sweep" || runMode == "hold") {
      uint32_t elapsed = now - runStartedMillis;
      if (elapsed >= runDurationMillis) {
        setThrottle(0);
        runMode = "idle";
      } else if (runMode == "sweep") {
        float fraction = (float)elapsed / runDurationMillis;
        setThrottle(runStartThrottle + (runEndThrottle - runStartThrottle) * fraction);
      }
    }
  }

  if (now - lastSampleMillis >= SAMPLE_PERIOD_MS) {
    lastSampleMillis = now;
    if (loadCellEnabled && loadCell.is_ready()) {
      latestRaw = (float)loadCell.get_value(3);
      updateThrustFilter(loadCell.get_units(1));
    }
  }
}
