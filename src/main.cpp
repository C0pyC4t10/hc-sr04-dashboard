#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <NewPingESP8266.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>

#define TRIG_PIN 14
#define ECHO_PIN 12
#define MAX_DISTANCE 400
#define ALPHA 0.3
#define MAX_RATE_OF_CHANGE 50.0

const char* ssid = "Skarbol Tech Ltd";
const char* password = "skarbol.tech@soft";

NewPingESP8266 sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
ESP8266WebServer server(80);

float filteredDist = 0;
float lastRaw = 0;
float minDist = 999;
float maxDist = 0;
float sumDist = 0;
int readingCount = 0;
int successCount = 0;
bool firstReading = true;
bool paused = false;

WiFiClient mqttWiFiClient;
PubSubClient mqttClient(mqttWiFiClient);
unsigned long mqttReconnect = 0;

float lastNear = -1;
float lastMid = -1;
float lastFar = -1;
unsigned long lastDetectTime = 0;
String currentZone = "clear";

String getZoneName(float d) {
  if (d < 2 || d > MAX_DISTANCE) return "clear";
  if (d < 50) return "near";
  if (d < 150) return "mid";
  return "far";
}

float applyEMA(float raw) {
  if (firstReading) { firstReading = false; filteredDist = raw; return filteredDist; }
  filteredDist = ALPHA * raw + (1 - ALPHA) * filteredDist;
  return filteredDist;
}

bool validateReading(float raw) {
  if (raw < 2 || raw > MAX_DISTANCE) return false;
  if (!firstReading && fabs(raw - lastRaw) > MAX_RATE_OF_CHANGE) return false;
  return true;
}

void measureDistance() {
  if (paused) return;
  unsigned int echoTime = sonar.ping_median(3);
  readingCount++;
  if (echoTime == 0) return;

  float raw = sonar.convert_cm(echoTime);
  if (!validateReading(raw)) return;

  successCount++;
  lastRaw = raw;
  applyEMA(raw);
  if (raw < minDist) minDist = raw;
  if (raw > maxDist) maxDist = raw;
  sumDist += raw;

  String zone = getZoneName(raw);
  currentZone = zone;
  lastDetectTime = millis();
  if (zone == "near") lastNear = raw;
  else if (zone == "mid") lastMid = raw;
  else if (zone == "far") lastFar = raw;
}

void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>HC-SR04 Distance</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:Arial,sans-serif;background:#111;color:#fff;display:flex;justify-content:center;min-height:100vh;padding:20px}
.wrap{text-align:center;width:100%;max-width:480px}
.c{background:#1a1a2e;border-radius:16px;padding:30px;margin-bottom:16px}
h1{color:#4a6fa5;font-size:0.8rem;letter-spacing:2px;text-transform:uppercase;margin-bottom:20px}
.n{font-size:5rem;font-weight:bold;line-height:1;transition:color 0.3s}
.ok{color:#00d2ff}
.warn{color:#ffd700}
.danger{color:#ff4757}
.ux{color:#4a6fa5;font-size:2rem}
.bb{background:#0f3460;border-radius:13px;height:24px;overflow:hidden;margin:12px 0 8px}
.bf{height:100%;border-radius:13px;transition:width 0.3s;width:0%}
.bf.ok{background:#00d2ff}
.bf.warn{background:#ffd700}
.bf.danger{background:#ff4757}
.bq{display:flex;justify-content:space-between;color:#4a6fa5;font-size:0.7rem}
#cv{background:#0f3460;border-radius:12px;display:block;width:100%;height:auto}
.st{color:#4a6fa5;font-size:0.8rem;margin-top:16px}
.dt{display:inline-block;width:8px;height:8px;border-radius:50%;background:#00d2ff;margin-right:6px}
.btn{background:#0f3460;border:none;color:#fff;font-size:1rem;padding:12px 0;border-radius:12px;cursor:pointer;width:100%;transition:background 0.2s;margin-bottom:4px}
.btn:hover{background:#1a4a7a}
.btn.r{color:#00d2ff}
.btn.s{color:#ff4757}
.g{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-top:14px}
.bx{background:#0f3460;border-radius:12px;padding:12px}
.bv{font-size:1.3rem;font-weight:bold;color:#00d2ff}
.bl{font-size:0.65rem;color:#4a6fa5;text-transform:uppercase}
.zr{display:flex;justify-content:center;gap:6px;margin:12px 0 6px}
.zb{background:#0f3460;border-radius:8px;padding:6px 14px;font-size:0.75rem;color:#4a6fa5;border:1px solid transparent;transition:all 0.3s}
.zb.act{background:transparent}
.zb.act.n{color:#ff4757;border-color:#ff4757}
.zb.act.m{color:#ffd700;border-color:#ffd700}
.zb.act.f{color:#00d2ff;border-color:#00d2ff}
.zv{display:flex;justify-content:center;gap:6px;font-size:0.7rem;color:#4a6fa5}
.zv span{width:33%}
.pd{display:flex;align-items:center;justify-content:center;gap:8px;margin-top:10px;font-size:0.75rem;color:#4a6fa5;text-transform:uppercase;letter-spacing:1px}
.pd-dot{width:10px;height:10px;border-radius:50%;display:inline-block}
.pd-dot.y{background:#00d2ff;box-shadow:0 0 8px #00d2ff}
.pd-dot.n{background:#4a6fa5}
@media(max-width:500px){.n{font-size:3.5rem}}
</style>
</head>
<body>
<div class="wrap">
<h1 style="margin-bottom:16px">HC-SR04 Distance Sensor</h1>
<div class="c">
<div class="n ok" id="d">--<span class="ux"> cm</span></div>
<div class="bq"><span>0</span><span id="z">--</span><span>400</span></div>
<div class="bb"><div class="bf ok" id="b"></div></div>
</div>
<div class="c">
<div class="zr">
<div class="zb" id="zNear">🔴 NEAR</div>
<div class="zb" id="zMid">🟡 MID</div>
<div class="zb" id="zFar">🔵 FAR</div>
</div>
<div class="zv"><span id="vNear">--</span><span id="vMid">--</span><span id="vFar">--</span></div>
<div class="pd"><span class="pd-dot n" id="pdDot"></span><span id="pdText">No object detected</span></div>
<button class="btn s" id="btn" onclick="fetch('/toggle')" style="margin-top:14px">⏸ PAUSE</button>
<a href="https://thingspeak.com/channels/3388472" target="_blank" class="btn r" style="text-decoration:none;display:block;margin-top:8px">THINGSPEAK</a>
</div>
<div class="c">
<div class="g">
<div class="bx"><div class="bv" id="mn">--</div><div class="bl">Min</div></div>
<div class="bx"><div class="bv" id="av">--</div><div class="bl">Avg</div></div>
<div class="bx"><div class="bv" id="mx">--</div><div class="bl">Max</div></div>
</div>
</div>
<div class="c" style="padding:15px">
<canvas id="cv"></canvas>
</div>
<div class="st"><span class="dt" id="dot"></span><span id="m">Starting...</span></div>
</div>
<script>
(function(){
var cv=document.getElementById('cv'),cx=cv.getContext('2d');
var W=cv.width=cv.clientWidth*2,H=cv.height=240;
var rh=[],fh=[],mn=Infinity,mx=-Infinity,su=0,ct=0, paused=false, lastV=0;
function dr(){
cx.clearRect(0,0,W,H);if(fh.length<2)return;
var hi=Math.max.apply(null,fh.concat(rh).concat([50]));
var lo=Math.min.apply(null,fh.concat(rh).concat([0]));
var ra=hi-lo||1;
var py=function(v){return H-(v-lo)/ra*(H-20)-10};
function li(a,w,s){cx.strokeStyle=s;cx.lineWidth=w;cx.beginPath();for(var i=1;i<a.length;i++){var xi=i/(a.length-1)*W,yi=py(a[i]);i==1?cx.moveTo(xi,yi):cx.lineTo(xi,yi)}cx.stroke()}
li(rh,2,'rgba(58,122,213,0.4)');li(fh,3,'#00d2ff');
cx.fillStyle='rgba(0,210,255,0.08)';cx.lineTo(W,H);cx.lineTo(0,H);cx.closePath();cx.fill();
cx.fillStyle='#4a6fa5';cx.font='16px Arial';cx.textAlign='right';
cx.fillText(Math.round(hi)+'cm',W-8,22);cx.fillText(Math.round(lo)+'cm',W-8,H-6);
}
function up(){
var x=new XMLHttpRequest();x.open('GET','/distance',true);
x.onload=function(){
try{
var d=JSON.parse(x.responseText);if(d.status!=='ok'){document.getElementById('m').textContent=d.message;return}
paused=d.paused;var v=parseFloat(d.distance);if(!isNaN(v))lastV=v;
var e=document.getElementById('d'),btn=document.getElementById('btn'),dot=document.getElementById('dot');
if(paused){
btn.innerHTML='\u25B6 RESUME';btn.className='btn r';dot.style.background='#ff4757';dot.style.animation='none';
e.innerHTML=lastV.toFixed(1)+'<span class="ux"> cm</span>';
document.getElementById('m').textContent='PAUSED (last reading) | Uptime: '+d.time;return;
}else{
dot.style.background='#00d2ff';dot.style.animation='p 1.5s infinite';
btn.innerHTML='\u23F8 PAUSE';btn.className='btn s';
}
e.innerHTML=v.toFixed(1)+'<span class="ux"> cm</span>';
var z='ok',l='IN RANGE';if(v<10){z='danger';l='TOO CLOSE'}else if(v<50){z='warn';l='NEAR'}else if(v>300){z='danger';l='TOO FAR'}
e.className='n '+z;document.getElementById('b').style.width=Math.min(v/4,100)+'%';
document.getElementById('b').className='bf '+z;document.getElementById('z').textContent=l;
rh.push(parseFloat(d.raw));fh.push(parseFloat(d.distance));
if(rh.length>40){rh.shift();fh.shift()}
var vm=parseFloat(d.distance);if(vm<mn)mn=vm;if(vm>mx)mx=vm;su+=vm;ct++;
document.getElementById('mn').textContent=mn.toFixed(1);document.getElementById('mx').textContent=mx.toFixed(1);
document.getElementById('av').textContent=(su/ct).toFixed(1);dr();

var zone=d.zone||'clear',present=d.present||false;
var zn=document.getElementById('zNear'),zm=document.getElementById('zMid'),zf=document.getElementById('zFar');
zn.className='zb';zm.className='zb';zf.className='zb';
if(zone==='near')zn.className='zb act n';else if(zone==='mid')zm.className='zb act m';else if(zone==='far')zf.className='zb act f';
document.getElementById('vNear').textContent=d.lastNear>0?d.lastNear.toFixed(1)+'cm':'--';
document.getElementById('vMid').textContent=d.lastMid>0?d.lastMid.toFixed(1)+'cm':'--';
document.getElementById('vFar').textContent=d.lastFar>0?d.lastFar.toFixed(1)+'cm':'--';
var pd=document.getElementById('pdDot'),pt=document.getElementById('pdText');
if(present){pd.className='pd-dot y';pt.textContent='Object detected in '+zone.toUpperCase()+' zone'}
else{pd.className='pd-dot n';pt.textContent='No object detected'}

document.getElementById('m').textContent='Running | Uptime: '+d.time;
}catch(e){}
};
x.onerror=function(){document.getElementById('m').textContent='Connecting...'};
setTimeout(function(){x.send()},10);
}
setInterval(up,500);setTimeout(up,100);
})();
</script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

void handleDistance() {
  float d = firstReading ? -1 : filteredDist;
  String json;
  if (d < 0) {
    json = "{\"status\":\"waiting\",\"message\":\"First reading pending\",\"paused\":";
    json += paused ? "true" : "false";
    json += "}";
  } else {
    unsigned long t = millis() / 1000;
    String ts = String(t / 60) + "m " + String(t % 60) + "s";
    int q = (readingCount > 0) ? (successCount * 100 / readingCount) : 0;
    bool present = (t - lastDetectTime / 1000) < 3;
    json = "{\"status\":\"ok\",\"distance\":" + String(d, 1);
    json += ",\"raw\":" + String(lastRaw, 1);
    json += ",\"min\":" + String(minDist, 1);
    json += ",\"max\":" + String(maxDist, 1);
    json += ",\"quality\":" + String(q);
    json += ",\"paused\":" + String(paused ? "true" : "false");
    json += ",\"zone\":\"" + currentZone + "\"";
    json += ",\"present\":" + String(present ? "true" : "false");
    json += ",\"lastNear\":" + String(lastNear, 1);
    json += ",\"lastMid\":" + String(lastMid, 1);
    json += ",\"lastFar\":" + String(lastFar, 1);
    json += ",\"time\":\"" + ts + "\"}";
  }
  server.send(200, "application/json", json);
}

void handleToggle() {
  paused = !paused;
  String json = "{\"paused\":" + String(paused ? "true" : "false") + "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting");
  int a = 0;
  while (WiFi.status() != WL_CONNECTED && a < 40) {
    delay(500);
    Serial.print(".");
    a++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi failed. Starting AP...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("HC-SR04-Distance");
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
  }
  server.on("/", handleRoot);
  server.on("/distance", handleDistance);
  server.on("/toggle", handleToggle);
  mqttClient.setServer("broker.hivemq.com", 1883);
  mqttClient.setKeepAlive(30);
  if (MDNS.begin("hc-sr04")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("mDNS: http://hc-sr04.local");
  }
  server.begin();
  Serial.println("Server started");
}

void loop() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last >= 200) {
    measureDistance();
    last = now;
  }
  if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();

  if (!mqttClient.connected() && now - mqttReconnect > 5000) {
    mqttReconnect = now;
    if (mqttClient.connect("HC-SR04-ESP")) {
      Serial.println("MQTT: connected");
    }
  }
  mqttClient.loop();

  static unsigned long pubLast = 0;
  if (!paused && readingCount > 0 && now - pubLast >= 2000) {
    pubLast = now;
    int q = (readingCount > 0) ? (successCount * 100 / readingCount) : 0;
    String payload = "{\"distance\":" + String(filteredDist, 1) +
                     ",\"raw\":" + String(lastRaw, 1) +
                     ",\"zone\":\"" + currentZone + "\"" +
                     ",\"min\":" + String(minDist, 1) +
                     ",\"max\":" + String(maxDist, 1) +
                     ",\"quality\":" + String(q) + "}";
    mqttClient.publish("hc-sr04/distance", payload.c_str());
  }

  server.handleClient();
}
