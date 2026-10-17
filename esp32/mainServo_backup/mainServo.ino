/*
  ==============================================================
  KONTROL ROBOT 4 RODA (2 DRIVER BTS7960) + GRIPPER (2 SERVO)
  WEB REMOTE VIA ESP32-S3
  Mode: Access Point (AP) + WebSocket
  IP Default: 192.168.4.1
  Library yang Dibutuhkan:
    - WebSockets by Markus Sattler (install via Library Manager)
    - ESP32Servo
  ==============================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>

// ================= KONFIGURASI WIFI AP =================
const char* AP_SSID = "Robot-Remote";
const char* AP_PASS = "1234567890"; // Minimal 8 karakter

// ================= PIN MOTOR BTS7960 =================
// Index 0: Sisi Kiri (Depan & Belakang paralel)
// Index 1: Sisi Kanan (Depan & Belakang paralel)
const int rpwmPins[] = {17, 6};  // {RPWM Kiri, RPWM Kanan}
const int lpwmPins[] = {18, 7};  // {LPWM Kiri, LPWM Kanan}
const int numDrivers = 2;

const int pwmFreq = 1000;
const int pwmResolution = 8;     // Rentang duty cycle 0 - 255

// Kecepatan default
int kecepatanAktif = 130;

// ================= GRIPPER (SERVO) =================
Servo servoLift;     // Servo pengangkat (wrist)
Servo servoGripper;  // Servo penjepit

const int pinLift = 4;     // diganti dari 18 (bentrok dengan LPWM kiri)
const int pinGripper = 5;  // diganti dari 19 (GPIO19 = USB di ESP32-S3)

int liftAngle = 90;
int gripAngle = 30;

const int LIFT_MIN = 45;
const int LIFT_MAX = 120;
const int GRIP_MIN = 30;   // terbuka
const int GRIP_MAX = 90;   // menjepit

// Failsafe & Heartbeat
unsigned long lastCommandTime = 0;
const unsigned long TIMEOUT_FAILSAFE = 400; // ms (berhenti jika koneksi putus/hilang sinyal)
bool isMoving = false;

WebServer server(80);
WebSocketsServer webSocket(81);

// ================= HTML, CSS & JAVASCRIPT UI =================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="id">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>Eco-Robot Remote</title>
  <style>
    :root {
      --bg: #070b18;
      --card: rgba(30, 41, 59, 0.65);
      --border: rgba(148, 163, 184, 0.18);
      --text: #f8fafc;
      --subtext: #94a3b8;
      --accent: #38bdf8;
      --accent2: #a78bfa;
      --danger: #ef4444;
      --success: #22c55e;
      --warn: #f59e0b;
    }
    * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
    body {
      margin: 0;
      padding: 16px;
      background:
        radial-gradient(circle at 15% 0%, rgba(56,189,248,0.18), transparent 45%),
        radial-gradient(circle at 90% 100%, rgba(167,139,250,0.18), transparent 45%),
        var(--bg);
      color: var(--text);
      font-family: system-ui, -apple-system, sans-serif;
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 14px;
      user-select: none;
      touch-action: manipulation;
    }
    header {
      width: 100%;
      max-width: 420px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding-bottom: 12px;
      border-bottom: 1px solid var(--border);
    }
    .title {
      font-weight: 800;
      font-size: 1.15rem;
      letter-spacing: 0.5px;
      background: linear-gradient(90deg, var(--accent), var(--accent2));
      -webkit-background-clip: text;
      background-clip: text;
      color: transparent;
    }
    .title small {
      display: block;
      font-weight: 500;
      font-size: 0.7rem;
      color: var(--subtext);
      -webkit-text-fill-color: var(--subtext);
      letter-spacing: 1.5px;
      text-transform: uppercase;
    }
    .status-badge {
      display: flex;
      align-items: center;
      gap: 6px;
      font-size: 0.82rem;
      color: var(--subtext);
      padding: 6px 12px;
      border-radius: 999px;
      background: var(--card);
      border: 1px solid var(--border);
    }
    .dot {
      width: 9px;
      height: 9px;
      border-radius: 50%;
      background: var(--danger);
      box-shadow: 0 0 8px var(--danger);
    }
    .dot.connected { background: var(--success); box-shadow: 0 0 8px var(--success); }

    .section-label {
      width: 100%;
      max-width: 420px;
      font-size: 0.7rem;
      letter-spacing: 2px;
      text-transform: uppercase;
      color: var(--subtext);
      margin-bottom: -6px;
    }

    /* D-PAD GRID */
    .dpad-container {
      display: grid;
      grid-template-columns: repeat(3, 80px);
      grid-template-rows: repeat(3, 80px);
      gap: 12px;
      justify-content: center;
      align-items: center;
      padding: 16px;
      border-radius: 28px;
      background: var(--card);
      border: 1px solid var(--border);
      backdrop-filter: blur(8px);
    }
    .btn {
      background: linear-gradient(160deg, #26344d, #1a2438);
      border: 1px solid var(--border);
      border-radius: 18px;
      color: var(--text);
      font-size: 1.4rem;
      font-weight: 600;
      display: flex;
      justify-content: center;
      align-items: center;
      cursor: pointer;
      box-shadow: 0 6px 12px -2px rgba(0,0,0,0.45), inset 0 1px 0 rgba(255,255,255,0.06);
      transition: all 0.08s ease;
    }
    .btn:active, .btn.pressed {
      background: linear-gradient(160deg, var(--accent), #0ea5e9);
      color: #0f172a;
      transform: scale(0.94);
      box-shadow: 0 0 18px rgba(56,189,248,0.6);
    }
    .btn-stop {
      background: linear-gradient(160deg, #f87171, var(--danger));
      border-color: var(--danger);
      font-size: 0.95rem;
      font-weight: bold;
      box-shadow: 0 0 14px rgba(239,68,68,0.35);
    }
    .btn-stop:active {
      background: #b91c1c;
      color: #fff;
    }

    /* PANELS */
    .panel {
      width: 100%;
      max-width: 420px;
      background: var(--card);
      border: 1px solid var(--border);
      border-radius: 20px;
      padding: 16px;
      backdrop-filter: blur(8px);
    }
    .panel h3 {
      margin: 0 0 10px 0;
      font-size: 0.95rem;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    .panel h3 b {
      color: var(--accent);
      font-variant-numeric: tabular-nums;
    }
    .slider-wrap { display: flex; flex-direction: column; gap: 8px; }
    .slider-info {
      display: flex;
      justify-content: space-between;
      font-size: 0.9rem;
      color: var(--subtext);
    }
    input[type=range] {
      width: 100%;
      height: 8px;
      border-radius: 4px;
      background: var(--border);
      outline: none;
      accent-color: var(--accent);
    }
    .row { display: flex; gap: 8px; margin-top: 12px; }
    .row .btn {
      flex: 1;
      height: 50px;
      font-size: 0.95rem;
      border-radius: 14px;
    }
    .btn-grip {
      background: linear-gradient(160deg, #c4b5fd, var(--accent2));
      color: #1e1b4b;
      border-color: var(--accent2);
    }
    .btn-release {
      background: linear-gradient(160deg, #fcd34d, var(--warn));
      color: #451a03;
      border-color: var(--warn);
    }
    .grip-state {
      text-align: center;
      font-size: 0.8rem;
      margin-top: 10px;
      color: var(--subtext);
    }
    .grip-state span { font-weight: 700; color: var(--accent2); }
  </style>
</head>
<body>
  <header>
    <div class="title">ECO-ROBOT<small>ESP32-S3 Remote</small></div>
    <div class="status-badge">
      <div class="dot" id="dot"></div>
      <span id="status-text">Terputus</span>
    </div>
  </header>

  <div class="section-label">Navigasi</div>
  <div class="dpad-container">
    <div></div>
    <button class="btn" id="btn-up" data-cmd="FORWARD">&#9650;</button>
    <div></div>

    <button class="btn" id="btn-left" data-cmd="LEFT">&#9664;</button>
    <button class="btn btn-stop" id="btn-stop" data-cmd="STOP">STOP</button>
    <button class="btn" id="btn-right" data-cmd="RIGHT">&#9654;</button>

    <div></div>
    <button class="btn" id="btn-down" data-cmd="BACKWARD">&#9660;</button>
    <div></div>
  </div>

  <div class="panel">
    <div class="slider-wrap">
      <div class="slider-info">
        <span>Kecepatan Motor</span>
        <b id="speed-val">130</b>
      </div>
      <input type="range" id="speed-slider" min="60" max="255" value="130" step="5">
    </div>
  </div>

  <div class="section-label">Gripper</div>

  <div class="panel">
    <h3><span>&#8693; Angkat / Turun</span><b><span id="lift-val">90</span>&deg;</b></h3>
    <input type="range" id="lift-slider" min="45" max="120" value="90" step="1">
    <div class="row">
      <button class="btn" onclick="setServo('lift', 45)">Turun</button>
      <button class="btn" onclick="setServo('lift', 90)">Tengah</button>
      <button class="btn" onclick="setServo('lift', 120)">Naik</button>
    </div>
  </div>

  <div class="panel">
    <h3><span>&#9994; Penjepit</span><b><span id="grip-val">30</span>&deg;</b></h3>
    <input type="range" id="grip-slider" min="30" max="90" value="30" step="1">
    <div class="row">
      <button class="btn btn-release" onclick="setServo('grip', 30)">Lepas</button>
      <button class="btn btn-grip" onclick="setServo('grip', 90)">Jepit</button>
    </div>
    <div class="grip-state">Status: <span id="grip-state">Terbuka</span> &middot; menjepit akan bertahan sampai ditekan Lepas</div>
  </div>

  <script>
    let ws;
    let heartbeatTimer = null;
    let currentCmd = 'STOP';

    const dot = document.getElementById('dot');
    const statusText = document.getElementById('status-text');
    const speedSlider = document.getElementById('speed-slider');
    const speedVal = document.getElementById('speed-val');

    const liftSlider = document.getElementById('lift-slider');
    const liftVal = document.getElementById('lift-val');
    const gripSlider = document.getElementById('grip-slider');
    const gripVal = document.getElementById('grip-val');
    const gripState = document.getElementById('grip-state');

    function connectWS() {
      ws = new WebSocket('ws://' + window.location.hostname + ':81/');

      ws.onopen = () => {
        dot.classList.add('connected');
        statusText.innerText = 'Terhubung';
        sendCmd('STOP');
        sendSpeed(speedSlider.value);
        sendServo('lift', liftSlider.value);
        sendServo('grip', gripSlider.value);
      };

      ws.onclose = () => {
        dot.classList.remove('connected');
        statusText.innerText = 'Terputus';
        clearInterval(heartbeatTimer);
        setTimeout(connectWS, 1000);
      };
    }

    function sendCmd(cmd) {
      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send("CMD:" + cmd);
      }
    }

    function sendSpeed(val) {
      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send("SPD:" + val);
      }
    }

    // ===== Servo gripper =====
    function sendServo(part, val) {
      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send((part === 'lift' ? "LFT:" : "GRP:") + val);
      }
    }

    function updateGripText(val) {
      const v = parseInt(val);
      gripState.innerText = v <= 32 ? 'Terbuka' : (v >= 88 ? 'Menjepit (ditahan)' : 'Setengah');
    }

    function setServo(part, val) {
      if (part === 'lift') {
        liftSlider.value = val;
        liftVal.innerText = val;
      } else {
        gripSlider.value = val;
        gripVal.innerText = val;
        updateGripText(val);
      }
      sendServo(part, val);
    }

    // Throttle supaya slider tidak membanjiri WebSocket
    let lastServoSend = 0;
    function bindServoSlider(slider, label, part) {
      slider.addEventListener('input', (e) => {
        const val = e.target.value;
        label.innerText = val;
        if (part === 'grip') updateGripText(val);
        const now = Date.now();
        if (now - lastServoSend > 60) {
          lastServoSend = now;
          sendServo(part, val);
        }
      });
      // Kirim nilai akhir saat slider dilepas
      slider.addEventListener('change', (e) => sendServo(part, e.target.value));
    }
    bindServoSlider(liftSlider, liftVal, 'lift');
    bindServoSlider(gripSlider, gripVal, 'grip');

    // Touch & Mouse Handler untuk tombol arah (tahan untuk jalan, lepas untuk stop)
    const buttons = document.querySelectorAll('.btn[data-cmd]');
    buttons.forEach(b => {
      const cmd = b.getAttribute('data-cmd');

      const start = (e) => {
        e.preventDefault();
        b.classList.add('pressed');
        currentCmd = cmd;
        sendCmd(currentCmd);

        // Kirim heartbeat loop setiap 150ms selama ditekan agar robot tetap jalan
        if (heartbeatTimer) clearInterval(heartbeatTimer);
        heartbeatTimer = setInterval(() => {
          if (currentCmd !== 'STOP') sendCmd(currentCmd);
        }, 150);
      };

      const end = (e) => {
        e.preventDefault();
        b.classList.remove('pressed');
        if (cmd !== 'STOP') {
          currentCmd = 'STOP';
          sendCmd('STOP');
        }
        clearInterval(heartbeatTimer);
      };

      b.addEventListener('touchstart', start, {passive: false});
      b.addEventListener('touchend', end, {passive: false});
      b.addEventListener('mousedown', start);
      b.addEventListener('mouseup', end);
      b.addEventListener('mouseleave', end);
    });

    speedSlider.addEventListener('input', (e) => {
      const val = e.target.value;
      speedVal.innerText = val;
      sendSpeed(val);
    });

    window.addEventListener('load', connectWS);
  </script>
</body>
</html>
)rawliteral";

// ================= FUNGSI MOTOR =================
void aturGerak(int speedLeft, int speedRight, bool majuKiri, bool majuKanan) {
  // Sisi Kiri (Driver 0)
  if (speedLeft == 0) {
    ledcWrite(rpwmPins[0], 0);
    ledcWrite(lpwmPins[0], 0);
  } else if (majuKiri) {
    ledcWrite(rpwmPins[0], speedLeft);
    ledcWrite(lpwmPins[0], 0);
  } else {
    ledcWrite(rpwmPins[0], 0);
    ledcWrite(lpwmPins[0], speedLeft);
  }

  // Sisi Kanan (Driver 1)
  if (speedRight == 0) {
    ledcWrite(rpwmPins[1], 0);
    ledcWrite(lpwmPins[1], 0);
  } else if (majuKanan) {
    ledcWrite(rpwmPins[1], speedRight);
    ledcWrite(lpwmPins[1], 0);
  } else {
    ledcWrite(rpwmPins[1], 0);
    ledcWrite(lpwmPins[1], speedRight);
  }
}

void maju(int speed)      { aturGerak(speed, speed, true, true); }
void mundur(int speed)    { aturGerak(speed, speed, false, false); }
void belokKanan(int speed) { aturGerak(speed, speed, false, true); }  // Putar di tempat kiri
void belokKiri(int speed){ aturGerak(speed, speed, true, false); }  // Putar di tempat kanan
void berhenti()           { aturGerak(0, 0, true, true); }

// ================= WEBSOCKET HANDLER =================
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    String msg = "";
    for (size_t i = 0; i < length; i++) msg += (char)payload[i];

    lastCommandTime = millis(); // Refresh waktu failsafe

    if (msg.startsWith("CMD:")) {
      String cmd = msg.substring(4);

      if (cmd == "FORWARD") {
        maju(kecepatanAktif);
        isMoving = true;
      } else if (cmd == "BACKWARD") {
        mundur(kecepatanAktif);
        isMoving = true;
      } else if (cmd == "LEFT") {
        belokKiri(kecepatanAktif);
        isMoving = true;
      } else if (cmd == "RIGHT") {
        belokKanan(kecepatanAktif);
        isMoving = true;
      } else if (cmd == "STOP") {
        berhenti();
        isMoving = false;
      }
    } else if (msg.startsWith("SPD:")) {
      kecepatanAktif = msg.substring(4).toInt();
      kecepatanAktif = constrain(kecepatanAktif, 0, 255);
    }
    // ----- TAMBAHAN GRIPPER -----
    else if (msg.startsWith("LFT:")) {
      liftAngle = constrain(msg.substring(4).toInt(), LIFT_MIN, LIFT_MAX);
      servoLift.write(liftAngle);
    } else if (msg.startsWith("GRP:")) {
      gripAngle = constrain(msg.substring(4).toInt(), GRIP_MIN, GRIP_MAX);
      servoGripper.write(gripAngle);   // posisi ditahan sampai ada perintah lepas
    }
  } else if (type == WStype_DISCONNECTED) {
    berhenti();
    isMoving = false;
    // Servo sengaja tidak diubah: capit tetap menjepit walau koneksi putus
  }
}

// ================= SETUP & LOOP =================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Inisialisasi PWM BTS7960
  for (int i = 0; i < numDrivers; i++) {
    ledcAttach(rpwmPins[i], pwmFreq, pwmResolution);
    ledcAttach(lpwmPins[i], pwmFreq, pwmResolution);
    ledcWrite(rpwmPins[i], 0);
    ledcWrite(lpwmPins[i], 0);
  }

  // Inisialisasi Servo Gripper
  servoLift.setPeriodHertz(50);
  servoGripper.setPeriodHertz(50);
  servoLift.attach(pinLift, 500, 2400);
  servoGripper.attach(pinGripper, 500, 2400);
  servoLift.write(liftAngle);
  servoGripper.write(gripAngle);

  // Buat Access Point WiFi
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setSleep(false);

  Serial.println("\n--- WiFi AP Siap ---");
  Serial.print("SSID : "); Serial.println(AP_SSID);
  Serial.print("IP   : "); Serial.println(WiFi.softAPIP());

  // Setup Web Server & WebSockets
  server.on("/", []() {
    server.send_P(200, "text/html", INDEX_HTML);
  });
  server.begin();

  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  lastCommandTime = millis();
}

void loop() {
  server.handleClient();
  webSocket.loop();

  // Pengaman Failsafe: Jika sedang jalan namun tidak ada sinyal baru selama > 400ms, hentikan robot
  if (isMoving && (millis() - lastCommandTime > TIMEOUT_FAILSAFE)) {
    berhenti();
    isMoving = false;
  }
}