// ===========================
// WebHelper.ino
// Webpage + key handling + webPrint log + webPlot canvas
// Accepts ANY keypress automatically
// webPrint() pushes messages to browser via SSE (default event)
// webPlot()  pushes labeled values via SSE (named "plot" event)
// ===========================

#include <WebServer.h>
#include "WebHelper.h"

WebServer server(80);

Keyboard keyboard;
unsigned long lastStateTime = 0;

static WiFiClient sseClients[3];
static const int SSE_SLOTS = 3;

const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Control</title>
  <style>
    @import url('https://fonts.googleapis.com/css2?family=Space+Mono:wght@400;700&display=swap');

    * { box-sizing: border-box; margin: 0; padding: 0; }

    body {
      font-family: 'Space Mono', monospace;
      background: #0f0f0f;
      color: #f0f0f0;
      height: 100vh;
      display: flex;
      flex-direction: column;
    }

    header {
      text-align: center;
      padding: 1.2rem;
      font-size: 0.85rem;
      letter-spacing: 0.3em;
      text-transform: uppercase;
      color: #555;
      border-bottom: 1px solid #1a1a1a;
    }

    .panels {
      flex: 1;
      display: flex;
      min-height: 0;
    }

    .key-panel {
      width: 220px;
      flex-shrink: 0;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      gap: 1rem;
      border-right: 1px solid #222;
      background: #0d0d0d;
      padding: 1rem;
    }

    #keyDisplay {
      font-size: 4.5rem;
      font-weight: 700;
      width: 110px;
      height: 110px;
      display: flex;
      align-items: center;
      justify-content: center;
      border: 2px solid #333;
      border-radius: 12px;
      background: #1a1a1a;
      transition: all 0.1s ease;
      color: #f0f0f0;
    }

    #keyDisplay.active {
      border-color: #4af;
      color: #4af;
      box-shadow: 0 0 30px #4af3;
    }

    #status { font-size: 0.65rem; color: #444; letter-spacing: 0.08em; text-align: center; }

    #log {
      font-size: 0.65rem;
      color: #555;
      width: 180px;
      height: 100px;
      overflow-y: auto;
      border: 1px solid #1c1c1c;
      border-radius: 6px;
      padding: 0.4rem;
      display: flex;
      flex-direction: column-reverse;
    }

    .log-entry { color: #4af; }
    .log-entry.stop { color: #2a2a2a; }

    #hint { font-size: 0.55rem; color: #222; letter-spacing: 0.06em; text-align: center; line-height: 1.7; }

    /* ── right panel ── */
    .monitor-panel {
      flex: 1;
      display: flex;
      flex-direction: column;
      padding: 1rem 1.2rem;
      gap: 0.5rem;
      min-width: 0;
    }

    /* ── tab bar ── */
    .tab-bar {
      display: flex;
      align-items: center;
      justify-content: space-between;
      padding-bottom: 0.4rem;
      border-bottom: 1px solid #1a2e1a;
      flex-shrink: 0;
    }

    .tabs { display: flex; gap: 1rem; }

    .tab-btn {
      font-family: inherit;
      font-size: 0.65rem;
      letter-spacing: 0.2em;
      text-transform: uppercase;
      background: none;
      border: none;
      border-bottom: 2px solid transparent;
      padding: 0.15rem 0 0.3rem;
      cursor: pointer;
      color: #2a4a2a;
      transition: color 0.15s, border-color 0.15s;
    }

    .tab-btn.active {
      color: #7f7;
      border-bottom-color: #4a7a4a;
    }

    #resumeBtn {
      font-family: inherit;
      font-size: 0.65rem;
      letter-spacing: 0.1em;
      text-transform: uppercase;
      background: none;
      border: 1px solid #1a2e1a;
      border-radius: 4px;
      padding: 0.3rem 0.8rem;
      cursor: pointer;
      color: #1a3a1a;
      transition: color 0.15s, border-color 0.15s, background 0.15s;
    }

    #resumeBtn.paused {
      color: #0f0f0f;
      background: #7f7;
      border-color: #7f7;
      font-weight: 700;
    }

    /* ── monitor tab content ── */
    #monitor-content {
      flex: 1;
      display: flex;
      flex-direction: column;
      gap: 0.5rem;
      min-height: 0;
    }

    #monitor {
      flex: 1;
      overflow-y: auto;
      overflow-anchor: none;
      font-size: 1.17rem;
      color: #7f7;
      background: #0a140a;
      border: 1px solid #1a2e1a;
      border-radius: 6px;
      position: relative;
    }

    #mon-inner { position: relative; }

    #mon-rows {
      position: absolute;
      left: 0;
      right: 0;
      padding: 0 0.7rem;
    }

    .mon-entry {
      height: 28px;
      line-height: 28px;
      white-space: nowrap;
      overflow: hidden;
      text-overflow: ellipsis;
    }

    #monitor-status { font-size: 0.6rem; color: #2a4a2a; letter-spacing: 0.08em; flex-shrink: 0; }

    /* ── plotter tab content ── */
    #plotter-content {
      flex: 1;
      display: none;
      min-height: 0;
    }

    #plotterCanvas {
      width: 100%;
      height: 100%;
      display: block;
      border-radius: 6px;
    }
  </style>
</head>
<body>
  <header>Robot Control</header>

  <div class="panels">
    <div class="key-panel">
      <div id="keyDisplay">—</div>
      <div id="status">ready — press any key</div>
      <div id="log"></div>
      <div id="hint">WASD / arrows / 0–9<br>combinations supported</div>
    </div>

    <div class="monitor-panel">
      <div class="tab-bar">
        <div class="tabs">
          <button class="tab-btn active" id="tabMonitor">monitor</button>
          <button class="tab-btn"        id="tabPlotter">plotter</button>
        </div>
        <button id="resumeBtn">Resume</button>
      </div>

      <div id="monitor-content">
        <div id="monitor"><div id="mon-inner"><div id="mon-rows"></div></div></div>
        <div id="monitor-status">connecting...</div>
      </div>

      <div id="plotter-content">
        <canvas id="plotterCanvas"></canvas>
      </div>
    </div>
  </div>

  <script>
    // ── key panel ──────────────────────────────────────────────
    const display   = document.getElementById('keyDisplay');
    const status    = document.getElementById('status');
    const log       = document.getElementById('log');
    const resumeBtn = document.getElementById('resumeBtn');

    function addLog(key) {
      const entry = document.createElement('div');
      entry.className = 'log-entry' + (key === 'stop' ? ' stop' : '');
      entry.textContent = key === 'stop' ? '— stop' : '▶ ' + key;
      log.prepend(entry);
      if (log.children.length > 20) log.removeChild(log.lastChild);
    }

    function normalizeKey(k) {
      if (k === 'ArrowUp')    return 'UP';
      if (k === 'ArrowDown')  return 'DOWN';
      if (k === 'ArrowLeft')  return 'LEFT';
      if (k === 'ArrowRight') return 'RIGHT';
      if (k === ' ')          return 'SPACE';
      if (k.length === 1 && /^[a-zA-Z0-9]$/.test(k)) return k.toLowerCase();
      return null;
    }

    function displayLabel(token) {
      if (token === 'UP')    return '↑';
      if (token === 'DOWN')  return '↓';
      if (token === 'LEFT')  return '←';
      if (token === 'RIGHT') return '→';
      if (token === 'SPACE') return 'SPC';
      return token.toUpperCase();
    }

    const heldKeys = new Set();

    function updateDisplay() {
      const keys = [...heldKeys];
      if (keys.length === 0) {
        display.textContent    = '—';
        display.className      = '';
        display.style.fontSize = '';
        status.textContent     = 'ready — press any key';
      } else {
        display.textContent    = keys.map(displayLabel).join('+');
        display.className      = 'active';
        display.style.fontSize = keys.length === 1 ? '' : keys.length === 2 ? '2rem' : '1.2rem';
        status.textContent     = 'held: ' + keys.join('+');
      }
    }

    document.addEventListener('keydown', e => {
      if (e.repeat) return;
      const key = normalizeKey(e.key);
      if (!key) return;
      if (!heldKeys.has(key)) addLog(key);
      heldKeys.add(key);
      updateDisplay();
    });
    document.addEventListener('keyup', e => {
      const key = normalizeKey(e.key);
      if (!key) return;
      heldKeys.delete(key);
      updateDisplay();
      if (heldKeys.size === 0) addLog('stop');
    });

    setInterval(() => {
      const held = [...heldKeys].join(',');
      fetch('/state?held=' + encodeURIComponent(held)).catch(() => {
        status.textContent = 'connection lost — refresh page';
        display.className  = '';
      });
    }, 50);

    // ── monitor ────────────────────────────────────────────────
    const monitor   = document.getElementById('monitor');
    const monInner  = document.getElementById('mon-inner');
    const monRows   = document.getElementById('mon-rows');
    const monStatus = document.getElementById('monitor-status');

    const ROW_HEIGHT  = 28;
    const BUFFER      = 5;
    const MAX_MSGS    = 10000;
    const allMessages = [];
    const messageQueue = [];
    let rafPending       = false;
    let scrollRafPending = false;
    let paused = false;

    function setPaused(val) {
      paused = val;
      resumeBtn.classList.toggle('paused', val);
      resumeBtn.textContent = val ? 'Paused — click to resume' : 'Resume';
    }

    function renderWindow() {
      const total = allMessages.length;
      monInner.style.height = (total * ROW_HEIGHT) + 'px';
      if (total === 0) { monRows.innerHTML = ''; return; }

      const firstVis = Math.floor(monitor.scrollTop / ROW_HEIGHT);
      const visCount = Math.ceil((monitor.clientHeight || 400) / ROW_HEIGHT);
      const start    = Math.max(0, firstVis - BUFFER);
      const end      = Math.min(total, firstVis + visCount + BUFFER);
      const needed   = end - start;

      monRows.style.top = (start * ROW_HEIGHT) + 'px';

      while (monRows.children.length > needed) monRows.removeChild(monRows.lastChild);
      while (monRows.children.length < needed) {
        const div = document.createElement('div');
        div.className = 'mon-entry';
        monRows.appendChild(div);
      }
      for (let i = 0; i < needed; i++) {
        monRows.children[i].textContent = allMessages[start + i];
      }
    }

    resumeBtn.addEventListener('click', () => {
      if (paused) {
        setPaused(false);
        monitor.scrollTop = allMessages.length * ROW_HEIGHT;
        renderWindow();
      } else {
        setPaused(true);
      }
    });

    monitor.addEventListener('click', () => { if (!paused) setPaused(true); });

    monitor.addEventListener('scroll', () => {
      const distFromBottom = allMessages.length * ROW_HEIGHT - monitor.scrollTop - monitor.clientHeight;
      if (!paused && distFromBottom > 20) setPaused(true);
      if (paused && distFromBottom <= 20) { setPaused(false); renderWindow(); }
      if (!scrollRafPending) {
        scrollRafPending = true;
        requestAnimationFrame(() => { scrollRafPending = false; renderWindow(); });
      }
    });

    function flushMessages() {
      rafPending = false;
      if (messageQueue.length === 0) return;
      while (messageQueue.length > 0) allMessages.push(messageQueue.shift());
      while (allMessages.length > MAX_MSGS) allMessages.shift();
      monInner.style.height = (allMessages.length * ROW_HEIGHT) + 'px';
      if (!paused) {
        monitor.scrollTop = allMessages.length * ROW_HEIGHT;
        renderWindow();
      }
    }

    function addMonitorLine(text) {
      messageQueue.push(text);
      if (!rafPending) { rafPending = true; requestAnimationFrame(flushMessages); }
    }

    // ── plotter ────────────────────────────────────────────────
    const canvas      = document.getElementById('plotterCanvas');
    const PLOT_WINDOW = 200;
    const COLORS      = ['#4af','#f84','#7f7','#f4f','#fa4','#a4f'];
    const series      = {};
    let colorCounter  = 0;
    let plotRafPending = false;

    function onPlotData(e) {
      const eq = e.data.indexOf('=');
      if (eq < 0) return;
      const label = e.data.slice(0, eq);
      const value = parseFloat(e.data.slice(eq + 1));
      if (isNaN(value)) return;

      if (!series[label]) {
        series[label] = { values: [], colorIdx: colorCounter++ % COLORS.length };
      }
      const s = series[label];
      s.values.push(value);
      if (s.values.length > PLOT_WINDOW) s.values.shift();

      if (!plotRafPending) { plotRafPending = true; requestAnimationFrame(drawPlot); }
    }

    function drawPlot() {
      plotRafPending = false;
      const dpr = window.devicePixelRatio || 1;
      const W   = canvas.clientWidth;
      const H   = canvas.clientHeight;
      if (W === 0 || H === 0) return;

      if (canvas.width !== W * dpr || canvas.height !== H * dpr) {
        canvas.width  = W * dpr;
        canvas.height = H * dpr;
      }

      const ctx = canvas.getContext('2d');
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, W, H);
      ctx.fillStyle = '#0a140a';
      ctx.fillRect(0, 0, W, H);

      const labels = Object.keys(series);
      if (labels.length === 0) {
        ctx.fillStyle = '#1a3a1a';
        ctx.font = '12px Space Mono, monospace';
        ctx.textAlign = 'center';
        ctx.fillText('no data — call webPlot(label, value)', W / 2, H / 2);
        return;
      }

      // global min/max across all series
      let mn = Infinity, mx = -Infinity;
      for (const lbl of labels) {
        for (const v of series[lbl].values) {
          if (v < mn) mn = v;
          if (v > mx) mx = v;
        }
      }
      if (mn === mx) { mn -= 1; mx += 1; }
      const pad = (mx - mn) * 0.08;
      mn -= pad; mx += pad;

      const ML = 52, MR = 12, MT = 14, MB = 10;
      const pw = W - ML - MR;
      const ph = H - MT - MB;

      // grid lines + Y labels
      ctx.font = '10px Space Mono, monospace';
      ctx.textAlign = 'right';
      const gridN = 5;
      for (let i = 0; i <= gridN; i++) {
        const y   = MT + ph * i / gridN;
        const val = mx - (mx - mn) * i / gridN;
        ctx.strokeStyle = '#162816';
        ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(ML, y); ctx.lineTo(ML + pw, y); ctx.stroke();
        ctx.fillStyle = '#2a4a2a';
        ctx.fillText(val.toFixed(1), ML - 5, y + 3);
      }

      // plot each series
      for (const lbl of labels) {
        const s    = series[lbl];
        const vals = s.values;
        if (vals.length < 2) continue;
        ctx.strokeStyle = COLORS[s.colorIdx];
        ctx.lineWidth   = 1.5;
        ctx.beginPath();
        for (let i = 0; i < vals.length; i++) {
          const x = ML + pw * i / (PLOT_WINDOW - 1);
          const y = MT + ph * (1 - (vals[i] - mn) / (mx - mn));
          i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
        }
        ctx.stroke();
      }

      // legend (top-left inside plot area)
      ctx.font = '11px Space Mono, monospace';
      ctx.textAlign = 'left';
      let ly = MT + 14;
      for (const lbl of labels) {
        ctx.fillStyle = COLORS[series[lbl].colorIdx];
        ctx.fillText('— ' + lbl, ML + 6, ly);
        ly += 16;
      }
    }

    new ResizeObserver(() => {
      if (currentTab === 'plotter') drawPlot();
    }).observe(document.getElementById('plotter-content'));

    // ── tabs ───────────────────────────────────────────────────
    let currentTab = 'monitor';
    const tabMonitor    = document.getElementById('tabMonitor');
    const tabPlotter    = document.getElementById('tabPlotter');
    const monitorContent = document.getElementById('monitor-content');
    const plotterContent = document.getElementById('plotter-content');

    tabMonitor.addEventListener('click', () => {
      if (currentTab === 'monitor') return;
      currentTab = 'monitor';
      tabMonitor.classList.add('active');
      tabPlotter.classList.remove('active');
      monitorContent.style.display = 'flex';
      plotterContent.style.display = 'none';
      resumeBtn.style.display = '';
    });

    tabPlotter.addEventListener('click', () => {
      if (currentTab === 'plotter') return;
      currentTab = 'plotter';
      tabPlotter.classList.add('active');
      tabMonitor.classList.remove('active');
      monitorContent.style.display = 'none';
      plotterContent.style.display = 'flex';
      resumeBtn.style.display = 'none';
      requestAnimationFrame(drawPlot);
    });

    // ── SSE ────────────────────────────────────────────────────
    function connectSSE() {
      const es = new EventSource('/events');
      es.onopen    = () => { monStatus.textContent = 'connected'; };
      es.onmessage = e  => { addMonitorLine(e.data); };
      es.addEventListener('plot', onPlotData);
      es.onerror   = () => {
        monStatus.textContent = 'reconnecting...';
        es.close();
        setTimeout(connectSSE, 2000);
      };
    }

    connectSSE();
  </script>
</body>
</html>
)rawliteral";

static void handleRoot() {
  server.send(200, "text/html", PAGE);
}

static void applyKey(const String& key) {
  if (key == "UP")    { keyboard.up    = true; return; }
  if (key == "DOWN")  { keyboard.down  = true; return; }
  if (key == "LEFT")  { keyboard.left  = true; return; }
  if (key == "RIGHT") { keyboard.right = true; return; }
  if (key == "SPACE") { keyboard.space = true; return; }
  if (key.length() != 1) return;
  char c = key[0];
  switch (c) {
    case 'a': keyboard.a = true; break; case 'b': keyboard.b = true; break;
    case 'c': keyboard.c = true; break; case 'd': keyboard.d = true; break;
    case 'e': keyboard.e = true; break; case 'f': keyboard.f = true; break;
    case 'g': keyboard.g = true; break; case 'h': keyboard.h = true; break;
    case 'i': keyboard.i = true; break; case 'j': keyboard.j = true; break;
    case 'k': keyboard.k = true; break; case 'l': keyboard.l = true; break;
    case 'm': keyboard.m = true; break; case 'n': keyboard.n = true; break;
    case 'o': keyboard.o = true; break; case 'p': keyboard.p = true; break;
    case 'q': keyboard.q = true; break; case 'r': keyboard.r = true; break;
    case 's': keyboard.s = true; break; case 't': keyboard.t = true; break;
    case 'u': keyboard.u = true; break; case 'v': keyboard.v = true; break;
    case 'w': keyboard.w = true; break; case 'x': keyboard.x = true; break;
    case 'y': keyboard.y = true; break; case 'z': keyboard.z = true; break;
    case '0': keyboard.n0 = true; break; case '1': keyboard.n1 = true; break;
    case '2': keyboard.n2 = true; break; case '3': keyboard.n3 = true; break;
    case '4': keyboard.n4 = true; break; case '5': keyboard.n5 = true; break;
    case '6': keyboard.n6 = true; break; case '7': keyboard.n7 = true; break;
    case '8': keyboard.n8 = true; break; case '9': keyboard.n9 = true; break;
  }
}

static void handleState() {
  keyboard = {};
  if (server.hasArg("held")) {
    String held = server.arg("held");
    int start = 0;
    for (int i = 0; i <= (int)held.length(); i++) {
      if (i == (int)held.length() || held[i] == ',') {
        if (i > start) applyKey(held.substring(start, i));
        start = i + 1;
      }
    }
  }
  lastStateTime = millis();
  server.send(200, "text/plain", "ok");
}

static void handleEvents() {
  WiFiClient c = server.client();

  c.print("HTTP/1.1 200 OK\r\n");
  c.print("Content-Type: text/event-stream\r\n");
  c.print("Cache-Control: no-cache\r\n");
  c.print("Connection: keep-alive\r\n");
  c.print("Access-Control-Allow-Origin: *\r\n");
  c.print("\r\n");
  c.flush();

  for (int i = 0; i < SSE_SLOTS; i++) {
    if (!sseClients[i].connected()) {
      sseClients[i] = c;
      c.print("data: connected\r\n\r\n");
      c.flush();
      return;
    }
  }
  sseClients[0].stop();
  for (int i = 1; i < SSE_SLOTS; i++) sseClients[i - 1] = sseClients[i];
  sseClients[SSE_SLOTS - 1] = c;
  c.print("data: connected\r\n\r\n");
  c.flush();
}


static void sseWrite(String msg) {
  for (int i = 0; i < SSE_SLOTS; i++) {
    if (sseClients[i].connected()) {
      sseClients[i].print("data: ");
      sseClients[i].print(msg);
      sseClients[i].print("\r\n\r\n");
      sseClients[i].flush();
    }
  }
}

static void sseWritePlot(const char* label, float value) {
  String msg = String(label) + "=" + String(value, 4);
  for (int i = 0; i < SSE_SLOTS; i++) {
    if (sseClients[i].connected()) {
      sseClients[i].print("event: plot\r\ndata: ");
      sseClients[i].print(msg);
      sseClients[i].print("\r\n\r\n");
      sseClients[i].flush();
    }
  }
}

void webPrint(String msg)        { Serial.print(msg);   sseWrite(msg); }
void webPrintLn(String msg)      { Serial.println(msg); sseWrite(msg); }
void webPrintLn()                { Serial.println();    }

void webPrint(const char* msg)   { webPrint(String(msg)); }
void webPrint(int msg)           { webPrint(String(msg)); }
void webPrint(double msg)        { webPrint(String(msg)); }
void webPrint(float msg)         { webPrint(String(msg)); }
void webPrint(bool msg)          { webPrint(msg ? "true" : "false"); }
void webPrint(long msg)          { webPrint(String(msg)); }
void webPrint(unsigned long msg) { webPrint(String(msg)); }

void webPrintLn(const char* msg)   { webPrintLn(String(msg)); }
void webPrintLn(int msg)           { webPrintLn(String(msg)); }
void webPrintLn(double msg)        { webPrintLn(String(msg)); }
void webPrintLn(float msg)         { webPrintLn(String(msg)); }
void webPrintLn(bool msg)          { webPrintLn(msg ? "true" : "false"); }
void webPrintLn(long msg)          { webPrintLn(String(msg)); }
void webPrintLn(unsigned long msg) { webPrintLn(String(msg)); }

void webPlot(const char* label, float value)        { sseWritePlot(label, value); }
void webPlot(const char* label, int value)           { sseWritePlot(label, (float)value); }
void webPlot(const char* label, double value)        { sseWritePlot(label, (float)value); }
void webPlot(const char* label, long value)          { sseWritePlot(label, (float)value); }
void webPlot(const char* label, unsigned long value) { sseWritePlot(label, (float)value); }

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/state", handleState);
  server.on("/events", HTTP_GET, handleEvents);
  server.begin();
  Serial.println("Web server started");
  Serial.println("Open browser to: http://" + WiFi.localIP().toString());
}

void handleWebServer() {
  server.handleClient();
  if (lastStateTime > 0 && millis() - lastStateTime > 200) keyboard = {};
}
