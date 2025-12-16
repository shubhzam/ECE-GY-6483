/*
  PD Detector Web Bluetooth dashboard

  Matches the firmware in src/main.cpp:
  - Service UUID: 0x1810
  - Tremor Char: 0x1910 (6 bytes)
  - Dysk Char:   0x1911 (6 bytes)
  - FOG Char:    0x1912 (4 bytes)

  NOTE: Web Bluetooth works in Chromium-based browsers over HTTPS or localhost.
*/

const UUIDS = {
  service: 0x1810,
  tremor: 0x1910,
  dyskinesia: 0x1911,
  fog: 0x1912,
};

const els = {
  btnConnect: document.getElementById('btnConnect'),
  btnDisconnect: document.getElementById('btnDisconnect'),
  status: document.getElementById('status'),
  deviceName: document.getElementById('deviceName'),
  lastUpdate: document.getElementById('lastUpdate'),
  compatNote: document.getElementById('compatNote'),
  log: document.getElementById('log'),

  cardTremor: document.getElementById('cardTremor'),
  cardDysk: document.getElementById('cardDysk'),
  cardFog: document.getElementById('cardFog'),

  tremorDetected: document.getElementById('tremorDetected'),
  tremorIntensity: document.getElementById('tremorIntensity'),
  tremorIntensityPct: document.getElementById('tremorIntensityPct'),
  tremorIntensityBar: document.getElementById('tremorIntensityBar'),
  tremorFreq: document.getElementById('tremorFreq'),
  tremorFreqPlot: document.getElementById('tremorFreqPlot'),
  tremorPower: document.getElementById('tremorPower'),

  dyskDetected: document.getElementById('dyskDetected'),
  dyskIntensity: document.getElementById('dyskIntensity'),
  dyskIntensityPct: document.getElementById('dyskIntensityPct'),
  dyskIntensityBar: document.getElementById('dyskIntensityBar'),
  dyskFreq: document.getElementById('dyskFreq'),
  dyskFreqPlot: document.getElementById('dyskFreqPlot'),
  dyskPower: document.getElementById('dyskPower'),

  fogDetected: document.getElementById('fogDetected'),
  fogIntensity: document.getElementById('fogIntensity'),
  fogIntensityPct: document.getElementById('fogIntensityPct'),
  fogIntensityBar: document.getElementById('fogIntensityBar'),
};

const HISTORY_LEN = 60;
const tremorFreqHistory = [];
const dyskFreqHistory = [];

function pushHistory(arr, value) {
  if (!Number.isFinite(value)) return;
  arr.push(value);
  while (arr.length > HISTORY_LEN) arr.shift();
}

function resizeCanvasToDisplaySize(canvas) {
  if (!canvas) return;
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(1, Math.round(rect.width * dpr));
  const height = Math.max(1, Math.round(rect.height * dpr));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
}

function drawSparkline(canvas, values, opts) {
  if (!canvas) return;
  resizeCanvasToDisplaySize(canvas);
  const ctx = canvas.getContext('2d');
  if (!ctx) return;

  const { minY, maxY, stroke, grid } = opts;
  const w = canvas.width;
  const h = canvas.height;
  const pad = Math.round(10 * (window.devicePixelRatio || 1));

  // Background
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = '#ffffff';
  ctx.fillRect(0, 0, w, h);

  // Grid (2 horizontal lines)
  ctx.strokeStyle = grid;
  ctx.lineWidth = Math.max(1, Math.round(1 * (window.devicePixelRatio || 1)));
  ctx.beginPath();
  ctx.moveTo(pad, pad);
  ctx.lineTo(w - pad, pad);
  ctx.moveTo(pad, Math.round(h / 2));
  ctx.lineTo(w - pad, Math.round(h / 2));
  ctx.moveTo(pad, h - pad);
  ctx.lineTo(w - pad, h - pad);
  ctx.stroke();

  if (!values || values.length < 2) return;

  const clamp = (v) => Math.max(minY, Math.min(maxY, v));
  const xStep = (w - pad * 2) / (HISTORY_LEN - 1);
  const yScale = (h - pad * 2) / (maxY - minY);

  const toX = (i) => pad + i * xStep;
  const toY = (v) => h - pad - (clamp(v) - minY) * yScale;

  // Line
  ctx.strokeStyle = stroke;
  ctx.lineWidth = Math.max(2, Math.round(2 * (window.devicePixelRatio || 1)));
  ctx.lineJoin = 'round';
  ctx.lineCap = 'round';
  ctx.beginPath();

  // Align values to the right of the chart (latest on the right)
  const start = Math.max(0, HISTORY_LEN - values.length);
  for (let i = 0; i < values.length; i++) {
    const x = toX(start + i);
    const y = toY(values[i]);
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.stroke();

  // Latest point
  const lastIdx = values.length - 1;
  const lastX = toX(start + lastIdx);
  const lastY = toY(values[lastIdx]);
  ctx.fillStyle = stroke;
  ctx.beginPath();
  ctx.arc(lastX, lastY, Math.round(3 * (window.devicePixelRatio || 1)), 0, Math.PI * 2);
  ctx.fill();
}

let device = null;
let server = null;
let characteristics = {
  tremor: null,
  dyskinesia: null,
  fog: null,
};

let lastKnownMotionState = 'Disconnected';
let lastLoggedMotionState = 'Disconnected';

function logLine(message) {
  const time = new Date().toLocaleTimeString();
  els.log.textContent = `[${time}] ${message}\n` + els.log.textContent;
}

function setStatus(text) {
  els.status.textContent = text;
}

function refreshStatus() {
  if (device?.gatt?.connected) {
    setStatus(lastKnownMotionState);
  }
}

function setConnectedUi(connected) {
  els.btnConnect.disabled = connected;
  els.btnDisconnect.disabled = !connected;
}

function setLastUpdate() {
  els.lastUpdate.textContent = new Date().toLocaleString();
}

function formatDetected(flag) {
  return flag ? 'YES' : 'No';
}

function clampPct(value) {
  const n = Number(value);
  if (!Number.isFinite(n)) return 0;
  return Math.max(0, Math.min(100, n));
}

function setDetectedPill(pillEl, cardEl, detected) {
  const isYes = !!detected;
  pillEl.textContent = isYes ? 'DETECTED' : 'OK';
  pillEl.classList.remove('pill--yes', 'pill--no', 'pill--neutral');
  pillEl.classList.add(isYes ? 'pill--yes' : 'pill--no');
  if (cardEl) cardEl.classList.toggle('card--active', isYes);
}

function setIntensity(intensityEl, pctEl, barEl, intensity) {
  const pct = clampPct(intensity);
  intensityEl.textContent = `${pct}`;
  pctEl.textContent = `${pct}%`;
  barEl.style.width = `${pct}%`;
}

function requireWebBluetooth() {
  const ok = !!navigator.bluetooth;
  els.compatNote.hidden = ok;
  if (!ok) {
    setStatus('Web Bluetooth not available in this browser');
    setConnectedUi(false);
  }
  return ok;
}

function requireSecureContext() {
  // Web Bluetooth requires a secure context (HTTPS) except for localhost.
  const ok = window.isSecureContext;
  if (!ok) {
    setStatus('Open this page on https:// or http://localhost');
    setConnectedUi(false);
  }
  return ok;
}

function parse6ByteSymptom(view) {
  if (view.byteLength < 6) return null;
  const detected = view.getUint8(0);
  const intensity = view.getUint8(1);
  const freqX100 = view.getUint16(2, true);
  const power = view.getUint16(4, true);
  return {
    detected,
    intensity,
    freqHz: freqX100 / 100.0,
    power,
  };
}

function parseFog(view) {
  if (view.byteLength < 2) return null;
  const detected = view.getUint8(0);
  const intensity = view.getUint8(1);

  // Firmware uses reserved field (uint16 LE) to send extra state.
  // bit0: walkingActive
  let walking = 0;
  if (view.byteLength >= 4) {
    const reserved = view.getUint16(2, true);
    walking = reserved & 0x0001;
  }

  return { detected, intensity, walking };
}

async function connect() {
  if (!requireWebBluetooth()) return;
  if (!requireSecureContext()) return;

  setStatus('Requesting device…');
  logLine('Opening device picker');

  // Using acceptAllDevices makes discovery more reliable on macOS,
  // especially if the advertised name differs or is truncated.
  // The user should pick "PD-Detector" from the list.
  device = await navigator.bluetooth.requestDevice({
    acceptAllDevices: true,
    optionalServices: [UUIDS.service],
  });

  els.deviceName.textContent = device.name || 'Unknown';
  device.addEventListener('gattserverdisconnected', onDisconnected);

  setStatus('Connecting…');
  logLine(`Connecting to ${device.name || 'device'}…`);

  server = await device.gatt.connect();

  let service;
  try {
    service = await server.getPrimaryService(UUIDS.service);
  } catch (e) {
    throw new Error(
      'Selected device does not expose service 0x1810. Pick the board advertising as PD-Detector.'
    );
  }

  try {
    characteristics.tremor = await service.getCharacteristic(UUIDS.tremor);
    characteristics.dyskinesia = await service.getCharacteristic(UUIDS.dyskinesia);
    characteristics.fog = await service.getCharacteristic(UUIDS.fog);
  } catch (e) {
    throw new Error(
      'Connected, but expected characteristics (0x1910/0x1911/0x1912) were not found. Ensure the correct firmware is flashed.'
    );
  }

  await startNotify(characteristics.tremor, onTremor);
  await startNotify(characteristics.dyskinesia, onDyskinesia);
  await startNotify(characteristics.fog, onFog);

  setConnectedUi(true);
  lastKnownMotionState = 'Monitoring';
  setStatus(lastKnownMotionState);
  logLine('Subscribed to 0x1910/0x1911/0x1912 notifications');
}

async function startNotify(ch, handler) {
  ch.addEventListener('characteristicvaluechanged', handler);
  await ch.startNotifications();
}

function onTremor(event) {
  const data = parse6ByteSymptom(event.target.value);
  if (!data) return;

  setDetectedPill(els.tremorDetected, els.cardTremor, data.detected);
  setIntensity(els.tremorIntensity, els.tremorIntensityPct, els.tremorIntensityBar, data.intensity);
  els.tremorFreq.textContent = `${data.freqHz.toFixed(2)} Hz`;
  els.tremorPower.textContent = `${data.power}`;
  pushHistory(tremorFreqHistory, data.freqHz);
  drawSparkline(els.tremorFreqPlot, tremorFreqHistory, {
    minY: 0,
    maxY: 10,
    stroke: '#111827',
    grid: '#e5e7eb',
  });
  setLastUpdate();
}

function onDyskinesia(event) {
  const data = parse6ByteSymptom(event.target.value);
  if (!data) return;

  setDetectedPill(els.dyskDetected, els.cardDysk, data.detected);
  setIntensity(els.dyskIntensity, els.dyskIntensityPct, els.dyskIntensityBar, data.intensity);
  els.dyskFreq.textContent = `${data.freqHz.toFixed(2)} Hz`;
  els.dyskPower.textContent = `${data.power}`;
  pushHistory(dyskFreqHistory, data.freqHz);
  drawSparkline(els.dyskFreqPlot, dyskFreqHistory, {
    minY: 0,
    maxY: 10,
    stroke: '#111827',
    grid: '#e5e7eb',
  });
  setLastUpdate();
}

function onFog(event) {
  const data = parseFog(event.target.value);
  if (!data) return;

  setDetectedPill(els.fogDetected, els.cardFog, data.detected);
  setIntensity(els.fogIntensity, els.fogIntensityPct, els.fogIntensityBar, data.intensity);

  if (data.detected) {
    lastKnownMotionState = 'FOG detected';
  } else if (data.walking) {
    lastKnownMotionState = 'Walking';
  } else {
    lastKnownMotionState = 'Monitoring';
  }

  if (lastKnownMotionState !== lastLoggedMotionState) {
    logLine(`State: ${lastKnownMotionState}`);
    lastLoggedMotionState = lastKnownMotionState;
  }

  refreshStatus();

  setLastUpdate();
}

function onDisconnected() {
  logLine('Device disconnected');
  lastKnownMotionState = 'Disconnected';
  setStatus('Disconnected');
  setConnectedUi(false);
}

async function disconnect() {
  try {
    if (device?.gatt?.connected) {
      logLine('Disconnecting…');
      device.gatt.disconnect();
    }
  } finally {
    lastKnownMotionState = 'Disconnected';
    setStatus('Disconnected');
    setConnectedUi(false);
  }
}

els.btnConnect.addEventListener('click', () => {
  connect().catch((err) => {
    console.error(err);
    logLine(`Error: ${err?.message || String(err)}`);
    setStatus('Error (see log)');
    setConnectedUi(false);
  });
});

els.btnDisconnect.addEventListener('click', () => {
  disconnect().catch((err) => {
    console.error(err);
    logLine(`Error: ${err?.message || String(err)}`);
  });
});

requireWebBluetooth();
requireSecureContext();
refreshStatus();

window.addEventListener('resize', () => {
  drawSparkline(els.tremorFreqPlot, tremorFreqHistory, { minY: 0, maxY: 10, stroke: '#111827', grid: '#e5e7eb' });
  drawSparkline(els.dyskFreqPlot, dyskFreqHistory, { minY: 0, maxY: 10, stroke: '#111827', grid: '#e5e7eb' });
});
