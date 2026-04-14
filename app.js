const STORAGE_KEY = 'robot-control-ui-config';
const HOLD_DURATION = 2000;
const MOBILE_VIEW_KEY = 'robot-control-ui-mobile-view';
const MOBILE_VIEWS = ['vision', 'drive', 'status', 'logs'];

const state = {
  robotName: 'LATON',
  streamUrl: 'http://172.20.10.2:5000',
  mockMode: false,
  connected: true,
  battery: 86,
  mode: 'AUTO',
  target: 'Charging Dock',
  distance: '1.8 m',
  behavior: 'Autonomous Patrol',
  log: [],
  apiBase: 'http://172.20.10.2:9000',
  mockTimer: null,
  streamError: false,
  streamReady: false,
  mobileView: 'vision',
};

const mockScenarios = [
  { target: 'Charging Dock', distance: '1.8 m', behavior: 'Autonomous Patrol', batteryDelta: -1 },
  { target: 'Visitor', distance: '0.9 m', behavior: 'Target Tracking', batteryDelta: 0 },
  { target: 'Obstacle Box', distance: '0.4 m', behavior: 'Obstacle Avoiding', batteryDelta: -1 },
  { target: 'Hallway', distance: '3.1 m', behavior: 'Cruise Control', batteryDelta: 0 },
  { target: 'Operator', distance: '1.2 m', behavior: 'Awaiting Command', batteryDelta: 0 },
];

const $ = (selector) => document.querySelector(selector);
const $$ = (selector) => Array.from(document.querySelectorAll(selector));

const els = {
  robotName: $('#robotName'),
  appShell: $('.app-shell'),
  connectionPill: $('#connectionPill'),
  connectionText: $('#connectionText'),
  connectionDot: $('#connectionDot'),
  modeChipValue: $('#modeChipValue'),
  batteryText: $('#batteryText'),
  batteryPanelValue: $('#batteryPanelValue'),
  modeValue: $('#modeValue'),
  targetValue: $('#targetValue'),
  behaviorValue: $('#behaviorValue'),
  distanceValue: $('#distanceValue'),
  statusConnectionValue: $('#statusConnectionValue'),
  connectionStatValue: $('#connectionStatValue'),
  mockModeValue: $('#mockModeValue'),
  missionSummary: $('#missionSummary'),
  routeStateValue: $('#routeStateValue'),
  routeModeValue: $('#routeModeValue'),
  routeTargetValue: $('#routeTargetValue'),
  routeMockValue: $('#routeMockValue'),
  cameraState: $('#cameraState'),
  cameraStateLabel: $('#cameraStateLabel'),
  cameraStateTitle: $('#cameraStateTitle'),
  cameraStateDescription: $('#cameraStateDescription'),
  streamHint: $('#streamHint'),
  streamUrlInput: $('#streamUrlInput'),
  applyStreamBtn: $('#applyStreamBtn'),
  mockToggle: $('#mockToggle'),
  mockToggleText: $('#mockToggleText'),
  streamImage: $('#streamImage'),
  videoFrame: $('#videoFrame'),
  eventLog: $('#eventLog'),
  refreshBtn: $('#refreshBtn'),
  refreshQuickBtn: $('#refreshQuickBtn'),
  clearLogBtn: $('#clearLogBtn'),
  holdBar: $('#holdBar'),
  shutdownBtn: $('#shutdownBtn'),
  mobileNavButtons: $$('.mobile-nav-btn[data-mobile-view]'),
};

function setText(element, value) {
  if (element) {
    element.textContent = value;
  }
}

function clampBattery(value) {
  return Math.max(0, Math.min(100, value));
}

function loadConfig() {
  try {
    const saved = JSON.parse(localStorage.getItem(STORAGE_KEY) || '{}');
    Object.assign(state, {
      streamUrl: saved.streamUrl ?? state.streamUrl,
      mockMode: saved.mockMode ?? state.mockMode,
      apiBase: saved.apiBase ?? state.apiBase,
      mode: saved.mode ?? state.mode,
    });
    state.robotName = 'LATON';
  } catch (error) {
    console.warn('Failed to load saved config:', error);
  }

  try {
    const savedMobileView = localStorage.getItem(MOBILE_VIEW_KEY) || state.mobileView;
    state.mobileView = MOBILE_VIEWS.includes(savedMobileView) ? savedMobileView : 'vision';
  } catch (error) {
    console.warn('Failed to load mobile view:', error);
  }
}

function saveConfig() {
  localStorage.setItem(
    STORAGE_KEY,
    JSON.stringify({
      streamUrl: state.streamUrl,
      mockMode: state.mockMode,
      apiBase: state.apiBase,
      mode: state.mode,
    })
  );

  localStorage.setItem(MOBILE_VIEW_KEY, state.mobileView);
}

function addLog(message, type = 'info') {
  state.log.unshift({
    message,
    type,
    time: new Date().toLocaleTimeString('zh-CN', { hour12: false }),
  });
  state.log = state.log.slice(0, 20);
  renderLog();
}

function renderLog() {
  els.eventLog.innerHTML = '';

  if (!state.log.length) {
    const li = document.createElement('li');
    li.className = 'empty';

    const time = document.createElement('span');
    time.className = 'time';
    time.textContent = '--:--:--';

    const message = document.createElement('p');
    message.className = 'message';
    message.textContent = 'Waiting for commands or telemetry updates.';

    li.append(time, message);
    els.eventLog.appendChild(li);
    return;
  }

  state.log.forEach((entry) => {
    const li = document.createElement('li');
    if (entry.type && entry.type !== 'info') {
      li.classList.add(entry.type);
    }

    const time = document.createElement('span');
    time.className = 'time';
    time.textContent = entry.time;

    const message = document.createElement('p');
    message.className = 'message';
    message.textContent = entry.message;

    li.append(time, message);
    els.eventLog.appendChild(li);
  });
}

function renderConnection() {
  const isOnline = state.connected;
  const label = isOnline ? 'ONLINE' : 'OFFLINE';
  const detail = isOnline ? 'Online' : 'Offline';

  setText(els.connectionText, label);
  setText(els.statusConnectionValue, detail);
  setText(els.connectionStatValue, detail);

  els.connectionPill.classList.toggle('offline', !isOnline);
  els.connectionDot.classList.toggle('offline', !isOnline);
}

function renderMobileView() {
  if (!els.appShell) return;

  els.appShell.dataset.mobileView = state.mobileView;
  els.mobileNavButtons.forEach((button) => {
    const isActive = button.dataset.mobileView === state.mobileView;
    button.classList.toggle('is-active', isActive);
    button.setAttribute('aria-pressed', String(isActive));
  });
}

function renderModeButtons() {
  $$('.segment').forEach((button) => {
    button.classList.toggle('active', button.dataset.mode === state.mode);
  });
}

function renderRouteNotes() {
  const routeState = state.mockMode ? 'Mock Session Ready' : state.connected ? 'System Ready' : 'Connection Lost';
  const routeMock = state.mockMode
    ? 'Simulated video and telemetry are active'
    : state.streamError
      ? 'Fallback deck shown because the camera stream failed'
      : state.streamUrl
        ? 'Live stream URL is configured'
        : 'Camera stream URL is not configured';

  setText(els.routeStateValue, routeState);
  setText(els.routeModeValue, `Mode ${state.mode} selected`);
  setText(els.routeTargetValue, `Target ${state.target}`);
  setText(els.routeMockValue, routeMock);

  els.routeModeValue.className = state.mode === 'AUTO' ? 'done' : '';
  els.routeTargetValue.className = state.connected ? 'active' : '';
  els.routeMockValue.className = state.mockMode || state.streamUrl ? 'done' : '';
}

function renderVideoMeta() {
  const hint = state.mockMode
    ? 'Mock mode is generating video and telemetry locally.'
    : state.streamError
      ? 'Stream could not be loaded. Check the camera URL or network.'
      : state.streamUrl
        ? 'Live stream configured. Use Refresh to sync robot status.'
        : 'Add a camera URL to display live robot video.';

  setText(els.streamHint, hint);
}

function renderCameraState() {
  if (!els.cameraState) {
    return;
  }

  if (state.mockMode) {
    els.cameraState.hidden = false;
    setText(els.cameraStateLabel, 'Mock Mode Active');
    setText(els.cameraStateTitle, 'Live camera is disabled while mock mode is on.');
    setText(els.cameraStateDescription, 'Turn off Mock to show the real robot camera stream.');
    return;
  }

  if (!state.streamUrl) {
    els.cameraState.hidden = false;
    setText(els.cameraStateLabel, 'No Camera URL');
    setText(els.cameraStateTitle, 'Add a camera stream URL to show live video.');
    setText(els.cameraStateDescription, 'Enter a reachable MJPEG or image snapshot stream address below.');
    return;
  }

  if (state.streamError) {
    els.cameraState.hidden = false;
    setText(els.cameraStateLabel, 'Stream Unavailable');
    setText(els.cameraStateTitle, 'This device could not load the live camera stream.');
    setText(els.cameraStateDescription, `Check whether ${state.streamUrl} can be opened from this phone.`);
    return;
  }

  if (!state.streamReady) {
    els.cameraState.hidden = false;
    setText(els.cameraStateLabel, 'Loading Camera');
    setText(els.cameraStateTitle, 'Trying to open the live camera stream...');
    setText(els.cameraStateDescription, 'If it keeps loading, verify the phone and robot are on the same network.');
    return;
  }

  els.cameraState.hidden = true;
}

function renderVideo(forceReload = false) {
  const shouldUseLiveImage = !state.mockMode && Boolean(state.streamUrl);

  renderVideoMeta();

  if (!shouldUseLiveImage) {
    state.streamReady = false;
    els.streamImage.hidden = true;
    renderCameraState();
    return;
  }

  const currentSource = els.streamImage.dataset.source || '';
  if (forceReload || currentSource !== state.streamUrl) {
    state.streamError = false;
    state.streamReady = false;
    els.streamImage.dataset.source = state.streamUrl;
    els.streamImage.hidden = false;
    els.streamImage.src = state.streamUrl;
  }

  els.streamImage.hidden = state.streamError;
  renderCameraState();
}

function renderStatus(forceReloadVideo = false) {
  const batteryLabel = `${state.battery}%`;
  const mockLabel = state.mockMode ? 'Mock' : 'Live';
  const missionSummary = state.connected
    ? `Tracking ${state.target} at ${state.distance}.`
    : 'Robot connection is offline. Check the stream or API endpoint.';

  setText(els.robotName, state.robotName);
  setText(els.modeChipValue, state.mode);
  setText(els.batteryText, batteryLabel);
  setText(els.batteryPanelValue, batteryLabel);
  setText(els.modeValue, state.mode);
  setText(els.targetValue, state.target);
  setText(els.behaviorValue, state.behavior);
  setText(els.distanceValue, state.distance);
  setText(els.connectionStatValue, state.connected ? 'Online' : 'Offline');
  setText(els.mockModeValue, mockLabel);
  setText(els.missionSummary, missionSummary);
  els.streamUrlInput.value = state.streamUrl;

  els.mockToggle.classList.toggle('active', state.mockMode);
  els.mockToggle.setAttribute('aria-pressed', String(state.mockMode));
  setText(els.mockToggleText, state.mockMode ? 'Mock On' : 'Mock Off');

  renderConnection();
  renderModeButtons();
  renderRouteNotes();
  renderVideo(forceReloadVideo);
  renderMobileView();
}

function applyMockScenario() {
  if (!state.mockMode) return;

  const scenario = mockScenarios[Math.floor(Math.random() * mockScenarios.length)];
  state.target = scenario.target;
  state.distance = scenario.distance;
  state.behavior = scenario.behavior;
  state.connected = true;
  state.streamError = false;
  state.battery = clampBattery(state.battery + scenario.batteryDelta);

  renderStatus();
}

function startMockLoop() {
  stopMockLoop();
  applyMockScenario();
  state.mockTimer = setInterval(applyMockScenario, 2600);
}

function stopMockLoop() {
  if (state.mockTimer) {
    clearInterval(state.mockTimer);
    state.mockTimer = null;
  }
}

function applyCommandState(payload) {
  if (payload.mode) {
    state.mode = payload.mode;
  }

  if (payload.command === 'stop') {
    state.behavior = 'Stopped';
  }

  if (payload.command && payload.command !== 'stop') {
    state.behavior = `Moving ${payload.command}`;
  }

  if (payload.action === 'hello') {
    state.behavior = 'Waving Hello';
  }

  if (payload.action === 'pushup') {
    state.behavior = 'Doing Pushups';
  }

  if (payload.system === 'start') {
    state.connected = true;
    state.streamError = false;
    state.behavior = 'Boot Sequence';
  }

  if (payload.system === 'stop') {
    state.behavior = 'Idle';
  }

  if (payload.system === 'shutdown') {
    state.connected = false;
    state.behavior = 'Powered Off';
  }
}

async function sendCommand(kind, payload) {
  const label = payload.command || payload.mode || payload.action || payload.system;

  if (state.mockMode) {
    applyCommandState(payload);
    renderStatus();
    saveConfig();
    addLog(`[MOCK] ${kind} -> ${label}`);
    return { ok: true, mock: true };
  }

  if (!state.apiBase) {
    addLog('API base URL is missing. Command was not sent.', 'error');
    return { ok: false, error: 'missing-api-base' };
  }

  try {
    const response = await fetch(`${state.apiBase}${kind}`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });

    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }

    applyCommandState(payload);
    renderStatus();
    saveConfig();
    addLog(`[API] ${kind} -> ${label}`);

    return await response.json().catch(() => ({ ok: true }));
  } catch (error) {
    state.connected = false;
    renderStatus();
    addLog(`Command failed: ${error.message}`, 'error');
    return { ok: false, error: error.message };
  }
}

async function refreshStatus() {
  if (state.mockMode) {
    applyMockScenario();
    addLog('Mock telemetry refreshed.');
    return;
  }

  if (!state.apiBase) {
    addLog('API base URL is missing. Cannot refresh robot status.', 'error');
    return;
  }

  try {
    const response = await fetch(`${state.apiBase}/status`);
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }

    const data = await response.json();
    state.mode = data.mode ?? state.mode;
    state.target = data.target ?? state.target;
    state.distance = data.distance ?? state.distance;
    state.behavior = data.behavior ?? state.behavior;
    state.battery = clampBattery(Number(data.battery ?? state.battery));
    state.connected = data.connected ?? true;
    state.robotName = String(data.robotName ?? state.robotName).toUpperCase();

    renderStatus();
    addLog('Status refreshed from API.');
  } catch (error) {
    state.connected = false;
    renderStatus();
    addLog(`Status refresh failed: ${error.message}`, 'error');
  }
}

function bindEvents() {
  els.streamImage.addEventListener('load', () => {
    if (state.mockMode) return;
    state.streamError = false;
    state.streamReady = true;
    state.connected = true;
    els.streamImage.hidden = false;
    renderStatus();
  });

  els.streamImage.addEventListener('error', () => {
    if (state.mockMode || !state.streamUrl) return;
    state.streamError = true;
    state.streamReady = false;
    state.connected = false;
    els.streamImage.hidden = true;
    renderStatus();
    addLog('Video stream failed to load. Showing fallback deck.', 'error');
  });

  els.applyStreamBtn.addEventListener('click', () => {
    state.streamUrl = els.streamUrlInput.value.trim();
    state.streamError = false;
    state.streamReady = false;
    els.streamImage.dataset.source = '';
    saveConfig();
    renderStatus(true);
    addLog(state.streamUrl ? `Stream URL updated: ${state.streamUrl}` : 'Stream URL cleared.');
  });

  els.mockToggle.addEventListener('click', () => {
    state.mockMode = !state.mockMode;
    state.streamError = false;
    state.streamReady = false;
    saveConfig();

    if (state.mockMode) {
      startMockLoop();
      state.connected = true;
    } else {
      stopMockLoop();
    }

    renderStatus(true);
    addLog(`Mock mode ${state.mockMode ? 'enabled' : 'disabled'}.`);
  });

  els.mobileNavButtons.forEach((button) => {
    button.addEventListener('click', () => {
      const nextView = button.dataset.mobileView || 'vision';
      state.mobileView = MOBILE_VIEWS.includes(nextView) ? nextView : 'vision';
      renderMobileView();
      saveConfig();
    });
  });

  $$('.motion').forEach((button) => {
    button.addEventListener('click', () => {
      sendCommand('/control/move', { command: button.dataset.command });
    });
  });

  $$('.segment').forEach((button) => {
    button.addEventListener('click', () => {
      sendCommand('/control/mode', { mode: button.dataset.mode });
    });
  });

  $$('[data-action]').forEach((button) => {
    button.addEventListener('click', () => {
      sendCommand('/control/action', { action: button.dataset.action });
    });
  });

  $$('[data-system]').forEach((button) => {
    if (button.dataset.system === 'shutdown') return;
    button.addEventListener('click', () => {
      sendCommand('/system', { system: button.dataset.system });
    });
  });

  els.refreshBtn.addEventListener('click', refreshStatus);
  els.refreshQuickBtn.addEventListener('click', refreshStatus);

  els.clearLogBtn.addEventListener('click', () => {
    state.log = [];
    renderLog();
  });

  let holdTimer = null;
  let holdProgressTimer = null;
  let holdStart = 0;

  const resetHold = () => {
    clearTimeout(holdTimer);
    clearInterval(holdProgressTimer);
    holdTimer = null;
    holdProgressTimer = null;
    els.holdBar.style.width = '0%';
  };

  const beginHold = () => {
    holdStart = Date.now();

    holdTimer = setTimeout(() => {
      sendCommand('/system', { system: 'shutdown' });
      addLog('Shutdown command sent.');
      resetHold();
    }, HOLD_DURATION);

    holdProgressTimer = setInterval(() => {
      const elapsed = Date.now() - holdStart;
      const progress = Math.min(100, (elapsed / HOLD_DURATION) * 100);
      els.holdBar.style.width = `${progress}%`;
    }, 30);
  };

  ['pointerup', 'pointerleave', 'pointercancel'].forEach((eventName) => {
    els.shutdownBtn.addEventListener(eventName, resetHold);
  });

  els.shutdownBtn.addEventListener('pointerdown', beginHold);
}

function init() {
  loadConfig();
  bindEvents();
  renderMobileView();
  renderStatus(true);
  renderLog();
  addLog('Dashboard ready.');

  if (state.mockMode) {
    startMockLoop();
  }
}

init();
