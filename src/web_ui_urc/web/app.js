/*
  LUSI rover browser interface: connects to ROS through rosbridge and drives the Drive and Science pages

  Runs in: any browser (Steam Deck or laptop) that opened http://<base-station-ip>:8000
  Talks to (all started by web_ui_urc/launch/web_ui_launch.py on the base station):
    rosbridge_websocket  ws://<base-station-ip>:9090   ROS topics as JSON
    web_video_server     http://<base-station-ip>:8080 camera topics as MJPEG video

  Topics read (each one already exists in the code):
    /cmd_vel (geometry_msgs/Twist)                    JoyMapper / WaypointFollower drive command
    /roverDriveCommands (RoverComputerDriveCMD)        DriveTrainManager per-wheel speeds, rad/s
    /gps_data (GPSData)                                GPS fix (only the simulator publishes it today)
    /science/weight_g, /science/soil/<name> (Float32)  Science Pi sensors
    /science/spectrometer/intensity (Float32MultiArray) raw spectrometer intensity per pixel
    /web_ui/snapshot_status (String)                   result of "Save snapshot" (science_recorder)
  Topics written:
    /science/auger_speed (Float32, 0-100 %)            science_node stops the auger if this stops for 2 s
    /science/platform_command (String "up,50")         science_node stops the platform if this stops for 2 s
    /science/soil_drop_angle, /science/chamber_lid_angle (Float32, 0-180 deg)
    /web_ui/save_snapshot (String note)                asks science_recorder to save files
*/

'use strict';

// ---------------------------------------------------------------------------
// Settings
// ---------------------------------------------------------------------------

// The page is served by the base station, so the other servers live on the same host
const HOST = window.location.hostname || 'localhost';
const ROSBRIDGE_URL = `ws://${HOST}:9090`;

// Building a web_video_server stream URL; ros_compressed forwards the JPEG frames as they are
// (no re-encoding on the base station, and only compressed frames cross the radio)
const videoUrl = (topic) => `http://${HOST}:8080/stream?topic=${topic}&type=ros_compressed`;

// Camera topic shown on each page (null means the page has no camera)
const PAGE_VIDEO = { drive: '/video_stream', science: '/science/camera/image_raw' };

// How often a held motor button resends its command, in ms
// (science_node stops a motor after 2000 ms without a command, so this must stay well under that)
const REPEAT_MS = 500;

// A value is dimmed when it hasn't updated for this long, in ms
// The soil probe reports about every 5 s (longer when it retries), so it gets more time
const STALE_MS = 3000;
const STALE_MS_SOIL = 15000;

const SOIL_FIELDS = ['moisture', 'temperature', 'ph', 'conductivity', 'nitrogen', 'phosphorus',
  'potassium'];

// Decimal places shown for each value
const DIGITS = {
  cmd_forward: 2, cmd_turn: 1, gps_lat: 6, gps_lon: 6, gps_sats: 0, weight_g: 1,
  moisture: 1, temperature: 1, ph: 2, conductivity: 0, nitrogen: 0, phosphorus: 0, potassium: 0,
};

// ---------------------------------------------------------------------------
// Connection to ROS
// ---------------------------------------------------------------------------

let ros = null;
let connected = false;
const publishers = {};

/** Open the rosbridge connection; on close, retry every 2 s until it comes back. */
function connect() {
  ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });

  ros.on('connection', () => {
    setConnected(true);
    subscribeAll();
    advertiseAll();
  });

  // 'error' is always followed by 'close', so reconnecting only in 'close' avoids doing it twice
  ros.on('error', () => {});
  ros.on('close', () => {
    setConnected(false);
    stopAllHolds();
    setTimeout(connect, 2000);
  });
}

/** Update the header badge and enable or disable every control. */
function setConnected(isConnected) {
  connected = isConnected;
  const badge = document.getElementById('connection');
  badge.textContent = isConnected ? 'Connected' : 'Disconnected';
  badge.className = 'badge ' + (isConnected ? 'online' : 'offline');
  document.querySelectorAll('.controls button, .controls input').forEach((el) => {
    el.disabled = !isConnected;
  });
}

/** Subscribe to a topic; throttleMs > 0 asks rosbridge to send at most one message per period. */
function subscribe(name, messageType, callback, throttleMs = 0) {
  new ROSLIB.Topic({ ros, name, messageType, throttle_rate: throttleMs }).subscribe(callback);
}

/** Create a publisher once per connection, stored under its topic name. */
function advertise(name, messageType) {
  publishers[name] = new ROSLIB.Topic({ ros, name, messageType });
}

/** Publish if connected; messages are silently dropped while disconnected. */
function publish(name, data) {
  if (connected && publishers[name]) {
    publishers[name].publish(new ROSLIB.Message(data));
  }
}

function subscribeAll() {
  // Drive page
  subscribe('/cmd_vel', 'geometry_msgs/msg/Twist', (m) => {
    setValue('cmd_forward', m.linear.x);
    // DriveTrainManager and JoyMapper put the turn rate in angular.y, in deg/s
    setValue('cmd_turn', m.angular.y);
  });
  subscribe('/roverDriveCommands', 'cross_pkg_messages/msg/RoverComputerDriveCMD', (m) => {
    // In this message x/y/z mean front/middle/back wheel
    setValue('wheel_lf', m.cmd_l.x, 2); setValue('wheel_lm', m.cmd_l.y, 2);
    setValue('wheel_lb', m.cmd_l.z, 2); setValue('wheel_rf', m.cmd_r.x, 2);
    setValue('wheel_rm', m.cmd_r.y, 2); setValue('wheel_rb', m.cmd_r.z, 2);
  });
  subscribe('/gps_data', 'cross_pkg_messages/msg/GPSData', (m) => {
    // In GPSData, lla.x is latitude and lla.y is longitude
    setValue('gps_lat', m.lla.x);
    setValue('gps_lon', m.lla.y);
    setValue('gps_sats', m.sats);
  });

  // Science page
  subscribe('/science/weight_g', 'std_msgs/msg/Float32', (m) => setValue('weight_g', m.data));
  SOIL_FIELDS.forEach((name) => {
    subscribe(`/science/soil/${name}`, 'std_msgs/msg/Float32', (m) => setValue(name, m.data));
  });
  subscribe('/science/spectrometer/intensity', 'std_msgs/msg/Float32MultiArray',
    (m) => drawSpectrum(m.data), 200);
  subscribe('/web_ui/snapshot_status', 'std_msgs/msg/String', (m) => {
    document.getElementById('snapshot-status').textContent = m.data;
  });
}

function advertiseAll() {
  advertise('/science/auger_speed', 'std_msgs/msg/Float32');
  advertise('/science/platform_command', 'std_msgs/msg/String');
  advertise('/science/soil_drop_angle', 'std_msgs/msg/Float32');
  advertise('/science/chamber_lid_angle', 'std_msgs/msg/Float32');
  advertise('/web_ui/save_snapshot', 'std_msgs/msg/String');
}

// ---------------------------------------------------------------------------
// Readings
// ---------------------------------------------------------------------------

// Time each value last changed (ms since page load), used to dim old values
const lastUpdate = {};

/** Show a number in every element tagged data-value="<key>" and mark it fresh. */
function setValue(key, value, digits = DIGITS[key] ?? 2) {
  const text = Number.isFinite(value) ? value.toFixed(digits) : '--';
  document.querySelectorAll(`[data-value="${key}"]`).forEach((el) => {
    el.textContent = text;
    el.classList.remove('stale');
  });
  lastUpdate[key] = performance.now();
}

/** Dim every value that hasn't updated recently (runs once a second). */
function markStale() {
  const now = performance.now();
  document.querySelectorAll('[data-value]').forEach((el) => {
    const key = el.dataset.value;
    const limit = SOIL_FIELDS.includes(key) ? STALE_MS_SOIL : STALE_MS;
    const old = !(key in lastUpdate) || now - lastUpdate[key] > limit;
    el.classList.toggle('stale', old);
  });
}

/** Draw the raw spectrum as a line: x is pixel index, y is intensity scaled to the tallest point. */
function drawSpectrum(values) {
  const canvas = document.getElementById('spectrum');
  // Matching the canvas's pixel size to its on-screen size so the line stays sharp
  const scale = window.devicePixelRatio || 1;
  canvas.width = canvas.clientWidth * scale;
  canvas.height = canvas.clientHeight * scale;
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  if (!values || values.length < 2) return;

  const peak = Math.max(...values, 1);
  const pad = 6 * scale;
  ctx.strokeStyle = '#4da3ff';
  ctx.lineWidth = 1.5 * scale;
  ctx.beginPath();
  values.forEach((v, i) => {
    const x = pad + (i / (values.length - 1)) * (canvas.width - 2 * pad);
    const y = canvas.height - pad - (v / peak) * (canvas.height - 2 * pad);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();

  ctx.fillStyle = '#8b95a0';
  ctx.font = `${12 * scale}px system-ui, sans-serif`;
  ctx.fillText(`max ${peak.toFixed(0)}  |  ${values.length} pixels`, pad, 14 * scale);
}

// ---------------------------------------------------------------------------
// Science controls
// ---------------------------------------------------------------------------

// Only one hold-to-run button may be active at a time (stops two motors fighting on multi-touch)
let activeStop = null;
const allStops = [];

/**
 * Make a button run a motor only while it is held (mouse or touch).
 * sendRun is called immediately and then every REPEAT_MS; sendStop is called once on release.
 */
function makeHoldButton(button, sendRun, sendStop) {
  let timer = null;

  const stop = () => {
    if (timer === null) return;
    clearInterval(timer);
    timer = null;
    activeStop = null;
    button.classList.remove('active');
    sendStop();
  };

  button.addEventListener('pointerdown', (event) => {
    if (!connected || activeStop !== null) return;
    event.preventDefault();
    // Keeping pointer events on this button even if the finger slides off it
    // (wrapped in try because some browsers refuse capture for unusual pointer types)
    try { button.setPointerCapture(event.pointerId); } catch (err) { /* release still stops it */ }
    button.classList.add('active');
    activeStop = stop;
    sendRun();
    timer = setInterval(sendRun, REPEAT_MS);
  });

  // Any way the press can end stops the motor
  ['pointerup', 'pointercancel', 'lostpointercapture'].forEach((type) => {
    button.addEventListener(type, stop);
  });
  // Stopping the long-press menu on touch screens
  button.addEventListener('contextmenu', (event) => event.preventDefault());
  allStops.push(stop);
}

/** Release every held button (used on disconnect, tab switch, and when the page is hidden). */
function stopAllHolds() {
  allStops.forEach((stop) => stop());
}

/** Read a slider's current value as a number. */
const sliderValue = (id) => Number(document.getElementById(id).value);

function setupControls() {
  // Showing each slider's value next to its label
  document.querySelectorAll('input[type=range]').forEach((slider) => {
    const out = document.querySelector(`output[data-for="${slider.id}"]`);
    slider.addEventListener('input', () => { out.textContent = slider.value; });
  });

  // The speed is read on every repeat, so moving the slider while holding changes the speed
  makeHoldButton(document.getElementById('auger-run'),
    () => publish('/science/auger_speed', { data: sliderValue('auger-speed') }),
    () => publish('/science/auger_speed', { data: 0 }));

  // science_node expects "direction,speed"; speed 0 stops the platform whichever direction is given
  makeHoldButton(document.getElementById('platform-up'),
    () => publish('/science/platform_command', { data: `up,${sliderValue('platform-speed')}` }),
    () => publish('/science/platform_command', { data: 'up,0' }));
  makeHoldButton(document.getElementById('platform-down'),
    () => publish('/science/platform_command', { data: `down,${sliderValue('platform-speed')}` }),
    () => publish('/science/platform_command', { data: 'down,0' }));

  // Servos hold their position, so these send once per press
  document.getElementById('drop-move').addEventListener('click',
    () => publish('/science/soil_drop_angle', { data: sliderValue('drop-angle') }));
  document.getElementById('lid-move').addEventListener('click',
    () => publish('/science/chamber_lid_angle', { data: sliderValue('lid-angle') }));

  document.getElementById('stop-motors').addEventListener('click', () => {
    stopAllHolds();
    publish('/science/auger_speed', { data: 0 });
    publish('/science/platform_command', { data: 'up,0' });
  });

  document.getElementById('snapshot-save').addEventListener('click', () => {
    const note = document.getElementById('snapshot-note').value;
    document.getElementById('snapshot-status').textContent = 'Saving...';
    publish('/web_ui/save_snapshot', { data: note });
  });

  // Stopping motors if the browser loses focus or the screen turns off mid-press
  window.addEventListener('blur', stopAllHolds);
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) stopAllHolds();
  });
}

// ---------------------------------------------------------------------------
// Pages and video
// ---------------------------------------------------------------------------

/** Show one page, and stream only that page's camera (hidden pages close their stream). */
function showPage(page) {
  if (!(page in PAGE_VIDEO)) page = 'drive';
  stopAllHolds();
  document.querySelectorAll('.tab').forEach((tab) => {
    tab.classList.toggle('active', tab.dataset.page === page);
  });
  document.querySelectorAll('.page').forEach((el) => {
    el.classList.toggle('active', el.id === `page-${page}`);
  });
  Object.entries(PAGE_VIDEO).forEach(([name, topic]) => {
    const img = document.getElementById(`${name}-video`);
    if (name === page && topic) startVideo(img, topic); else stopVideo(img);
  });
  // Remembering the page in the address, so a reload or bookmark opens the same page
  history.replaceState(null, '', `#${page}`);
}

function startVideo(img, topic) {
  const frame = img.parentElement;
  img.onload = () => frame.classList.add('live');
  // Retrying every 3 s while the page is open (the camera or the server may start later)
  img.onerror = () => {
    frame.classList.remove('live');
    setTimeout(() => { if (img.dataset.topic === topic) img.src = videoUrl(topic); }, 3000);
  };
  img.dataset.topic = topic;
  img.src = videoUrl(topic);
}

function stopVideo(img) {
  img.dataset.topic = '';
  img.onerror = null;
  img.removeAttribute('src');
  img.parentElement.classList.remove('live');
}

// ---------------------------------------------------------------------------
// Start
// ---------------------------------------------------------------------------

document.querySelectorAll('.tab').forEach((tab) => {
  tab.addEventListener('click', () => showPage(tab.dataset.page));
});
setupControls();
setConnected(false);
showPage(window.location.hash.slice(1));
setInterval(markStale, 1000);
markStale();
connect();
