// WebXR page for the RoArm-M3 Quest 2 controller.
//
// Streams the pose and buttons of both controllers to the Python server over
// a WebSocket, and draws a small marker on each controller so you can see
// what the server is doing (grey = idle, green = following, orange = at a
// limit, blue = reading the arm's position, purple = playing a recording,
// red = no connection; the other controller's marker is red while recording).
//
// The page also lists the recordings and can record and play them, so it is
// useful on the computer too (it doesn't take over the headset's control).

"use strict";

const $ = (id) => document.getElementById(id);

// ---------------------------------------------------------------------------
// Connection to the server
// ---------------------------------------------------------------------------
let ws = null;
let status = null;        // latest status message from the server
let inputSources = [];    // current XR input sources (for haptics)

function connect() {
  const scheme = location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${scheme}://${location.host}/ws`);
  ws.onopen = () => {
    setConn("connected", "ok");
    if (session) send({ type: "session", state: "start", mode: sessionMode }); // after a server restart
  };
  ws.onclose = () => {
    status = null;
    setConn("disconnected, retrying...", "bad");
    setTimeout(connect, 1000);
  };
  ws.onmessage = (event) => {
    let data;
    try { data = JSON.parse(event.data); } catch { return; }
    for (const msg of Array.isArray(data) ? data : [data]) {
      if (msg.type === "status") {
        status = msg;
        showStatus(msg);
      } else if (msg.type === "haptic") {
        pulse(msg.hand, msg.intensity, msg.ms);
      } else if (msg.type === "recordings") {
        showRecordings(msg.items);
      } else if (msg.type === "notice") {
        $("message").textContent = msg.message;
      }
    }
  };
}

function send(obj) {
  // Skip updates if the network is backed up, so the server always gets fresh data.
  if (ws && ws.readyState === WebSocket.OPEN && ws.bufferedAmount < 16384) {
    ws.send(JSON.stringify(obj));
  }
}

function setConn(text, cls) {
  $("conn").textContent = text;
  $("conn").className = cls;
}

function showStatus(s) {
  $("link").textContent = s.link;
  let state = "idle (hold the grip to move)";
  let cls = "";
  if (s.syncing) { state = "reading arm position..."; cls = "warn"; }
  else if (s.engaged && s.limited) { state = "following, at a limit"; cls = "warn"; }
  else if (s.engaged) { state = "following the controller"; cls = "ok"; }
  $("state").textContent = state;
  $("state").className = cls;
  $("orientation").textContent = s.orientation ? "following" : "locked";
  const t = s.target;
  $("target").textContent =
    `x ${t.x.toFixed(0)}  y ${t.y.toFixed(0)}  z ${t.z.toFixed(0)} mm  ` +
    `pitch ${t.t.toFixed(2)}  roll ${t.r.toFixed(2)}  grip ${t.g.toFixed(2)} rad`;
  $("message").textContent = s.message;
  $("trigger-help").textContent = s.gripperMode === "toggle"
    ? "Pull to open or close the gripper."
    : "Closes the gripper (pull further = tighter).";
  const hand = s.hand.charAt(0).toUpperCase() + s.hand.slice(1);
  const other = s.otherHand.charAt(0).toUpperCase() + s.otherHand.slice(1);
  for (const el of document.querySelectorAll(".hand-cap")) el.textContent = hand;
  for (const el of document.querySelectorAll(".hand-low")) el.textContent = s.hand;
  $("key-home").textContent = `${hand} ${s.buttons.home}`;
  $("key-orientation").textContent = `${hand} ${s.buttons.orientation}`;
  $("key-record").textContent = `${other} ${s.buttons.record}`;
  $("key-play").textContent = `${other} ${s.buttons.play}`;
  showRecordingStatus(s);
}

// ---------------------------------------------------------------------------
// Recordings panel
// ---------------------------------------------------------------------------
let recordingItems = [];

function showRecordings(items) {
  recordingItems = items;
  const list = $("rec-list");
  list.textContent = "";
  if (!items.length) {
    list.textContent = "No recordings yet.";
    return;
  }
  for (const item of items) {
    const label = document.createElement("label");
    const radio = document.createElement("input");
    radio.type = "radio";
    radio.name = "recording";
    radio.value = item.name;
    radio.disabled = !!item.error;
    radio.checked = !!(status && status.selected.name === item.name);
    radio.addEventListener("change", sendSelection);
    label.append(radio, ` ${item.name} `);
    const info = document.createElement("span");
    info.className = item.error ? "bad" : "label";
    info.textContent = item.error ? `can't play: ${item.error}` : `${item.duration} s, ${item.commands} commands`;
    label.append(info);
    list.append(label);
  }
}

function selectedName() {
  const checked = document.querySelector('input[name="recording"]:checked');
  return checked ? checked.value : null;
}

function loopCount() {
  const n = parseInt($("loops").value, 10);
  return Number.isFinite(n) && n >= 0 ? n : 1;
}

function sendSelection() {
  send({ type: "select", name: selectedName(), loops: loopCount() });
}

function showRecordingStatus(s) {
  const recordButton = $("record");
  if (s.recording) {
    recordButton.textContent = "Stop recording and save";
    recordButton.className = "recording";
  } else {
    recordButton.textContent = "Start recording";
    recordButton.className = "secondary";
  }
  $("play").disabled = !!(s.playback || s.recording);
  $("stop").disabled = !s.playback;

  let text = "";
  let cls = "";
  if (s.recording) {
    text = `REC  ${s.recording.elapsed.toFixed(1)} s, ${s.recording.commands} commands`;
    cls = "bad";
  } else if (s.playback) {
    const p = s.playback;
    const total = p.loops === 0 ? "forever" : `of ${p.loops}`;
    const where = p.phase === "play" ? `${p.elapsed.toFixed(1)} / ${p.duration.toFixed(1)} s`
      : p.phase === "reading" ? "reading the arm's position" : "moving to the start position";
    text = `Playing ${p.name}, loop ${p.loop} ${total}: ${where}`;
    cls = "ok";
  }
  $("rec-status").textContent = text;
  $("rec-status").className = cls;

  // Show the selection made on the headset or another page.
  for (const radio of document.querySelectorAll('input[name="recording"]')) {
    radio.checked = radio.value === s.selected.name;
  }
  if (document.activeElement !== $("loops")) $("loops").value = s.selected.loops;
}

$("record").addEventListener("click", () => send({ type: "record" }));
$("play").addEventListener("click", () => send({ type: "play", name: selectedName(), loops: loopCount() }));
$("stop").addEventListener("click", () => send({ type: "stop" }));
$("refresh").addEventListener("click", () => send({ type: "list" }));
$("loops").addEventListener("change", sendSelection);

function pulse(hand, intensity, ms) {
  for (const source of inputSources) {
    if (source.handedness !== hand || !source.gamepad) continue;
    const actuator = source.gamepad.hapticActuators && source.gamepad.hapticActuators[0];
    if (actuator && actuator.pulse) {
      actuator.pulse(intensity, ms).catch(() => {});
    } else if (source.gamepad.vibrationActuator) {
      source.gamepad.vibrationActuator.playEffect("dual-rumble",
        { duration: ms, strongMagnitude: intensity, weakMagnitude: intensity }).catch(() => {});
    }
  }
}

// ---------------------------------------------------------------------------
// Drawing the controller markers (plain WebGL, no libraries)
// ---------------------------------------------------------------------------
const VERTEX_SHADER = `
attribute vec3 aPos;
attribute vec3 aNormal;
uniform mat4 uProj, uView, uModel;
varying float vLight;
void main() {
  vec3 n = normalize((uModel * vec4(aNormal, 0.0)).xyz);
  vLight = 0.55 + 0.45 * max(dot(n, normalize(vec3(0.3, 1.0, 0.5))), 0.0);
  gl_Position = uProj * uView * uModel * vec4(aPos, 1.0);
}`;
const FRAGMENT_SHADER = `
precision mediump float;
uniform vec3 uColor;
varying float vLight;
void main() { gl_FragColor = vec4(uColor * vLight, 1.0); }`;

const COLORS = {
  idle: [0.6, 0.6, 0.65],
  engaged: [0.3, 0.9, 0.4],
  limited: [1.0, 0.6, 0.2],
  syncing: [0.35, 0.6, 1.0],
  playing: [0.7, 0.4, 1.0],
  recording: [1.0, 0.2, 0.3],
  offline: [1.0, 0.25, 0.25],
  other: [0.3, 0.3, 0.33],
};

let gl = null;
let program = null;
let cube = null;

function compile(type, source) {
  const shader = gl.createShader(type);
  gl.shaderSource(shader, source);
  gl.compileShader(shader);
  if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(shader));
  return shader;
}

function initGL() {
  program = gl.createProgram();
  gl.attachShader(program, compile(gl.VERTEX_SHADER, VERTEX_SHADER));
  gl.attachShader(program, compile(gl.FRAGMENT_SHADER, FRAGMENT_SHADER));
  gl.linkProgram(program);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(program));

  // Unit cube (-0.5..0.5), 4 vertices per face so each face has its own normal.
  const faces = [
    [[1, 0, 0], [0, 1, 0], [0, 0, 1]], [[-1, 0, 0], [0, 1, 0], [0, 0, -1]],
    [[0, 1, 0], [0, 0, 1], [1, 0, 0]], [[0, -1, 0], [0, 0, -1], [1, 0, 0]],
    [[0, 0, 1], [1, 0, 0], [0, 1, 0]], [[0, 0, -1], [-1, 0, 0], [0, 1, 0]],
  ];
  const data = [];
  const indices = [];
  faces.forEach(([n, u, v], f) => {
    for (const [su, sv] of [[-1, -1], [1, -1], [1, 1], [-1, 1]]) {
      for (let i = 0; i < 3; i++) data.push(0.5 * (n[i] + su * u[i] + sv * v[i]));
      data.push(...n);
    }
    const b = f * 4;
    indices.push(b, b + 1, b + 2, b, b + 2, b + 3);
  });
  cube = { vbo: gl.createBuffer(), ibo: gl.createBuffer(), count: indices.length };
  gl.bindBuffer(gl.ARRAY_BUFFER, cube.vbo);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(data), gl.STATIC_DRAW);
  gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, cube.ibo);
  gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, new Uint16Array(indices), gl.STATIC_DRAW);
}

// 4x4 column-major matrix product a * b.
function mul(a, b) {
  const out = new Float32Array(16);
  for (let c = 0; c < 4; c++) {
    for (let r = 0; r < 4; r++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k];
      out[c * 4 + r] = s;
    }
  }
  return out;
}

// Translation then scale, as one matrix.
function boxMatrix(tx, ty, tz, sx, sy, sz) {
  return new Float32Array([sx, 0, 0, 0, 0, sy, 0, 0, 0, 0, sz, 0, tx, ty, tz, 1]);
}

// A marker on the controller: a small block with a "nose" pointing where the controller points.
const MARKER_PARTS = [
  boxMatrix(0, 0, 0, 0.03, 0.03, 0.03),
  boxMatrix(0, 0, -0.05, 0.012, 0.012, 0.07),
];

function markerColor(handedness) {
  if (!ws || ws.readyState !== WebSocket.OPEN || !status) return COLORS.offline;
  if (status.playback) return COLORS.playing;
  if (handedness !== status.hand) return status.recording ? COLORS.recording : COLORS.other;
  if (status.syncing) return COLORS.syncing;
  if (status.engaged) return status.limited ? COLORS.limited : COLORS.engaged;
  return COLORS.idle;
}

function drawMarkers(viewerPose, markers, layer) {
  gl.useProgram(program);
  gl.bindBuffer(gl.ARRAY_BUFFER, cube.vbo);
  gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, cube.ibo);
  const aPos = gl.getAttribLocation(program, "aPos");
  const aNormal = gl.getAttribLocation(program, "aNormal");
  gl.enableVertexAttribArray(aPos);
  gl.enableVertexAttribArray(aNormal);
  gl.vertexAttribPointer(aPos, 3, gl.FLOAT, false, 24, 0);
  gl.vertexAttribPointer(aNormal, 3, gl.FLOAT, false, 24, 12);
  const uProj = gl.getUniformLocation(program, "uProj");
  const uView = gl.getUniformLocation(program, "uView");
  const uModel = gl.getUniformLocation(program, "uModel");
  const uColor = gl.getUniformLocation(program, "uColor");

  for (const view of viewerPose.views) {
    const vp = layer.getViewport(view);
    gl.viewport(vp.x, vp.y, vp.width, vp.height);
    gl.uniformMatrix4fv(uProj, false, view.projectionMatrix);
    gl.uniformMatrix4fv(uView, false, view.transform.inverse.matrix);
    for (const { matrix, color } of markers) {
      gl.uniform3fv(uColor, color);
      for (const part of MARKER_PARTS) {
        gl.uniformMatrix4fv(uModel, false, mul(matrix, part));
        gl.drawElements(gl.TRIANGLES, cube.count, gl.UNSIGNED_SHORT, 0);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// XR session
// ---------------------------------------------------------------------------
let session = null;
let refSpace = null;
let sessionMode = null;

const vec = (p) => [p.x, p.y, p.z];
const quat = (q) => [q.x, q.y, q.z, q.w];

// Button layout of the "xr-standard" gamepad mapping used by Quest controllers.
const BUTTON = { trigger: 0, squeeze: 1, thumbstick: 3, a: 4, b: 5 };

function buttonValue(gamepad, index) {
  const b = gamepad.buttons[index];
  return b ? (b.value || (b.pressed ? 1 : 0)) : 0;
}

function buttonPressed(gamepad, index) {
  const b = gamepad.buttons[index];
  return !!(b && b.pressed);
}

async function startSession(mode) {
  if (session) return;
  try {
    session = await navigator.xr.requestSession(mode, { optionalFeatures: ["local-floor"] });
  } catch (e) {
    $("message").textContent = `Could not start: ${e.message}`;
    return;
  }
  sessionMode = mode;
  const canvas = document.createElement("canvas");
  gl = canvas.getContext("webgl", { xrCompatible: true, alpha: true });
  initGL();
  const layer = new XRWebGLLayer(session, gl);
  session.updateRenderState({ baseLayer: layer });
  try {
    refSpace = await session.requestReferenceSpace("local-floor");
  } catch {
    refSpace = await session.requestReferenceSpace("local");
  }
  session.addEventListener("end", () => {
    send({ type: "session", state: "end" });
    session = null;
    inputSources = [];
    setButtons(true);
  });
  setButtons(false);
  send({ type: "session", state: "start", mode });
  session.requestAnimationFrame(onFrame);
}

function onFrame(time, frame) {
  const s = frame.session;
  s.requestAnimationFrame(onFrame);
  const layer = s.renderState.baseLayer;
  inputSources = Array.from(s.inputSources);

  const viewerPose = frame.getViewerPose(refSpace);
  const hands = {};
  const markers = [];
  for (const source of inputSources) {
    if (!source.gamepad || !source.gripSpace || (source.handedness !== "left" && source.handedness !== "right")) {
      continue; // e.g. hand tracking without controllers
    }
    const grip = frame.getPose(source.gripSpace, refSpace);
    const ray = frame.getPose(source.targetRaySpace, refSpace);
    if (!grip || !ray) continue; // not tracked this frame
    const gp = source.gamepad;
    hands[source.handedness] = {
      pos: vec(grip.transform.position),
      quat: quat(ray.transform.orientation),
      trigger: buttonValue(gp, BUTTON.trigger),
      grip: buttonValue(gp, BUTTON.squeeze),
      a: buttonPressed(gp, BUTTON.a),
      b: buttonPressed(gp, BUTTON.b),
    };
    markers.push({ matrix: ray.transform.matrix, color: markerColor(source.handedness) });
  }

  send({
    type: "input",
    head: viewerPose ? { pos: vec(viewerPose.transform.position), quat: quat(viewerPose.transform.orientation) } : null,
    hands,
  });

  gl.bindFramebuffer(gl.FRAMEBUFFER, layer.framebuffer);
  if (sessionMode === "immersive-ar") {
    gl.clearColor(0, 0, 0, 0); // transparent: show the passthrough camera
  } else {
    gl.clearColor(0.08, 0.09, 0.1, 1);
  }
  gl.enable(gl.DEPTH_TEST);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  if (viewerPose) drawMarkers(viewerPose, markers, layer);
}

// ---------------------------------------------------------------------------
// Start-up
// ---------------------------------------------------------------------------
const support = { "immersive-ar": false, "immersive-vr": false };

function setButtons(enabled) {
  $("start-ar").disabled = !(enabled && support["immersive-ar"]);
  $("start-vr").disabled = !(enabled && support["immersive-vr"]);
}

async function checkSupport() {
  if (!window.isSecureContext) {
    $("xr-support").textContent = "This page must be opened over https (or http://localhost) for WebXR to work.";
    return;
  }
  if (!navigator.xr) {
    $("xr-support").textContent = "WebXR is not available in this browser. Open this page in the Quest Browser.";
    return;
  }
  for (const mode of Object.keys(support)) {
    try { support[mode] = await navigator.xr.isSessionSupported(mode); } catch { support[mode] = false; }
  }
  if (!support["immersive-ar"] && !support["immersive-vr"]) {
    $("xr-support").textContent = "No VR headset found. Open this page in the Quest Browser.";
  } else if (!support["immersive-ar"]) {
    $("xr-support").textContent = "Passthrough is not available; use Start (VR).";
  }
  setButtons(true);
}

$("start-ar").addEventListener("click", () => startSession("immersive-ar"));
$("start-vr").addEventListener("click", () => startSession("immersive-vr"));
connect();
checkSupport();
