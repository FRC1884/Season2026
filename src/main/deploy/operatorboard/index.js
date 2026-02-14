import { NT4_Client } from "./NT4.js";

const defaultNt4Port = 5810;

// REBUILT field nominal dimensions from the manual (Section 5.2).
const FIELD_LENGTH_METERS = 16.541;
const FIELD_WIDTH_METERS = 8.069;

const STALE_MS = 1500;

const contract = {
  base: "/OperatorBoard/v1",
  toRobot: "/OperatorBoard/v1/ToRobot/",
  toDashboard: "/OperatorBoard/v1/ToDashboard/",
  keys: {
    requestedState: "RequestedState",
    autoStateEnable: "AutoStateEnable",
    playSwerveMusic: "PlaySwerveMusic",
    currentState: "CurrentState",
    requestAccepted: "RequestAccepted",
    requestReason: "RequestReason",
    climbPhase: "ClimbPhase",
    climbLevel: "ClimbLevel",
    targetType: "TargetType",
    targetPose: "TargetPose",
    targetPoseValid: "TargetPoseValid",
    robotPose: "RobotPose",
    hasBall: "HasBall",
    dsMode: "DsMode",
    batteryVoltage: "BatteryVoltage",
    brownout: "Brownout",
    alliance: "Alliance",
    matchTime: "MatchTime",
    hubTimeframe: "HubTimeframe",
    hubStatusValid: "HubStatusValid",
    redHubStatus: "RedHubStatus",
    blueHubStatus: "BlueHubStatus",
    ourHubStatus: "OurHubStatus",
    ourHubActive: "OurHubActive",
    autoWinnerAlliance: "AutoWinnerAlliance",
    gameDataRaw: "GameDataRaw",
    hubRecommendation: "HubRecommendation",
    turretAtSetpoint: "TurretAtSetpoint",
    turretMode: "TurretMode",
    visionStatus: "VisionStatus",
  },
};

const STATES = [
  "IDLING",
  "INTAKING",
  "SHOOTING",
  "FERRYING",
  "ENDGAME_CLIMB",
  "AUTO_CLIMB",
  "CLIMB_DETACH",
  "TESTING",
];

const state = {
  requestedState: null,
  currentState: null,
  requestAccepted: null,
  requestReason: null,
  climbPhase: null,
  climbLevel: null,
  targetType: null,
  targetPose: null,
  targetPoseValid: false,
  robotPose: null,
  hasBall: null,
  dsMode: null,
  batteryVoltage: null,
  brownout: null,
  alliance: null,
  matchTime: null,
  hubTimeframe: null,
  hubStatusValid: null,
  redHubStatus: null,
  blueHubStatus: null,
  ourHubStatus: null,
  ourHubActive: null,
  autoWinnerAlliance: null,
  gameDataRaw: null,
  hubRecommendation: null,
  turretAtSetpoint: null,
  turretMode: null,
  visionStatus: null,
};

const ui = {
  chipNt: null,
  chipData: null,
  chipMode: null,
  requestedState: null,
  currentState: null,
  requestStatus: null,
  requestReason: null,
  alliance: null,
  matchTime: null,
  hubTimeframe: null,
  ourHub: null,
  hubRecommendation: null,
  hubFms: null,
  battery: null,
  brownout: null,
  hasBall: null,
  visionStatus: null,
  turretStatus: null,
  climb: null,
  robotPose: null,
  target: null,
  stateButtons: [],
  autoStateButton: null,
  musicButton: null,
  fieldImage: null,
  fieldCanvas: null,
};

let fieldCtx = null;
let lastAnyData = 0;
let ntConnected = false;

const queryParams = new URLSearchParams(window.location.search);
const { host: ntHost, port: ntPort } = resolveNtConnectionParams(queryParams);

const ntClient = new NT4_Client(
  ntHost,
  "OperatorBoard",
  () => {},
  () => {},
  (topic, _, value) => handleTopicUpdate(topic, value),
  () => {
    ntConnected = true;
    updateChips();
  },
  () => {
    ntConnected = false;
    updateChips();
  },
  { port: ntPort }
);

window.addEventListener("DOMContentLoaded", () => {
  cacheUi();
  buildStateButtons();
  setupFieldCanvas();
  startNetworkTables();
  window.addEventListener("resize", setupFieldCanvas);
  setInterval(() => {
    updateChips();
    render();
  }, 200);
});

function cacheUi() {
  ui.chipNt = document.getElementById("chip-nt");
  ui.chipData = document.getElementById("chip-data");
  ui.chipMode = document.getElementById("chip-mode");
  ui.requestedState = document.getElementById("requested-state");
  ui.currentState = document.getElementById("current-state");
  ui.requestStatus = document.getElementById("request-status");
  ui.requestReason = document.getElementById("request-reason");
  ui.alliance = document.getElementById("alliance");
  ui.matchTime = document.getElementById("match-time");
  ui.hubTimeframe = document.getElementById("hub-timeframe");
  ui.ourHub = document.getElementById("our-hub");
  ui.hubRecommendation = document.getElementById("hub-recommendation");
  ui.hubFms = document.getElementById("hub-fms");
  ui.battery = document.getElementById("battery");
  ui.brownout = document.getElementById("brownout");
  ui.hasBall = document.getElementById("has-ball");
  ui.visionStatus = document.getElementById("vision-status");
  ui.turretStatus = document.getElementById("turret-status");
  ui.climb = document.getElementById("climb");
  ui.robotPose = document.getElementById("robot-pose");
  ui.target = document.getElementById("target");
  ui.fieldImage = document.getElementById("field-image");
  ui.fieldCanvas = document.getElementById("field-canvas");
  ui.autoStateButton = document.getElementById("auto-state-button");
  if (ui.autoStateButton) {
    ui.autoStateButton.addEventListener("click", sendAutoStateEnable);
  }
  ui.musicButton = document.getElementById("music-button");
  if (ui.musicButton) {
    ui.musicButton.addEventListener("click", sendPlaySwerveMusic);
  }
}

function buildStateButtons() {
  const container = document.getElementById("state-buttons");
  if (!container) return;
  container.innerHTML = "";
  ui.stateButtons = STATES.map((name) => {
    const button = document.createElement("button");
    button.className = "state-button";
    button.type = "button";
    button.dataset.state = name;
    button.innerText = prettify(name);
    button.addEventListener("click", () => sendStateRequest(name));
    container.appendChild(button);
    return button;
  });
}

function sendStateRequest(stateName) {
  if (!stateName) return;
  ntClient.addSample(contract.toRobot + contract.keys.requestedState, stateName);
}

function sendAutoStateEnable() {
  ntClient.addSample(contract.toRobot + contract.keys.autoStateEnable, true);
}

function sendPlaySwerveMusic() {
  ntClient.addSample(contract.toRobot + contract.keys.playSwerveMusic, true);
}

function startNetworkTables() {
  const topics = Object.values(contract.keys).map((k) => contract.toDashboard + k);
  ntClient.subscribe(topics, false, false, 0.02);
  ntClient.publishTopic(contract.toRobot + contract.keys.requestedState, "string");
  ntClient.publishTopic(contract.toRobot + contract.keys.autoStateEnable, "boolean");
  ntClient.publishTopic(contract.toRobot + contract.keys.playSwerveMusic, "boolean");
  ntClient.connect();
}

function handleTopicUpdate(topic, value) {
  if (!topic || !topic.name) return;
  lastAnyData = Date.now();

  switch (topic.name) {
    case contract.toDashboard + contract.keys.requestedState:
      state.requestedState = parseString(value);
      break;
    case contract.toDashboard + contract.keys.currentState:
      state.currentState = parseString(value);
      break;
    case contract.toDashboard + contract.keys.requestAccepted:
      state.requestAccepted = !!value;
      break;
    case contract.toDashboard + contract.keys.requestReason:
      state.requestReason = parseString(value);
      break;
    case contract.toDashboard + contract.keys.climbPhase:
      state.climbPhase = parseString(value);
      break;
    case contract.toDashboard + contract.keys.climbLevel:
      state.climbLevel = Number.isFinite(value) ? value : null;
      break;
    case contract.toDashboard + contract.keys.targetType:
      state.targetType = parseString(value);
      break;
    case contract.toDashboard + contract.keys.targetPose:
      state.targetPose = parsePose(value);
      break;
    case contract.toDashboard + contract.keys.targetPoseValid:
      state.targetPoseValid = !!value;
      break;
    case contract.toDashboard + contract.keys.robotPose:
      state.robotPose = parsePose(value);
      break;
    case contract.toDashboard + contract.keys.hasBall:
      state.hasBall = !!value;
      break;
    case contract.toDashboard + contract.keys.dsMode:
      state.dsMode = parseString(value);
      break;
    case contract.toDashboard + contract.keys.batteryVoltage:
      state.batteryVoltage = Number.isFinite(value) ? value : null;
      break;
    case contract.toDashboard + contract.keys.brownout:
      state.brownout = !!value;
      break;
    case contract.toDashboard + contract.keys.alliance:
      state.alliance = parseString(value);
      break;
    case contract.toDashboard + contract.keys.matchTime:
      state.matchTime = Number.isFinite(value) ? value : null;
      break;
    case contract.toDashboard + contract.keys.hubTimeframe:
      state.hubTimeframe = parseString(value);
      break;
    case contract.toDashboard + contract.keys.hubStatusValid:
      state.hubStatusValid = !!value;
      break;
    case contract.toDashboard + contract.keys.redHubStatus:
      state.redHubStatus = parseString(value);
      break;
    case contract.toDashboard + contract.keys.blueHubStatus:
      state.blueHubStatus = parseString(value);
      break;
    case contract.toDashboard + contract.keys.ourHubStatus:
      state.ourHubStatus = parseString(value);
      break;
    case contract.toDashboard + contract.keys.ourHubActive:
      state.ourHubActive = !!value;
      break;
    case contract.toDashboard + contract.keys.autoWinnerAlliance:
      state.autoWinnerAlliance = parseString(value);
      break;
    case contract.toDashboard + contract.keys.gameDataRaw:
      state.gameDataRaw = parseString(value);
      break;
    case contract.toDashboard + contract.keys.hubRecommendation:
      state.hubRecommendation = parseString(value);
      break;
    case contract.toDashboard + contract.keys.turretAtSetpoint:
      state.turretAtSetpoint = !!value;
      break;
    case contract.toDashboard + contract.keys.turretMode:
      state.turretMode = parseString(value);
      break;
    case contract.toDashboard + contract.keys.visionStatus:
      state.visionStatus = parseString(value);
      break;
    default:
      break;
  }
}

function render() {
  setText(ui.requestedState, state.requestedState || "--");
  setText(ui.currentState, state.currentState || "--");

  if (state.requestAccepted === null) {
    setText(ui.requestStatus, "--");
  } else {
    setText(ui.requestStatus, state.requestAccepted ? "ACCEPTED" : "REJECTED");
  }
  setText(ui.requestReason, state.requestReason || "--");

  setText(ui.alliance, state.alliance || "--");
  setText(ui.matchTime, formatMatchTime(state.matchTime));
  setText(ui.hubTimeframe, state.hubTimeframe || "--");

  const ourHubText = formatOurHubStatus(
    state.ourHubStatus,
    state.ourHubActive,
    state.hubStatusValid
  );
  setText(ui.ourHub, ourHubText);
  applyStatusClass(ui.ourHub, state.ourHubActive, state.hubStatusValid);

  setText(ui.hubRecommendation, state.hubRecommendation || "--");
  applyRecommendationClass(ui.hubRecommendation, state.hubRecommendation);

  setText(ui.hubFms, formatHubFms(state));
  setText(ui.battery, formatVoltage(state.batteryVoltage));
  setText(ui.brownout, state.brownout ? "YES" : "NO");
  setText(ui.hasBall, state.hasBall ? "YES" : "NO");
  setText(ui.visionStatus, state.visionStatus || "--");

  const turret = state.turretMode ? state.turretMode : "UNAVAILABLE";
  const turretExtra =
    state.turretAtSetpoint === null
      ? ""
      : state.turretAtSetpoint
        ? " (AT GOAL)"
        : " (MOVING)";
  setText(ui.turretStatus, turret + turretExtra);

  const climb = [state.climbPhase || "UNKNOWN", state.climbLevel || 0].join(" • L");
  setText(ui.climb, climb);

  setText(ui.robotPose, formatPose(state.robotPose, true));
  setText(ui.target, formatTarget(state.targetType, state.targetPose, state.targetPoseValid));

  // Button highlight
  ui.stateButtons.forEach((b) => {
    const name = b.dataset.state;
    b.classList.toggle("is-requested", name && name === state.requestedState);
    b.classList.toggle("is-current", name && name === state.currentState);
  });

  renderField();
}

function formatOurHubStatus(ourHubStatus, ourHubActive, hubStatusValid) {
  if (!hubStatusValid) {
    return ourHubStatus || "UNKNOWN";
  }
  if (ourHubStatus) {
    return ourHubStatus;
  }
  return ourHubActive ? "ACTIVE" : "INACTIVE";
}

function formatHubFms(s) {
  const raw = (s.gameDataRaw || "").trim();
  const winner = s.autoWinnerAlliance || "UNKNOWN";
  const red = s.redHubStatus || "UNKNOWN";
  const blue = s.blueHubStatus || "UNKNOWN";

  const parts = [];
  parts.push(`AutoWinner=${winner}`);
  parts.push(`Red=${red}`);
  parts.push(`Blue=${blue}`);
  parts.push(`Raw=${raw || "(empty)"}`);
  return parts.join("  ");
}

function applyStatusClass(el, isActive, hubStatusValid) {
  if (!el) return;
  el.classList.remove("tile__v--ok", "tile__v--bad", "tile__v--warn");
  if (!hubStatusValid) {
    el.classList.add("tile__v--warn");
  } else if (isActive) {
    el.classList.add("tile__v--ok");
  } else {
    el.classList.add("tile__v--bad");
  }
}

function applyRecommendationClass(el, recommendation) {
  if (!el) return;
  el.classList.remove("tile__v--ok", "tile__v--bad", "tile__v--warn");
  const token = (recommendation || "").toUpperCase();
  if (token.includes("SCORE")) {
    el.classList.add("tile__v--ok");
  } else if (token.includes("CHECK") || token.includes("UNKNOWN")) {
    el.classList.add("tile__v--warn");
  } else if (token.includes("COLLECT") || token.includes("DEFEND")) {
    el.classList.add("tile__v--bad");
  }
}

function renderField() {
  if (!fieldCtx || !ui.fieldCanvas) return;

  const { width, height } = getCanvasSize(ui.fieldCanvas);
  fieldCtx.clearRect(0, 0, width, height);

  const robot = state.robotPose ? fieldToCanvas(state.robotPose, width, height) : null;
  const target =
    state.targetPoseValid && state.targetPose
      ? fieldToCanvas(state.targetPose, width, height)
      : null;

  if (robot) {
    drawPoint(robot.x, robot.y, 7, "rgba(57,217,138,0.95)");
    drawHeading(robot.x, robot.y, state.robotPose?.theta ?? 0, "rgba(57,217,138,0.95)");
  }
  if (target) {
    drawPoint(target.x, target.y, 7, "rgba(255,209,102,0.95)");
  }
  if (robot && target) {
    drawLine(robot.x, robot.y, target.x, target.y, "rgba(255,255,255,0.35)");
  }
}

function setupFieldCanvas() {
  if (!ui.fieldCanvas || !ui.fieldImage) return;
  fieldCtx = ui.fieldCanvas.getContext("2d");
  resizeCanvasToDisplaySize(ui.fieldCanvas);
}

function resizeCanvasToDisplaySize(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(1, Math.floor(rect.width * dpr));
  const height = Math.max(1, Math.floor(rect.height * dpr));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
}

function getCanvasSize(canvas) {
  // Canvas width/height are in device pixels after resizeCanvasToDisplaySize
  return { width: canvas.width, height: canvas.height };
}

function fieldToCanvas(pose, width, height) {
  const x = clamp(pose.x / FIELD_LENGTH_METERS, 0, 1) * width;
  const y = (1 - clamp(pose.y / FIELD_WIDTH_METERS, 0, 1)) * height;
  return { x, y };
}

function drawPoint(x, y, r, color) {
  fieldCtx.save();
  fieldCtx.beginPath();
  fieldCtx.arc(x, y, r, 0, Math.PI * 2);
  fieldCtx.fillStyle = color;
  fieldCtx.fill();
  fieldCtx.lineWidth = 2;
  fieldCtx.strokeStyle = "rgba(0,0,0,0.35)";
  fieldCtx.stroke();
  fieldCtx.restore();
}

function drawHeading(x, y, thetaRad, color) {
  const len = 18;
  const dx = Math.cos(thetaRad) * len;
  const dy = -Math.sin(thetaRad) * len;
  drawLine(x, y, x + dx, y + dy, color, 2.5);
}

function drawLine(x1, y1, x2, y2, color, width = 2) {
  fieldCtx.save();
  fieldCtx.beginPath();
  fieldCtx.moveTo(x1, y1);
  fieldCtx.lineTo(x2, y2);
  fieldCtx.strokeStyle = color;
  fieldCtx.lineWidth = width;
  fieldCtx.stroke();
  fieldCtx.restore();
}

function updateChips() {
  const now = Date.now();
  const stale = now - lastAnyData > STALE_MS;

  setChip(ui.chipNt, ntConnected ? "NT OK" : "NT OFF", ntConnected ? "ok" : "bad");
  setChip(ui.chipData, stale ? "DATA STALE" : "DATA OK", stale ? "warn" : "ok");
  setChip(ui.chipMode, state.dsMode ? state.dsMode : "MODE", "none");
}

function setChip(el, text, status) {
  if (!el) return;
  el.classList.remove("chip--ok", "chip--bad", "chip--warn");
  if (status === "ok") el.classList.add("chip--ok");
  if (status === "bad") el.classList.add("chip--bad");
  if (status === "warn") el.classList.add("chip--warn");
  el.innerText = text;
}

function formatPose(pose, valid) {
  if (!pose || valid === false) return "--";
  const x = round(pose.x, 3);
  const y = round(pose.y, 3);
  const deg = round((pose.theta ?? 0) * (180 / Math.PI), 1);
  return `x=${x} y=${y} th=${deg}deg`;
}

function formatTarget(type, pose, valid) {
  if (!valid || !pose) return `${type || "TARGET"}: --`;
  return `${type || "TARGET"}: ${formatPose(pose, true)}`;
}

function formatVoltage(v) {
  if (!Number.isFinite(v)) return "--";
  return `${round(v, 2)} V`;
}

function formatMatchTime(t) {
  if (!Number.isFinite(t)) return "--";
  const clamped = Math.max(0, t);
  const m = Math.floor(clamped / 60);
  const s = Math.floor(clamped % 60);
  return `${m}:${String(s).padStart(2, "0")}`;
}

function prettify(token) {
  return String(token)
    .toLowerCase()
    .replaceAll("_", " ")
    .replace(/\b\w/g, (m) => m.toUpperCase());
}

function parseString(value) {
  if (typeof value === "string") return value;
  if (value === null || value === undefined) return null;
  return String(value);
}

function parsePose(value) {
  // Expected: [x,y,theta] or {x,y,theta}
  if (!value) return null;
  if (Array.isArray(value) && value.length >= 2) {
    const x = Number(value[0]);
    const y = Number(value[1]);
    const theta = Number(value[2] ?? 0);
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(theta)) return null;
    return { x, y, theta };
  }
  if (typeof value === "object") {
    const x = Number(value.x);
    const y = Number(value.y);
    const theta = Number(value.theta ?? 0);
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(theta)) return null;
    return { x, y, theta };
  }
  return null;
}

function setText(el, text) {
  if (!el) return;
  el.innerText = text;
}

function clamp(v, min, max) {
  return Math.max(min, Math.min(max, v));
}

function round(v, digits) {
  const p = Math.pow(10, digits);
  return Math.round(v * p) / p;
}

function resolveNtConnectionParams(params) {
  const host =
    params.get("ntHost") ||
    params.get("host") ||
    window.location.hostname ||
    "localhost";
  const portRaw = params.get("ntPort") || params.get("port");
  const portParsed = portRaw ? Number(portRaw) : NaN;
  const port = Number.isFinite(portParsed) ? portParsed : defaultNt4Port;
  return { host, port };
}
