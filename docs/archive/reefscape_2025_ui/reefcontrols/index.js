import { NT4_Client } from "./NT4.js";

const defaultNt4Port = 5810;
const queryParams = new URLSearchParams(window.location.search);
const { host: ntHost, port: ntPort } = resolveNtConnectionParams(queryParams);

const contract = {
  base: "/OperatorBoard/v1",
  toRobot: "/OperatorBoard/v1/ToRobot/",
  toDashboard: "/OperatorBoard/v1/ToDashboard/",
  keys: {
    requestedState: "RequestedState",
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

const FIELD_LENGTH_METERS = 17.55;
const FIELD_WIDTH_METERS = 8.05;
const STALE_MS = 1500;

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
  turretAtSetpoint: null,
  turretMode: null,
  visionStatus: null,
};

let lastAnyData = 0;
let ntConnected = false;

const ui = {
  connectionStatus: null,
  dataStatus: null,
  modeStatus: null,
  faultBanner: null,
  requestedState: null,
  currentState: null,
  requestStatus: null,
  requestReason: null,
  climbPhase: null,
  climbLevel: null,
  hasBall: null,
  targetType: null,
  targetPose: null,
  turretStatus: null,
  alliance: null,
  matchTime: null,
  battery: null,
  brownout: null,
  visionStatus: null,
  poseX: null,
  poseY: null,
  poseHeading: null,
  fieldFrame: null,
  fieldCanvas: null,
  fieldOverlay: null,
  buttons: [],
};

let fieldCtx = null;

const ntClient = new NT4_Client(
  ntHost,
  "OperatorBoard",
  () => {},
  () => {},
  (topic, _, value) => {
    handleTopicUpdate(topic, value);
  },
  () => {
    ntConnected = true;
    updateStatusChips();
  },
  () => {
    ntConnected = false;
    updateStatusChips();
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
    updateStatusChips();
    renderField();
  }, 200);
});

function cacheUi() {
  ui.connectionStatus = document.getElementById("connection-status");
  ui.dataStatus = document.getElementById("data-status");
  ui.modeStatus = document.getElementById("mode-status");
  ui.faultBanner = document.getElementById("fault-banner");
  ui.requestedState = document.getElementById("requested-state");
  ui.currentState = document.getElementById("current-state");
  ui.requestStatus = document.getElementById("request-status");
  ui.requestReason = document.getElementById("request-reason");
  ui.climbPhase = document.getElementById("climb-phase");
  ui.climbLevel = document.getElementById("climb-level");
  ui.hasBall = document.getElementById("has-ball");
  ui.targetType = document.getElementById("target-type");
  ui.targetPose = document.getElementById("target-pose");
  ui.turretStatus = document.getElementById("turret-status");
  ui.alliance = document.getElementById("alliance");
  ui.matchTime = document.getElementById("match-time");
  ui.battery = document.getElementById("battery");
  ui.brownout = document.getElementById("brownout");
  ui.visionStatus = document.getElementById("vision-status");
  ui.poseX = document.getElementById("pose-x");
  ui.poseY = document.getElementById("pose-y");
  ui.poseHeading = document.getElementById("pose-heading");
  ui.fieldFrame = document.getElementById("field-frame");
  ui.fieldCanvas = document.getElementById("field-canvas");
  ui.fieldOverlay = document.getElementById("field-overlay");
}

function buildStateButtons() {
  const container = document.getElementById("state-buttons");
  if (!container) {
    return;
  }
  container.innerHTML = "";
  ui.buttons = STATES.map((stateName) => {
    const button = document.createElement("button");
    button.className = "state-button";
    button.dataset.state = stateName;
    button.type = "button";
    button.innerText = prettifyState(stateName);
    button.addEventListener("click", () => sendStateRequest(stateName));
    container.appendChild(button);
    return button;
  });
}

function sendStateRequest(stateName) {
  if (!stateName) {
    return;
  }
  ntClient.addSample(contract.toRobot + contract.keys.requestedState, stateName);
}

function startNetworkTables() {
  const topics = Object.values(contract.keys).map(
    (key) => contract.toDashboard + key
  );
  ntClient.subscribe(topics, false, false, 0.02);
  ntClient.publishTopic(contract.toRobot + contract.keys.requestedState, "string");
  ntClient.connect();
}

function handleTopicUpdate(topic, value) {
  if (!topic || !topic.name) {
    return;
  }
  lastAnyData = Date.now();
  const name = topic.name;
  switch (name) {
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
      state.climbLevel = parseNumber(value);
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
      state.batteryVoltage = parseNumber(value);
      break;
    case contract.toDashboard + contract.keys.brownout:
      state.brownout = !!value;
      break;
    case contract.toDashboard + contract.keys.alliance:
      state.alliance = parseString(value);
      break;
    case contract.toDashboard + contract.keys.matchTime:
      state.matchTime = parseNumber(value);
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
      return;
  }
  updateUI();
}

function updateUI() {
  setText(ui.requestedState, state.requestedState || "--");
  setText(ui.currentState, state.currentState || "--");

  const requestAccepted = state.requestAccepted;
  if (requestAccepted === null || requestAccepted === undefined) {
    setText(ui.requestStatus, "--");
  } else {
    setText(ui.requestStatus, requestAccepted ? "Accepted" : "Rejected");
  }

  const reason = state.requestReason && state.requestReason.length > 0 ? state.requestReason : "--";
  setText(ui.requestReason, reason);

  updateFaultBanner(requestAccepted, state.requestReason);

  setText(ui.climbPhase, state.climbPhase || "--");
  setText(ui.climbLevel, formatInteger(state.climbLevel));

  if (ui.hasBall) {
    if (state.hasBall === null || state.hasBall === undefined) {
      ui.hasBall.innerText = "--";
      ui.hasBall.classList.remove("is-true", "is-false");
    } else {
      ui.hasBall.innerText = state.hasBall ? "TRUE" : "FALSE";
      ui.hasBall.classList.toggle("is-true", !!state.hasBall);
      ui.hasBall.classList.toggle("is-false", !state.hasBall);
    }
  }

  setText(ui.targetType, state.targetType || "--");
  setText(ui.targetPose, formatPose(state.targetPose, state.targetPoseValid));
  setText(ui.turretStatus, formatTurretStatus());

  setText(ui.alliance, state.alliance || "--");
  setText(ui.matchTime, formatMatchTime(state.matchTime));
  setText(ui.battery, formatVoltage(state.batteryVoltage));
  setText(ui.brownout, formatBoolean(state.brownout));
  setText(ui.visionStatus, state.visionStatus || "--");

  setText(ui.poseX, formatNumber(state.robotPose && state.robotPose.x));
  setText(ui.poseY, formatNumber(state.robotPose && state.robotPose.y));
  setText(ui.poseHeading, formatHeading(state.robotPose && state.robotPose.theta));

  updateButtons();
  updateStatusChips();
  renderField();
}

function updateButtons() {
  ui.buttons.forEach((button) => {
    const name = button.dataset.state;
    button.classList.toggle("is-requested", name === state.requestedState);
    button.classList.toggle("is-current", name === state.currentState);
    button.classList.toggle("is-disabled", !ntConnected);
    button.disabled = !ntConnected;
  });
}

function updateFaultBanner(accepted, reason) {
  if (!ui.faultBanner) {
    return;
  }
  if (accepted === false) {
    const message = reason && reason.trim().length > 0 ? reason : "Request rejected";
    ui.faultBanner.innerText = message.toUpperCase();
    ui.faultBanner.hidden = false;
  } else {
    ui.faultBanner.hidden = true;
  }
}

function updateStatusChips() {
  const connectionChip = document.getElementById("connection-chip");
  const dataChip = document.getElementById("data-chip");
  const modeChip = document.getElementById("mode-chip");

  if (connectionChip && ui.connectionStatus) {
    ui.connectionStatus.innerText = ntConnected ? "Connected" : "Offline";
    connectionChip.classList.remove("is-live", "is-offline");
    connectionChip.classList.add(ntConnected ? "is-live" : "is-offline");
  }

  const stale = isDataStale();
  if (dataChip && ui.dataStatus) {
    ui.dataStatus.innerText = stale ? "Stale" : "Live";
    dataChip.classList.remove("is-live", "is-stale");
    dataChip.classList.add(stale ? "is-stale" : "is-live");
  }

  if (modeChip && ui.modeStatus) {
    ui.modeStatus.innerText = state.dsMode || "Unknown";
  }
}

function isDataStale() {
  if (!ntConnected) {
    return true;
  }
  if (!lastAnyData) {
    return true;
  }
  return Date.now() - lastAnyData > STALE_MS;
}

function formatPose(pose, valid) {
  if (!pose || !valid) {
    return "--";
  }
  return `${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}`;
}

function formatTurretStatus() {
  if (!state.turretMode || state.turretMode === "UNAVAILABLE") {
    return "Unavailable";
  }
  const ready = state.turretAtSetpoint ? "Ready" : "Tracking";
  return `${ready} (${state.turretMode})`;
}

function formatMatchTime(value) {
  if (typeof value !== "number" || !isFinite(value) || value < 0) {
    return "--";
  }
  return value.toFixed(1) + "s";
}

function formatVoltage(value) {
  if (typeof value !== "number" || !isFinite(value)) {
    return "--";
  }
  return value.toFixed(1) + " V";
}

function formatBoolean(value) {
  if (value === null || value === undefined) {
    return "--";
  }
  return value ? "YES" : "NO";
}

function formatNumber(value) {
  if (typeof value !== "number" || !isFinite(value)) {
    return "--";
  }
  return value.toFixed(2);
}

function formatHeading(value) {
  if (typeof value !== "number" || !isFinite(value)) {
    return "--";
  }
  return (value * (180 / Math.PI)).toFixed(1) + " deg";
}

function formatInteger(value) {
  if (typeof value !== "number" || !isFinite(value)) {
    return "--";
  }
  return Math.round(value).toString();
}

function prettifyState(stateName) {
  return stateName
    .toLowerCase()
    .split("_")
    .map((chunk) => chunk.charAt(0).toUpperCase() + chunk.slice(1))
    .join(" ");
}

function parseString(value) {
  return typeof value === "string" ? value : "";
}

function parseNumber(value) {
  if (typeof value === "number" && isFinite(value)) {
    return value;
  }
  return NaN;
}

function parsePose(value) {
  const raw = normalizeArrayLike(value);
  const components = raw && raw.length >= 3 ? raw : normalizePoseObject(value);
  if (!components || components.length < 3) {
    return null;
  }
  const x = Number(components[0]);
  const y = Number(components[1]);
  const theta = Number(components[2]);
  if (![x, y, theta].every((v) => isFinite(v))) {
    return null;
  }
  return { x, y, theta };
}

function normalizeArrayLike(value) {
  if (Array.isArray(value)) {
    return value;
  }
  if (value && typeof value.length === "number" && ArrayBuffer.isView(value)) {
    return Array.from(value);
  }
  if (
    value &&
    typeof value === "object" &&
    typeof value.length === "number" &&
    !Array.isArray(value)
  ) {
    return Array.from(value);
  }
  if (value && typeof value === "object" && "value" in value) {
    return normalizeArrayLike(value.value);
  }
  if (typeof value === "string") {
    const parsed = parsePoseString(value);
    if (parsed) {
      return parsed;
    }
  }
  return null;
}

function normalizePoseObject(value) {
  if (!value || typeof value !== "object") {
    return null;
  }
  const x = value.x ?? value.X;
  const y = value.y ?? value.Y;
  const theta =
    value.theta ??
    value.heading ??
    value.rotation ??
    value.radians ??
    value.yaw;
  if ([x, y, theta].some((v) => v === undefined || v === null)) {
    return null;
  }
  return [x, y, theta];
}

function parsePoseString(value) {
  const trimmed = value.trim();
  if (!trimmed) {
    return null;
  }
  if (trimmed.startsWith("[") && trimmed.endsWith("]")) {
    try {
      const parsed = JSON.parse(trimmed);
      return normalizeArrayLike(parsed);
    } catch (error) {
      return null;
    }
  }
  if (trimmed.includes(",")) {
    const parts = trimmed.split(",").map((part) => part.trim());
    if (parts.length >= 3) {
      return parts.slice(0, 3).map((part) => Number(part));
    }
  }
  return null;
}

function setupFieldCanvas() {
  if (!ui.fieldCanvas || !ui.fieldFrame) {
    return;
  }
  const rect = ui.fieldFrame.getBoundingClientRect();
  const pixelRatio = window.devicePixelRatio || 1;
  ui.fieldCanvas.width = rect.width * pixelRatio;
  ui.fieldCanvas.height = rect.height * pixelRatio;
  fieldCtx = ui.fieldCanvas.getContext("2d");
  if (fieldCtx) {
    fieldCtx.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
  }
  renderField();
}

function renderField() {
  if (!fieldCtx || !ui.fieldFrame) {
    return;
  }
  const width = ui.fieldFrame.clientWidth;
  const height = ui.fieldFrame.clientHeight;
  fieldCtx.clearRect(0, 0, width, height);

  if (!state.robotPose) {
    if (ui.fieldOverlay) {
      ui.fieldOverlay.hidden = false;
    }
    return;
  }

  if (ui.fieldOverlay) {
    ui.fieldOverlay.hidden = true;
  }

  const robotPoint = fieldToCanvas(state.robotPose, width, height);
  drawRobot(robotPoint.x, robotPoint.y, state.robotPose.theta);

  if (state.targetPoseValid && state.targetPose) {
    const targetPoint = fieldToCanvas(state.targetPose, width, height);
    drawTarget(targetPoint.x, targetPoint.y);
    drawLink(robotPoint, targetPoint);
  }
}

function fieldToCanvas(pose, width, height) {
  const x = (pose.x / FIELD_LENGTH_METERS) * width;
  const y = height - (pose.y / FIELD_WIDTH_METERS) * height;
  return { x, y };
}

function drawRobot(x, y, headingRad) {
  const size = 14;
  fieldCtx.save();
  fieldCtx.translate(x, y);
  fieldCtx.rotate(-headingRad + Math.PI / 2);
  fieldCtx.fillStyle = "rgba(57, 198, 181, 0.9)";
  fieldCtx.strokeStyle = "rgba(57, 198, 181, 0.5)";
  fieldCtx.lineWidth = 2;
  fieldCtx.beginPath();
  fieldCtx.moveTo(0, -size);
  fieldCtx.lineTo(size * 0.7, size);
  fieldCtx.lineTo(-size * 0.7, size);
  fieldCtx.closePath();
  fieldCtx.fill();
  fieldCtx.stroke();
  fieldCtx.restore();
}

function drawTarget(x, y) {
  fieldCtx.save();
  fieldCtx.strokeStyle = "rgba(244, 166, 60, 0.9)";
  fieldCtx.lineWidth = 2;
  fieldCtx.beginPath();
  fieldCtx.arc(x, y, 10, 0, Math.PI * 2);
  fieldCtx.stroke();
  fieldCtx.restore();
}

function drawLink(from, to) {
  fieldCtx.save();
  fieldCtx.strokeStyle = "rgba(244, 166, 60, 0.4)";
  fieldCtx.lineWidth = 2;
  fieldCtx.setLineDash([6, 6]);
  fieldCtx.beginPath();
  fieldCtx.moveTo(from.x, from.y);
  fieldCtx.lineTo(to.x, to.y);
  fieldCtx.stroke();
  fieldCtx.restore();
}

function resolveNtConnectionParams(params) {
  const fallbackHost =
    window.location.hostname && window.location.hostname.trim().length > 0
      ? window.location.hostname
      : "localhost";
  const overrideKeys = ["nthost", "nt", "host", "rio"];
  let hostCandidate = null;
  for (const key of overrideKeys) {
    const value = getQueryParamIgnoreCase(params, key);
    if (value && value.trim().length > 0) {
      hostCandidate = value.trim();
      break;
    }
  }
  const parsed = extractHostAndPort(hostCandidate);
  let host = parsed.host || fallbackHost;
  let port = parsed.port || defaultNt4Port;
  const explicitPort = getQueryParamIgnoreCase(params, "ntport");
  if (explicitPort && explicitPort.trim().length > 0) {
    const candidate = Number(explicitPort);
    if (Number.isFinite(candidate) && candidate > 0) {
      port = candidate;
    }
  }
  return { host, port };
}

function extractHostAndPort(rawValue) {
  if (!rawValue) {
    return { host: null, port: null };
  }
  let token = rawValue.trim();
  if (!token) {
    return { host: null, port: null };
  }
  token = token.replace(/^\s*(?:ws|wss|http|https):\/\//i, "");
  token = token.replace(/\/.*$/, "");
  let host = token;
  let port = null;
  const colonIndex = token.lastIndexOf(":");
  if (colonIndex > -1) {
    const portCandidate = token.slice(colonIndex + 1);
    const parsedPort = Number(portCandidate);
    if (Number.isFinite(parsedPort) && parsedPort > 0) {
      port = parsedPort;
      host = token.slice(0, colonIndex);
    }
  }
  host = host.trim();
  if (host.length === 0) {
    host = null;
  }
  return { host, port };
}

function getQueryParamIgnoreCase(params, name) {
  if (!params || typeof params.entries !== "function") {
    return null;
  }
  const target = name.toLowerCase();
  for (const [key, value] of params.entries()) {
    if (key.toLowerCase() === target) {
      return value;
    }
  }
  return null;
}

function setText(el, value) {
  if (!el) {
    return;
  }
  el.innerText = value;
}
