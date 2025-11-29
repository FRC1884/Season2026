// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import { NT4_Client } from "./NT4.js";

const templateDirectory = "templates";
const defaultTemplateName = "2025";
const defaultNt4Port = 5810;
const queryParams = new URLSearchParams(window.location.search);
const { host: ntHost, port: ntPort } = resolveNtConnectionParams(queryParams);

// ***** NETWORKTABLES *****

const toRobotPrefix = "/ReefControls/ToRobot/";
const toDashboardPrefix = "/ReefControls/ToDashboard/";
const selectedLevelTopicName = "SelectedLevel";
const l1TopicName = "Level1";
const l2TopicName = "Level2";
const l3TopicName = "Level3";
const l4TopicName = "Level4";
const algaeTopicName = "Algae";
const coopTopicName = "Coop";
const isElimsTopicName = "IsElims";
const queueSpecTopicName = "QueueSpec";
const queueCommandTopicName = "QueueCommand";
const queueStateTopicName = "QueueState";

const ntSubscriptions = [
  toDashboardPrefix + selectedLevelTopicName,
  toDashboardPrefix + l1TopicName,
  toDashboardPrefix + l2TopicName,
  toDashboardPrefix + l3TopicName,
  toDashboardPrefix + l4TopicName,
  toDashboardPrefix + algaeTopicName,
  toDashboardPrefix + coopTopicName,
  toDashboardPrefix + isElimsTopicName,
  toDashboardPrefix + queueStateTopicName,
];

let templateConfig = null;
let templateName = defaultTemplateName;
let queueUiInitialized = false;
let canvasInitialized = false;
let ntSessionStarted = false;

const uiRefs = {
  counters: [],
  branchNodes: [],
  algaeNodes: [],
  controls: [],
  reefFlag: null,
  reefCanvas: null,
};

const ntClient = new NT4_Client(
  ntHost,
  "ReefControls",
  () => {
    // Topic announce
  },
  () => {
    // Topic unannounce
  },
  (topic, _, value) => {
    if (topic.name === toDashboardPrefix + selectedLevelTopicName) {
      selectedLevel = clampSelectedLevel(value);
    } else if (topic.name === toDashboardPrefix + l1TopicName) {
      l1State = value;
    } else if (topic.name === toDashboardPrefix + l2TopicName) {
      l2State = value;
    } else if (topic.name === toDashboardPrefix + l3TopicName) {
      l3State = value;
    } else if (topic.name === toDashboardPrefix + l4TopicName) {
      l4State = value;
    } else if (topic.name === toDashboardPrefix + algaeTopicName) {
      algaeState = value;
    } else if (topic.name === toDashboardPrefix + coopTopicName) {
      coopState = value;
    } else if (topic.name === toDashboardPrefix + isElimsTopicName) {
      isElims = value;
    } else if (topic.name === toDashboardPrefix + queueStateTopicName) {
      if (typeof value === "string") {
        applyQueueState(value);
      }
    } else {
      return;
    }
    updateUI();
  },
  () => {
    // Connected
  },
  () => {
    const disconnectedColor =
      (templateConfig && templateConfig.theme && templateConfig.theme.disconnectedBackground) ||
      "#b71c1c";
    document.body.style.backgroundColor = disconnectedColor;
  },
  { port: ntPort }
);

// Start everything once DOM is ready
window.addEventListener("DOMContentLoaded", () => {
  bootstrapTemplate();
});

async function bootstrapTemplate() {
  try {
    templateConfig = await loadTemplateFromQuery();
    buildAppShell();
    initQueueUi();
    updateUI();
    initReefCanvas();
    startNetworkTables();
  } catch (error) {
    console.error("Failed to load template", error);
    showTemplateError(error);
  }
}

async function loadTemplateFromQuery() {
  const requestedRaw = queryParams.get("template") || defaultTemplateName;
  const requested = requestedRaw && requestedRaw.trim().length > 0 ? requestedRaw.trim() : defaultTemplateName;
  templateName = requested;
  try {
    return await fetchTemplate(requested);
  } catch (error) {
    if (requested !== defaultTemplateName) {
      console.warn(`Falling back to default template due to error loading ${requested}`, error);
      return await fetchTemplate(defaultTemplateName);
    }
    throw error;
  }
}

async function fetchTemplate(name) {
  const response = await fetch(`${templateDirectory}/${name}.json`, { cache: "no-store" });
  if (!response.ok) {
    throw new Error(`Template ${name} not found`);
  }
  const config = await response.json();
  config.__name = name;
  return config;
}

function showTemplateError(error) {
  const root = document.getElementById("app-root");
  if (!root) {
    return;
  }
  root.innerHTML = "";
  const panel = document.createElement("div");
  panel.className = "loading-panel";
  panel.innerHTML = `
    <div class="loading-title">Unable to load template</div>
    <div class="loading-subtitle">${error && error.message ? error.message : error}</div>
  `;
  root.appendChild(panel);
}

function buildAppShell() {
  const root = document.getElementById("app-root");
  if (!root) {
    return;
  }
  resetUiRefs();
  root.classList.add("app-loaded");
  root.innerHTML = "";

  const shell = document.createElement("div");
  shell.className = "app-shell";
  root.appendChild(shell);

  const header = buildHeader();
  const main = buildMainLayout();
  shell.appendChild(header);
  shell.appendChild(main);
  injectMetadata();
}

function resetUiRefs() {
  uiRefs.counters = [];
  uiRefs.branchNodes = [];
  uiRefs.algaeNodes = [];
  uiRefs.controls = [];
  uiRefs.reefFlag = null;
  uiRefs.reefCanvas = null;
}

function injectMetadata() {
  const meta = (templateConfig && templateConfig.metadata) || {};
  if (meta.title) {
    document.title = meta.subtitle ? `${meta.title} — ${meta.subtitle}` : meta.title;
  }
  if (meta.favicon) {
    const existing = document.querySelector("link[rel='icon']");
    if (existing) {
      existing.href = meta.favicon;
    } else {
      const link = document.createElement("link");
      link.rel = "icon";
      link.href = meta.favicon;
      document.head.appendChild(link);
    }
  }
}

function buildHeader() {
  const meta = (templateConfig && templateConfig.metadata) || {};
  const header = document.createElement("header");
  header.className = "app-header";

  const title = document.createElement("div");
  title.className = "app-title";
  title.innerText = meta.title || "Reef Controls";

  const subtitle = document.createElement("div");
  subtitle.className = "app-subtitle";
  subtitle.innerText = meta.subtitle || templateName.toUpperCase();

  header.appendChild(title);
  header.appendChild(subtitle);
  return header;
}

function buildMainLayout() {
  const main = document.createElement("main");
  main.className = "main-layout";

  const primaryColumn = document.createElement("div");
  primaryColumn.className = "primary-column";
  main.appendChild(primaryColumn);

  const overviewSection = buildOverviewSection();
  if (overviewSection) {
    primaryColumn.appendChild(overviewSection);
  }

  const reefSection = buildReefSection();
  if (reefSection) {
    primaryColumn.appendChild(reefSection);
  }

  const controlsSection = buildControlsSection();
  if (controlsSection) {
    primaryColumn.appendChild(controlsSection);
  }

  const queueSection = buildQueueSection();
  if (queueSection) {
    main.appendChild(queueSection);
  }

  return main;
}

function buildOverviewSection() {
  const overview = templateConfig && templateConfig.overview;
  if (!overview || !Array.isArray(overview.counters) || overview.counters.length === 0) {
    return null;
  }

  const section = document.createElement("section");
  section.className = "overview-section";

  overview.counters.forEach((counter, index) => {
    const container = document.createElement("div");
    container.className = "counter-container";

    const area = document.createElement("div");
    area.className = "counter-area";
    area.dataset.counterId = counter.id || `counter-${index}`;

    const valueEl = document.createElement("div");
    valueEl.className = "counter";
    valueEl.innerText = "0";

    const labelEl = document.createElement("div");
    labelEl.className = "counter-label";
    labelEl.innerText = counter.label || `Counter ${index + 1}`;

    area.appendChild(valueEl);
    area.appendChild(labelEl);
    container.appendChild(area);
    section.appendChild(container);

    if (typeof counter.selectsLevel === "number") {
      bind(area, () => {
        setSelectedLevel(counter.selectsLevel);
      });
    }

    uiRefs.counters.push({ config: counter, area, valueEl });
  });

  return section;
}

function buildReefSection() {
  const reef = templateConfig && templateConfig.reef;
  if (!reef) {
    return null;
  }

  const section = document.createElement("section");
  section.className = "reef-section";

  const branchImage = reef.branchImage || "coral.png";
  const branchSize = reef.branchSizeVh || 12;
  const algaeImage = reef.algaeImage || "algae.png";
  const algaeSize = reef.algaeSizeVh || 14;

  if (Array.isArray(reef.branchNodes)) {
    reef.branchNodes.forEach((node, index) => {
      const element = document.createElement("div");
      element.className = "reef-node branch";
      applyNodePosition(element, node, branchSize);

      const img = document.createElement("img");
      img.src = node.image || branchImage;
      img.alt = node.alt || "Coral position";
      element.appendChild(img);
      section.appendChild(element);
      bind(element, () => handleBranchToggle(index));
      uiRefs.branchNodes.push({ element, config: node });
    });
  }

  if (Array.isArray(reef.algaeNodes)) {
    reef.algaeNodes.forEach((node, index) => {
      const element = document.createElement("div");
      element.className = "reef-node algae";
      applyNodePosition(element, node, algaeSize);

      const img = document.createElement("img");
      img.src = node.image || algaeImage;
      img.alt = node.alt || "Algae position";
      element.appendChild(img);
      section.appendChild(element);
      bind(element, () => handleAlgaeToggle(index));
      uiRefs.algaeNodes.push({ element, config: node });
    });
  }

  const flag = document.createElement("div");
  flag.className = "flag";
  flag.hidden = true;
  flag.innerText = reef.flagEmoji || "\uD83C\uDFC1";
  section.appendChild(flag);
  uiRefs.reefFlag = flag;

  const canvas = document.createElement("canvas");
  canvas.className = "reef-canvas";
  canvas.setAttribute("aria-hidden", "true");
  section.appendChild(canvas);
  uiRefs.reefCanvas = canvas;

  return section;
}

function applyNodePosition(element, config, defaultSize) {
  const size = config.sizeVh || defaultSize;
  element.style.left = config.x != null ? `${config.x}%` : "50%";
  element.style.top = config.y != null ? `${config.y}%` : "50%";
  element.style.width = `${size}vh`;
  element.style.height = `${size}vh`;
}

function buildControlsSection() {
  const controls = templateConfig && templateConfig.controls;
  if (!Array.isArray(controls) || controls.length === 0) {
    return null;
  }

  const layout = (templateConfig && templateConfig.layout) || {};
  const absoluteLayout = layout.controlsLayout === "absolute";
  const section = document.createElement("section");
  section.className = "controls-section";
  if (absoluteLayout) {
    section.classList.add("controls-layout-absolute");
  }

  const absoluteLayer = absoluteLayout ? document.createElement("div") : null;
  if (absoluteLayer) {
    absoluteLayer.className = "controls-absolute-layer";
    section.appendChild(absoluteLayer);
  }

  const unplaced = [];

  controls.forEach((control, index) => {
    const button = document.createElement("button");
    button.type = "button";
    button.className = "control-button";
    button.dataset.controlId = control.id || `control-${index}`;

    if (Array.isArray(control.classes)) {
      control.classes.forEach((cls) => button.classList.add(cls));
    }

    if (control.style) {
      if (control.style.background) {
        button.style.background = control.style.background;
      }
      if (control.style.textColor) {
        button.style.color = control.style.textColor;
      }
      if (control.style.borderColor) {
        button.style.borderColor = control.style.borderColor;
      }
    }

    const icon = document.createElement("div");
    icon.className = "button-icon";
    if (control.iconType === "image" && control.icon) {
      button.classList.add("image-button");
      const image = document.createElement("img");
      image.src = control.icon;
      image.alt = control.label || "control icon";
      icon.appendChild(image);
    } else {
      icon.innerText = control.icon || "";
    }
    button.appendChild(icon);

    const label = document.createElement("div");
    label.className = "button-label";
    label.innerText = control.label || `Button ${index + 1}`;
    button.appendChild(label);

    bind(button, () => handleControlAction(control));

    uiRefs.controls.push({ element: button, config: control });

    if (absoluteLayout) {
      const position = control.position || {};
      const x = position.x;
      const y = position.y;
      if (isFiniteNumber(x) && isFiniteNumber(y)) {
        button.classList.add("control-absolute");
        button.style.left = `${x}%`;
        button.style.top = `${y}%`;
        absoluteLayer.appendChild(button);
      } else {
        button.classList.add("control-unplaced");
        unplaced.push(button);
      }
    } else {
      section.appendChild(button);
    }
  });

  if (absoluteLayout && unplaced.length > 0) {
    const trayTitle = document.createElement("div");
    trayTitle.className = "controls-unplaced-title";
    trayTitle.innerText = "Unplaced controls";
    const tray = document.createElement("div");
    tray.className = "controls-unplaced";
    unplaced.forEach((button) => tray.appendChild(button));
    section.appendChild(trayTitle);
    section.appendChild(tray);
  }

  return section;
}

function buildQueueSection() {
  const queue = templateConfig && templateConfig.queue;
  if (queue === false) {
    return null;
  }
  const section = document.createElement("section");
  section.className = "queue-section";

  const panel = document.createElement("div");
  panel.className = "queue-panel";
  section.appendChild(panel);

  const header = document.createElement("div");
  header.className = "queue-header";
  panel.appendChild(header);

  const headerText = document.createElement("div");
  const title = document.createElement("div");
  title.className = "queue-title";
  title.innerText = (queue && queue.title) || "Tablet Queue";
  const subtitle = document.createElement("div");
  subtitle.className = "queue-subtitle";
  subtitle.innerText = (queue && queue.subtitle) || "Plan the robot's next moves";
  headerText.appendChild(title);
  headerText.appendChild(subtitle);
  header.appendChild(headerText);

  const controls = document.createElement("div");
  controls.className = "queue-controls";
  header.appendChild(controls);

  const manualToggle = document.createElement("button");
  manualToggle.type = "button";
  manualToggle.className = "queue-control queue-manual";
  manualToggle.id = "queue-manual-toggle";
  manualToggle.innerText = (queue && queue.manualToggle && queue.manualToggle.queueEnabledLabel) || "Disable Queue";
  controls.appendChild(manualToggle);

  const commandButtons = (queue && queue.commands) || [
    { label: "Start", command: "start" },
    { label: "Pause", command: "stop" },
    { label: "Skip", command: "skip" },
    { label: "Reset", command: "reset" },
  ];

  commandButtons.forEach((buttonConfig) => {
    const button = document.createElement("button");
    button.type = "button";
    button.className = "queue-control";
    button.dataset.queueCommand = buttonConfig.command;
    button.innerText = buttonConfig.label || buttonConfig.command;
    controls.appendChild(button);
  });

  const hint = document.createElement("div");
  hint.className = "queue-hint";
  hint.innerText =
    (queue && queue.hint) ||
    "Tap reef nodes to enqueue a source \u2192 reef pair for the selected level, then hit Start.";
  panel.appendChild(hint);

  const body = document.createElement("div");
  body.className = "queue-body";
  panel.appendChild(body);

  const list = document.createElement("ul");
  list.className = "queue-list";
  list.id = "queue-plan";
  body.appendChild(list);

  const statusPanel = document.createElement("div");
  statusPanel.className = "queue-status-panel";
  body.appendChild(statusPanel);

  const phase = document.createElement("div");
  phase.className = "queue-phase";
  const phaseLabel = document.createElement("span");
  phaseLabel.id = "queue-phase";
  phaseLabel.innerText = "IDLE";
  const sync = document.createElement("span");
  sync.id = "queue-sync-indicator";
  sync.className = "queue-sync";
  sync.hidden = true;
  sync.innerText = "Idle";
  phase.appendChild(phaseLabel);
  phase.appendChild(sync);
  statusPanel.appendChild(phase);

  const message = document.createElement("div");
  message.id = "queue-message";
  message.className = "queue-message";
  message.innerText = "Waiting for queue...";
  statusPanel.appendChild(message);

  const active = document.createElement("div");
  active.id = "queue-active-step";
  active.className = "queue-active";
  active.innerText = "Active Step: --";
  statusPanel.appendChild(active);

  return section;
}

function setSelectedLevel(index) {
  const newLevel = clampSelectedLevel(index);
  ntClient.addSample(toRobotPrefix + selectedLevelTopicName, newLevel);
  selectedLevel = newLevel;
  updateUI();
}

function handleBranchToggle(index) {
  const bit = 1 << index;
  let currentState = 0;
  let topic = l2TopicName;
  const levelIndex = getSelectedLevelIndex();
  switch (levelIndex) {
    case 0:
      currentState = l2State;
      topic = l2TopicName;
      break;
    case 1:
      currentState = l3State;
      topic = l3TopicName;
      break;
    case 2:
    default:
      currentState = l4State;
      topic = l4TopicName;
      break;
  }
  const newState = currentState ^ bit;
  ntClient.addSample(toRobotPrefix + topic, newState);
  if (levelIndex === 0) {
    l2State = newState;
  } else if (levelIndex === 1) {
    l3State = newState;
  } else {
    l4State = newState;
  }
  if ((newState & bit) > 0) {
    addReefStepFromBranch(index);
  }
  updateUI();
}

function handleAlgaeToggle(index) {
  const newState = algaeState ^ (1 << index);
  ntClient.addSample(toRobotPrefix + algaeTopicName, newState);
  algaeState = newState;
  updateUI();
}

function handleControlAction(control) {
  if (!control || !control.action) {
    return;
  }
  const action = control.action;
  let shouldRefresh = false;
  switch (action.type) {
    case "topic-delta":
      shouldRefresh = handleTopicDelta(action);
      break;
    case "topic-set":
      shouldRefresh = handleTopicSet(action);
      break;
    case "topic-toggle":
      shouldRefresh = handleTopicToggle(action);
      break;
    case "align-source":
      handleAlignSource(action);
      break;
    case "queue-command":
      if (action.command) {
        const topicPath = buildTopicPath(action, queueCommandTopicName);
        sendQueueCommand(action.command, action.extras, topicPath);
      }
      break;
    default:
      break;
  }
  if (shouldRefresh) {
    updateUI();
  }
}

function handleTopicDelta(action) {
  const topicInfo = resolveTopicInfo(action);
  if (!topicInfo || !topicInfo.topicPath) {
    return false;
  }
  const current = typeof topicInfo.getter === "function" ? topicInfo.getter() : 0;
  let next = current + (action.delta || 0);
  if (typeof action.min === "number") {
    next = Math.max(action.min, next);
  }
  if (typeof action.max === "number") {
    next = Math.min(action.max, next);
  }
  ntClient.addSample(topicInfo.topicPath, next);
  if (topicInfo.setter) {
    topicInfo.setter(next);
  }
  return true;
}

function handleTopicSet(action) {
  const topicInfo = resolveTopicInfo(action);
  if (!topicInfo || !topicInfo.topicPath) {
    return false;
  }
  ntClient.addSample(topicInfo.topicPath, action.value);
  if (topicInfo.setter) {
    topicInfo.setter(action.value);
  }
  return true;
}

function handleTopicToggle(action) {
  const topicInfo = resolveTopicInfo(action);
  if (!topicInfo || !topicInfo.topicPath) {
    return false;
  }
  const current = !!(topicInfo.getter ? topicInfo.getter() : action.defaultValue);
  const next = !current;
  ntClient.addSample(topicInfo.topicPath, next);
  if (topicInfo.setter) {
    topicInfo.setter(next);
  }
  return true;
}

function handleAlignSource(action) {
  const preference = normalizeSourcePreference(action && action.source);
  const topicPath = buildTopicPath(action, queueCommandTopicName);
  sendQueueCommand("ALIGN_SOURCE", { source: preference }, topicPath);
}

function normalizeSourcePreference(source) {
  if (typeof source !== "string") {
    return "NEAREST";
  }
  const token = source.trim().toUpperCase();
  if (token === "LEFT" || token === "RIGHT") {
    return token;
  }
  return "NEAREST";
}

function resolveTopicInfo(action) {
  if (!action) {
    return null;
  }
  const target = action.target;
  switch (target) {
    case "Level1":
      return { topicPath: buildTopicPath(action, l1TopicName), getter: () => l1State, setter: (v) => (l1State = v) };
    case "Level2":
      return { topicPath: buildTopicPath(action, l2TopicName), getter: () => l2State, setter: (v) => (l2State = v) };
    case "Level3":
      return { topicPath: buildTopicPath(action, l3TopicName), getter: () => l3State, setter: (v) => (l3State = v) };
    case "Level4":
      return { topicPath: buildTopicPath(action, l4TopicName), getter: () => l4State, setter: (v) => (l4State = v) };
    case "Coop":
      return { topicPath: buildTopicPath(action, coopTopicName), getter: () => coopState, setter: (v) => (coopState = v) };
    case "SelectedLevel":
      return {
        topicPath: buildTopicPath(action, selectedLevelTopicName),
        getter: () => selectedLevel,
        setter: (v) => (selectedLevel = clampSelectedLevel(v)),
      };
    default: {
      const topicPath = buildTopicPath(action);
      return topicPath ? { topicPath } : null;
    }
  }
}

function buildTopicPath(action, fallbackTopic) {
  const fromTable = buildTableTopicPath(action);
  if (fromTable) {
    return fromTable;
  }
  const topicOverride = action && action.topic;
  if (topicOverride) {
    const topic = String(topicOverride || "").trim();
    if (!topic) {
      return null;
    }
    if (topic.startsWith("/")) {
      return normalizeTopicPath(topic);
    }
    return toRobotPrefix + topic;
  }
  if (fallbackTopic) {
    return toRobotPrefix + fallbackTopic;
  }
  return null;
}

function buildTableTopicPath(action) {
  const table = action && typeof action.ntTable === "string" ? action.ntTable : "";
  const key = action && typeof action.ntKey === "string" ? action.ntKey : "";
  const cleanTable = normalizeSegment(table);
  const cleanKey = normalizeSegment(key);
  if (!cleanTable && !cleanKey) {
    return null;
  }
  const pieces = [];
  if (cleanTable) {
    pieces.push(cleanTable);
  }
  if (cleanKey) {
    pieces.push(cleanKey);
  }
  if (pieces.length === 0) {
    return null;
  }
  return "/" + pieces.join("/");
}

function normalizeSegment(value) {
  if (typeof value !== "string") {
    return "";
  }
  return value.replace(/^\/+|\/+$/g, "").trim();
}

function normalizeTopicPath(path) {
  if (typeof path !== "string") {
    return null;
  }
  const trimmed = path.trim();
  if (!trimmed) {
    return null;
  }
  return trimmed.startsWith("/") ? trimmed : "/" + trimmed;
}

function startNetworkTables() {
  if (ntSessionStarted) {
    return;
  }
  ntSessionStarted = true;
  ntClient.subscribe(ntSubscriptions, false, false, 0.02);
  ntClient.publishTopic(toRobotPrefix + selectedLevelTopicName, "int");
  ntClient.publishTopic(toRobotPrefix + l1TopicName, "int");
  ntClient.publishTopic(toRobotPrefix + l2TopicName, "int");
  ntClient.publishTopic(toRobotPrefix + l3TopicName, "int");
  ntClient.publishTopic(toRobotPrefix + l4TopicName, "int");
  ntClient.publishTopic(toRobotPrefix + algaeTopicName, "int");
  ntClient.publishTopic(toRobotPrefix + coopTopicName, "boolean");
  ntClient.publishTopic(toRobotPrefix + queueSpecTopicName, "string");
  ntClient.publishTopic(toRobotPrefix + queueCommandTopicName, "string");
  ntClient.connect();
}

// ***** QUERY PARAM HELPERS *****

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

// ***** STATE CACHE *****

let selectedLevel = 2; // 0 = L2, 1 = L3, 2 = L4
let l1State = 0;
let l2State = 0;
let l3State = 0;
let l4State = 0;
let algaeState = 0;
let coopState = false;
let isElims = false;
let queueState = { steps: [], manual: false };
const queueDragState = { active: false, fromIndex: null };

function isFiniteNumber(value) {
  if (typeof Number.isFinite === "function") {
    return Number.isFinite(value);
  }
  return isFinite(value);
}

function clampSelectedLevel(value) {
  const numeric = typeof value === "number" ? value : Number(value);
  if (!isFiniteNumber(numeric)) {
    return 2;
  }
  return Math.max(0, Math.min(2, Math.floor(numeric)));
}

function getSelectedLevelIndex() {
  return clampSelectedLevel(selectedLevel);
}

/** Update the full UI based on the state cache. */
function updateUI() {
  if (!templateConfig) {
    return;
  }
  const levelIndex = getSelectedLevelIndex();
  const theme = templateConfig.theme || {};
  const levelColors = Array.isArray(theme.levelBackgrounds)
    ? theme.levelBackgrounds
    : ["#5AB7ED", "#0F486C", "#dfbbfc"];
  if (levelColors[levelIndex]) {
    document.body.style.backgroundColor = levelColors[levelIndex];
  }

  let rpLevelCount = 0;
  const branchLength = (templateConfig.reef && templateConfig.reef.branchNodes && templateConfig.reef.branchNodes.length) || 12;

  uiRefs.counters.forEach((entry) => {
    const source = entry.config && entry.config.source;
    if (source === "Level1") {
      entry.valueEl.innerText = l1State;
      if (l1State >= 7) {
        rpLevelCount++;
      }
    } else if (source === "Level2" || source === "Level3" || source === "Level4") {
      const levelState = source === "Level2" ? l2State : source === "Level3" ? l3State : l4State;
      let count = 0;
      for (let i = 0; i < branchLength; i++) {
        if (((1 << i) & levelState) > 0) {
          count++;
        }
      }
      entry.valueEl.innerText = count === branchLength ? "\u2705" : count;
      if (count >= 7) {
        rpLevelCount++;
      }
    }
    if (typeof entry.config.selectsLevel === "number") {
      if (entry.config.selectsLevel === levelIndex) {
        entry.area.classList.add("active");
      } else {
        entry.area.classList.remove("active");
      }
    }
  });

  uiRefs.branchNodes.forEach((ref, index) => {
    const levelState = [l2State, l3State, l4State][levelIndex] || 0;
    if (((1 << index) & levelState) > 0) {
      ref.element.classList.add("active");
    } else {
      ref.element.classList.remove("active");
    }
  });

  uiRefs.algaeNodes.forEach((ref, index) => {
    if (((1 << index) & algaeState) > 0) {
      ref.element.classList.add("active");
    } else {
      ref.element.classList.remove("active");
    }
  });

  uiRefs.controls.forEach((control) => {
    if (control.config && control.config.action && control.config.action.type === "topic-toggle") {
      if (control.config.action.target === "Coop") {
        control.element.classList.toggle("active", !!coopState);
      }
    }
  });

  if (uiRefs.reefFlag) {
    uiRefs.reefFlag.hidden = isElims || rpLevelCount < (coopState ? 3 : 4);
  }

  if (isElims) {
    document.body.classList.add("elims");
  } else {
    document.body.classList.remove("elims");
  }
}

// ***** QUEUE MANAGEMENT *****

function applyQueueState(raw) {
  try {
    const payload = JSON.parse(raw);
    if (!payload || typeof payload !== "object") {
      return;
    }
    queueState = payload;
    renderQueue();
  } catch (error) {
    console.error("Failed to parse queue state", error);
  }
}

function renderQueue() {
  const list = document.getElementById("queue-plan");
  if (!list) {
    return;
  }
  list.innerHTML = "";

  const steps = queueState && Array.isArray(queueState.steps) ? queueState.steps : [];
  const manualEnabled = !!(queueState && queueState.manual);
  if (steps.length === 0) {
    const placeholder = document.createElement("li");
    placeholder.className = "queue-placeholder";
    placeholder.innerText = "No steps queued.";
    list.appendChild(placeholder);
  } else {
    steps.forEach((step, index) => {
      const status = (step.status ? step.status : "PENDING").toUpperCase();
      const li = document.createElement("li");
      li.className = `queue-item status-${status.toLowerCase()}`;
      li.setAttribute("data-queue-index", String(index));
      li.setAttribute("draggable", "true");
      li.innerHTML = `
        <div>
          <div class="queue-item-title">${formatStepTitle(step)}</div>
          <div class="queue-item-meta">${formatStepMeta(step)}</div>
        </div>
        <div class="queue-item-actions">
          <span class="queue-status-badge">${statusLabel(status)}</span>
        </div>
      `;
      list.appendChild(li);
    });
  }

  const phaseEl = document.getElementById("queue-phase");
  if (phaseEl) {
    phaseEl.innerText = queueState && queueState.phase ? queueState.phase : "IDLE";
  }

  const syncEl = document.getElementById("queue-sync-indicator");
  if (syncEl) {
    const running = queueState && queueState.running;
    syncEl.hidden = !!running;
    syncEl.innerText = running ? "" : manualEnabled ? "Manual" : "Idle";
  }

  const messageEl = document.getElementById("queue-message");
  if (messageEl) {
    const base = queueState && queueState.message ? queueState.message : "Waiting for queue...";
    messageEl.innerText = base;
  }

  const activeEl = document.getElementById("queue-active-step");
  if (activeEl) {
    const active =
      queueState && queueState.activeStep
        ? queueState.activeStep
        : manualEnabled
          ? "Manual Control"
          : "--";
    activeEl.innerText = `Active Step: ${active}`;
  }

  const manualToggle = document.getElementById("queue-manual-toggle");
  if (manualToggle) {
    const manualConfig = templateConfig && templateConfig.queue && templateConfig.queue.manualToggle;
    const queueEnabledLabel = (manualConfig && manualConfig.queueEnabledLabel) || "Disable Queue";
    const manualEnabledLabel = (manualConfig && manualConfig.manualEnabledLabel) || "Enable Queue";
    manualToggle.innerText = manualEnabled ? manualEnabledLabel : queueEnabledLabel;
    manualToggle.classList.toggle("queue-control-active", manualEnabled);
    manualToggle.setAttribute("aria-pressed", manualEnabled ? "true" : "false");
  }

  const startButton = document.querySelector('[data-queue-command="start"]');
  if (startButton) {
    startButton.disabled = manualEnabled;
    startButton.title = manualEnabled ? "Disable manual mode to run the queue." : "";
  }
}

function formatStepTitle(step) {
  if (step.type === "REEF") {
    const face = step.face != null ? step.face : 1;
    const sideToken = (step.side ? step.side : "LEFT").toLowerCase();
    const prettySide = sideToken.charAt(0).toUpperCase() + sideToken.slice(1);
    return `Reef - Face ${face} (${prettySide})`;
  }
  const source = (step.source ? step.source : "NEAREST").toLowerCase();
  const prettySource = source.charAt(0).toUpperCase() + source.slice(1);
  return `Source - ${prettySource}`;
}

function formatStepMeta(step) {
  if (step.type === "REEF") {
    return `Level ${step.level ? step.level : "L3"}`;
  }
  return "Align to station";
}

function statusLabel(status) {
  return status.replace(/_/g, " ");
}

function sendQueueCommand(command, extras, topicPathOverride) {
  if (!command) {
    return;
  }
  const payload = Object.assign(
    { command: command.toUpperCase(), at: Date.now() },
    extras || {}
  );
  const topicPath = topicPathOverride || toRobotPrefix + queueCommandTopicName;
  ntClient.addSample(topicPath, JSON.stringify(payload));
}

function sendAddStep(step) {
  if (!step || !step.type) {
    return;
  }
  sendQueueCommand("ADD", { step });
}

function addReefStepFromBranch(index) {
  const face = Math.floor(index / 2) + 1;
  const side = index % 2 === 0 ? "LEFT" : "RIGHT";
  const templateNode = templateConfig && templateConfig.reef && templateConfig.reef.branchNodes
    ? templateConfig.reef.branchNodes[index]
    : null;
  const nodeFace = templateNode && templateNode.face ? templateNode.face : face;
  const nodeSide = templateNode && templateNode.side ? templateNode.side : side;
  const levelTokens = ["L2", "L3", "L4"];
  const levelIndex = getSelectedLevelIndex();
  const level = levelTokens[levelIndex] || "L3";
  sendAddStep({ type: "SOURCE", source: nodeSide });
  sendAddStep({ type: "REEF", face: nodeFace, side: nodeSide, level });
}

function initQueueUi() {
  if (queueUiInitialized) {
    return;
  }
  queueUiInitialized = true;

  const queueList = document.getElementById("queue-plan");
  if (queueList) {
    queueList.addEventListener("click", (event) => {
      if (queueDragState.active) {
        return;
      }
      const item = findQueueItemElement(event.target, queueList);
      if (!item) {
        return;
      }
      const index = Number(item.getAttribute("data-queue-index"));
      if (!isNaN(index)) {
        sendQueueCommand("REMOVE", { index });
      }
    });
    enableQueueReordering(queueList);
  }

  document
    .querySelectorAll("[data-queue-command]")
    .forEach((button) =>
      button.addEventListener("click", () => sendQueueCommand(button.dataset.queueCommand))
    );

  const manualToggle = document.getElementById("queue-manual-toggle");
  if (manualToggle) {
    manualToggle.addEventListener("click", () => {
      const manualEnabled = !!(queueState && queueState.manual);
      sendQueueCommand("MANUAL", { manual: !manualEnabled });
    });
  }

  renderQueue();
}

function findQueueItemElement(element, container) {
  let node = element;
  while (node && node !== container) {
    if (node.hasAttribute && node.hasAttribute("data-queue-index")) {
      return node;
    }
    node = node.parentElement;
  }
  return null;
}

function enableQueueReordering(queueList) {
  queueList.addEventListener("dragstart", (event) => {
    const item = findQueueItemElement(event.target, queueList);
    if (!item) {
      return;
    }
    const index = Number(item.getAttribute("data-queue-index"));
    if (isNaN(index)) {
      return;
    }
    queueDragState.active = true;
    queueDragState.fromIndex = index;
    item.classList.add("dragging");
    queueList.classList.add("queue-dragging");
    if (event.dataTransfer) {
      event.dataTransfer.effectAllowed = "move";
      event.dataTransfer.setData("text/plain", String(index));
    }
  });

  queueList.addEventListener("dragover", (event) => {
    if (!queueDragState.active) {
      return;
    }
    event.preventDefault();
  });

  queueList.addEventListener("drop", (event) => {
    if (!queueDragState.active) {
      return;
    }
    event.preventDefault();
    const dropIndex = computeDropIndex(event, queueList);
    handleQueueReorder(dropIndex);
    finishQueueDrag(queueList);
  });

  queueList.addEventListener("dragend", () => {
    finishQueueDrag(queueList);
  });
}

function computeDropIndex(event, queueList) {
  const items = Array.from(queueList.querySelectorAll(".queue-item"));
  if (items.length === 0) {
    return 0;
  }
  const pointerY = typeof event.clientY === "number" ? event.clientY : 0;
  for (const item of items) {
    const rect = item.getBoundingClientRect();
    if (pointerY < rect.top + rect.height / 2) {
      const index = Number(item.getAttribute("data-queue-index"));
      return isNaN(index) ? 0 : index;
    }
  }
  return items.length;
}

function handleQueueReorder(dropIndex) {
  if (queueDragState.fromIndex == null || dropIndex == null) {
    return;
  }
  const steps = queueState && Array.isArray(queueState.steps) ? queueState.steps : [];
  if (steps.length < 2) {
    return;
  }
  const fromIndex = queueDragState.fromIndex;
  let toIndex = Math.max(0, Math.min(dropIndex, steps.length));
  if (fromIndex < toIndex) {
    toIndex -= 1;
  }
  if (fromIndex === toIndex) {
    return;
  }
  sendQueueCommand("MOVE", { fromIndex, toIndex });
}

function finishQueueDrag(queueList) {
  queueDragState.active = false;
  queueDragState.fromIndex = null;
  if (!queueList) {
    return;
  }
  queueList.classList.remove("queue-dragging");
  queueList
    .querySelectorAll(".queue-item.dragging")
    .forEach((node) => node.classList.remove("dragging"));
}

// ***** BUTTON BINDINGS *****

let isTouch = false;

function bind(element, callback) {
  let activate = (touchEvent) => {
    if (touchEvent) {
      isTouch = true;
    }
    if (isTouch == touchEvent) {
      callback();
    }
  };

  element.addEventListener("touchstart", () => activate(true));
  element.addEventListener("mousedown", () => activate(false));
  element.addEventListener("contextmenu", (event) => {
    event.preventDefault();
    activate(false);
  });
}

let lastMouseEvent = 0;
window.addEventListener("mousemove", () => {
  let now = new Date().getTime();
  if (now - lastMouseEvent < 50) {
    isTouch = false;
  }
  lastMouseEvent = now;
});

// ***** REEF CANVAS *****

function initReefCanvas() {
  if (canvasInitialized) {
    return;
  }
  const canvas = uiRefs.reefCanvas;
  if (!canvas) {
    return;
  }
  canvasInitialized = true;
  const context = canvas.getContext("2d");

  let render = () => {
    const devicePixelRatio = window.devicePixelRatio || 1;
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    canvas.width = width * devicePixelRatio;
    canvas.height = height * devicePixelRatio;
    context.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
    context.clearRect(0, 0, width, height);

    const corners = [
      [width * 0.74, height * 0.9],
      [width * 0.26, height * 0.9],
      [width * 0.03, height * 0.5],
      [width * 0.26, height * 0.1],
      [width * 0.74, height * 0.1],
      [width * 0.97, height * 0.5],
    ];

    context.beginPath();
    corners.forEach((corner) => {
      context.moveTo(width * 0.5, height * 0.5);
      context.lineTo(...corner);
    });
    corners.forEach((corner, index) => {
      if (index == 0) {
        context.moveTo(...corner);
      } else {
        context.lineTo(...corner);
      }
    });
    context.closePath();

    context.strokeStyle = "black";
    context.stroke();
  };

  render();
  window.addEventListener("resize", render);
}
