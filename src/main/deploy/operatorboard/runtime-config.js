import { NT4_Client } from "./NT4.js";

const defaultNt4Port = 5810;
const PROFILE_STORAGE_KEY = "operatorboard.runtime.profiles.v1";
const queryParams = new URLSearchParams(window.location.search);
const { host: ntHost, port: ntPort } = resolveNtConnectionParams(queryParams);

const SUBSYSTEM_KEYS = ["intake", "indexer", "shooter", "intakePivot", "shooterPivot", "turret"];
const SIGNAL_KEYS = [
  "IDENTITY",
  "CONNECTION",
  "FAULTS",
  "TEMPERATURE",
  "VOLTAGE",
  "CURRENT",
  "POSITION",
  "VELOCITY",
  "APPLIED_OUTPUT",
  "TARGET",
  "CLOSED_LOOP",
  "LIMIT_SWITCH",
  "SENSOR_STATE",
  "HEALTH",
  "CONFIG_SNAPSHOT",
];

const contract = {
  base: "/OperatorBoard/v1",
  toRobot: "/OperatorBoard/v1/ToRobot/",
  toDashboard: "/OperatorBoard/v1/ToDashboard/",
  keys: {
    runtimeProfileSpec: "RuntimeProfileSpec",
    applyRuntimeProfile: "ApplyRuntimeProfile",
    resetRuntimeProfile: "ResetRuntimeProfile",
    runtimeProfileState: "RuntimeProfileState",
    runtimeProfileStatus: "RuntimeProfileStatus",
  },
};

const state = {
  ntConnected: false,
  runtimeProfileStatus: "READY",
  presets: loadPresets(),
  ntTopics: new Map(),
  topicViewerEnabled: false,
};

const ui = {
  ntStatus: null,
  profileStatus: null,
  summaryMode: null,
  summaryModeDetail: null,
  summaryTuning: null,
  summaryTuningDetail: null,
  summaryDebug: null,
  summaryDebugDetail: null,
  summarySignals: null,
  summarySignalsDetail: null,
  loggingMode: null,
  loggingModeHelp: null,
  tuningEnabled: null,
  subsystemDebugHelp: null,
  subsystemDebugCount: null,
  subsystemDebugList: null,
  loggedSignalHelp: null,
  loggedSignalCount: null,
  loggedSignalList: null,
  publishedSignalHelp: null,
  publishedSignalCount: null,
  publishedSignalList: null,
  applyProfile: null,
  resetProfile: null,
  savePreset: null,
  presetNameInput: null,
  presetStatus: null,
  presetList: null,
  presetEmpty: null,
  jsonPreview: null,
  ntMetrics: null,
  topicFilter: null,
  toggleTopicViewer: null,
  topicViewerStatus: null,
  topicEmpty: null,
  topicList: null,
};

let ntClient = null;
let runtimeSubscriptionId = null;
let topicViewerSubscriptionId = null;

window.addEventListener("DOMContentLoaded", () => {
  bindUi();
  renderChecklist(ui.subsystemDebugList, SUBSYSTEM_KEYS, "subsystem");
  renderChecklist(ui.loggedSignalList, SIGNAL_KEYS, "logged");
  renderChecklist(ui.publishedSignalList, SIGNAL_KEYS, "published");
  renderPresets();
  renderTopicViewer();
  attachEvents();
  startNetworkTables();
  updatePreview();
  window.setInterval(() => {
    if (state.topicViewerEnabled) {
      renderTopicViewer();
    }
  }, 500);
});

function bindUi() {
  ui.ntStatus = document.getElementById("nt-status");
  ui.profileStatus = document.getElementById("profile-status");
  ui.summaryMode = document.getElementById("summary-mode");
  ui.summaryModeDetail = document.getElementById("summary-mode-detail");
  ui.summaryTuning = document.getElementById("summary-tuning");
  ui.summaryTuningDetail = document.getElementById("summary-tuning-detail");
  ui.summaryDebug = document.getElementById("summary-debug");
  ui.summaryDebugDetail = document.getElementById("summary-debug-detail");
  ui.summarySignals = document.getElementById("summary-signals");
  ui.summarySignalsDetail = document.getElementById("summary-signals-detail");
  ui.loggingMode = document.getElementById("logging-mode");
  ui.loggingModeHelp = document.getElementById("logging-mode-help");
  ui.tuningEnabled = document.getElementById("tuning-enabled");
  ui.subsystemDebugHelp = document.getElementById("subsystem-debug-help");
  ui.subsystemDebugCount = document.getElementById("subsystem-debug-count");
  ui.subsystemDebugList = document.getElementById("subsystem-debug-list");
  ui.loggedSignalHelp = document.getElementById("logged-signal-help");
  ui.loggedSignalCount = document.getElementById("logged-signal-count");
  ui.loggedSignalList = document.getElementById("logged-signal-list");
  ui.publishedSignalHelp = document.getElementById("published-signal-help");
  ui.publishedSignalCount = document.getElementById("published-signal-count");
  ui.publishedSignalList = document.getElementById("published-signal-list");
  ui.applyProfile = document.getElementById("apply-profile");
  ui.resetProfile = document.getElementById("reset-profile");
  ui.savePreset = document.getElementById("save-preset");
  ui.presetNameInput = document.getElementById("preset-name-input");
  ui.presetStatus = document.getElementById("preset-status");
  ui.presetList = document.getElementById("preset-list");
  ui.presetEmpty = document.getElementById("preset-empty");
  ui.jsonPreview = document.getElementById("json-preview");
  ui.ntMetrics = document.getElementById("nt-metrics");
  ui.topicFilter = document.getElementById("topic-filter");
  ui.toggleTopicViewer = document.getElementById("toggle-topic-viewer");
  ui.topicViewerStatus = document.getElementById("topic-viewer-status");
  ui.topicEmpty = document.getElementById("topic-empty");
  ui.topicList = document.getElementById("topic-list");
}

function attachEvents() {
  ui.applyProfile.addEventListener("click", applyProfile);
  ui.resetProfile.addEventListener("click", resetProfile);
  ui.savePreset.addEventListener("click", savePreset);
  ui.loggingMode.addEventListener("change", updatePreview);
  ui.tuningEnabled.addEventListener("change", updatePreview);
  ui.topicFilter.addEventListener("input", renderTopicViewer);
  ui.toggleTopicViewer.addEventListener("click", toggleTopicViewer);
  document.addEventListener("change", (event) => {
    if (
      event.target.matches(
        'input[data-scope="subsystem"], input[data-scope="logged"], input[data-scope="published"]'
      )
    ) {
      updatePreview();
    }
  });
}

function renderChecklist(container, keys, scope) {
  container.innerHTML = "";
  keys.forEach((key) => {
    const label = document.createElement("label");
    label.className = "check-item";

    const input = document.createElement("input");
    input.type = "checkbox";
    input.dataset.scope = scope;
    input.value = key;

    const text = document.createElement("span");
    text.className = "check-item__label";
    text.textContent = key;

    label.append(input, text);
    container.append(label);
  });
}

function startNetworkTables() {
  ntClient = new NT4_Client(
    ntHost,
    "RuntimeConfig",
    handleTopicAnnounce,
    handleTopicUnannounce,
    (topic, _, value) => handleTopicUpdate(topic, value),
    () => setNtConnected(true),
    () => setNtConnected(false),
    {
      port: ntPort,
      secure: window.location.protocol === "https:",
    }
  );

  runtimeSubscriptionId = ntClient.subscribe(
    [
      contract.toDashboard + contract.keys.runtimeProfileState,
      contract.toDashboard + contract.keys.runtimeProfileStatus,
    ],
    false,
    false,
    0.1
  );
  ntClient.publishTopic(contract.toRobot + contract.keys.runtimeProfileSpec, "string");
  ntClient.publishTopic(contract.toRobot + contract.keys.applyRuntimeProfile, "boolean");
  ntClient.publishTopic(contract.toRobot + contract.keys.resetRuntimeProfile, "boolean");
  ntClient.connect();
}

function setNtConnected(connected) {
  state.ntConnected = connected;
  ui.ntStatus.textContent = connected ? "NT Online" : "NT Offline";
  ui.ntStatus.className = connected ? "chip chip--ok" : "chip chip--bad";
  updateTopicViewerUi();
  renderTopicViewer();
}

function toggleTopicViewer() {
  state.topicViewerEnabled = !state.topicViewerEnabled;
  if (ntClient) {
    if (state.topicViewerEnabled) {
      if (topicViewerSubscriptionId == null) {
        topicViewerSubscriptionId = ntClient.subscribe([contract.base], true, false, 0.1);
      }
    } else if (topicViewerSubscriptionId != null) {
      ntClient.unsubscribe(topicViewerSubscriptionId);
      topicViewerSubscriptionId = null;
      for (const key of Array.from(state.ntTopics.keys())) {
        if (
          key !== contract.toDashboard + contract.keys.runtimeProfileState &&
          key !== contract.toDashboard + contract.keys.runtimeProfileStatus
        ) {
          state.ntTopics.delete(key);
        }
      }
    }
  }
  updateTopicViewerUi();
  renderTopicViewer();
}

function updateTopicViewerUi() {
  if (ui.toggleTopicViewer) {
    ui.toggleTopicViewer.textContent = state.topicViewerEnabled
      ? "Disable Live Viewer"
      : "Enable Live Viewer";
    ui.toggleTopicViewer.className = state.topicViewerEnabled
      ? "button button--primary"
      : "button";
  }
  if (ui.topicViewerStatus) {
    ui.topicViewerStatus.textContent = state.topicViewerEnabled
      ? "Viewer enabled. Full OperatorBoard namespace subscription is active."
      : "Viewer disabled. Enable only when you actively need namespace profiling.";
  }
}

function handleTopicAnnounce(topic) {
  if (!topic || !topic.name || !topic.name.startsWith(contract.base)) {
    return;
  }
  if (
    !state.topicViewerEnabled &&
    topic.name !== contract.toDashboard + contract.keys.runtimeProfileState &&
    topic.name !== contract.toDashboard + contract.keys.runtimeProfileStatus
  ) {
    return;
  }
  const existing = state.ntTopics.get(topic.name) || {};
  state.ntTopics.set(topic.name, {
    ...existing,
    name: topic.name,
    type: topic.type || "unknown",
    properties: topic.properties || {},
  });
  renderTopicViewer();
}

function handleTopicUnannounce(topic) {
  if (!topic || !topic.name) {
    return;
  }
  state.ntTopics.delete(topic.name);
  renderTopicViewer();
}

function handleTopicUpdate(topic, value) {
  if (!topic || !topic.name) {
    return;
  }
  if (
    !state.topicViewerEnabled &&
    topic.name !== contract.toDashboard + contract.keys.runtimeProfileState &&
    topic.name !== contract.toDashboard + contract.keys.runtimeProfileStatus
  ) {
    return;
  }

  const existing = state.ntTopics.get(topic.name) || {
    name: topic.name,
    type: topic.type || typeof value,
    properties: {},
  };
  state.ntTopics.set(topic.name, {
    ...existing,
    value,
    updatedAt: Date.now(),
  });

  const key = topic.name.replace(contract.toDashboard, "");
  if (key === contract.keys.runtimeProfileState && typeof value === "string" && value.trim()) {
    applyProfileToForm(parseProfile(value));
    updatePreview();
  } else if (key === contract.keys.runtimeProfileStatus) {
    state.runtimeProfileStatus = String(value ?? "READY");
    ui.profileStatus.textContent = state.runtimeProfileStatus;
  }

  renderTopicViewer();
}

function parseProfile(raw) {
  try {
    return JSON.parse(raw);
  } catch {
    return {
      loggingMode: "COMP",
      tuningEnabled: false,
      debugSubsystems: [],
      loggedSignals: [],
      publishedSignals: [],
    };
  }
}

function applyProfileToForm(profile) {
  ui.loggingMode.value = profile.loggingMode || "COMP";
  ui.tuningEnabled.checked = Boolean(profile.tuningEnabled);
  setCheckedValues("subsystem", profile.debugSubsystems || []);
  setCheckedValues("logged", profile.loggedSignals || []);
  setCheckedValues("published", profile.publishedSignals || []);
}

function setCheckedValues(scope, values) {
  const selected = new Set(values);
  document.querySelectorAll(`input[data-scope="${scope}"]`).forEach((input) => {
    input.checked = selected.has(input.value);
  });
}

function getCheckedValues(scope) {
  return Array.from(document.querySelectorAll(`input[data-scope="${scope}"]:checked`)).map(
    (input) => input.value
  );
}

function buildProfile() {
  return {
    loggingMode: ui.loggingMode.value,
    tuningEnabled: ui.tuningEnabled.checked,
    debugSubsystems: getCheckedValues("subsystem"),
    loggedSignals: getCheckedValues("logged"),
    publishedSignals: getCheckedValues("published"),
  };
}

function updatePreview() {
  const profile = buildProfile();
  ui.jsonPreview.textContent = JSON.stringify(profile, null, 2);
  renderProfileSummary(profile);
}

function applyProfile() {
  const payload = JSON.stringify(buildProfile());
  ntClient.addSample(contract.toRobot + contract.keys.runtimeProfileSpec, payload);
  pulseBoolean(contract.toRobot + contract.keys.applyRuntimeProfile);
  state.runtimeProfileStatus = "APPLY_REQUESTED";
  ui.profileStatus.textContent = state.runtimeProfileStatus;
}

function resetProfile() {
  pulseBoolean(contract.toRobot + contract.keys.resetRuntimeProfile);
  state.runtimeProfileStatus = "RESET_REQUESTED";
  ui.profileStatus.textContent = state.runtimeProfileStatus;
}

function pulseBoolean(topic) {
  ntClient.addSample(topic, true);
  window.setTimeout(() => ntClient.addSample(topic, false), 80);
}

function savePreset() {
  const name = ui.presetNameInput?.value?.trim();
  if (!name) {
    if (ui.presetStatus) {
      ui.presetStatus.textContent = "Enter a preset name before saving.";
    }
    return;
  }
  state.presets = state.presets.filter((preset) => preset.name !== name);
  state.presets.unshift({ name, profile: buildProfile() });
  persistPresets();
  if (ui.presetNameInput) {
    ui.presetNameInput.value = "";
  }
  if (ui.presetStatus) {
    ui.presetStatus.textContent = `Saved preset "${name}".`;
  }
  renderPresets();
}

function renderPresets() {
  ui.presetList.innerHTML = "";
  ui.presetEmpty.style.display = state.presets.length ? "none" : "block";
  state.presets.forEach((preset) => {
    const row = document.createElement("div");
    row.className = "preset-item";

    const name = document.createElement("div");
    name.className = "preset-item__name";
    name.textContent = preset.name;

    const loadButton = document.createElement("button");
    loadButton.className = "button";
    loadButton.type = "button";
    loadButton.textContent = "Load";
    loadButton.addEventListener("click", () => {
      applyProfileToForm(preset.profile);
      updatePreview();
      if (ui.presetStatus) {
        ui.presetStatus.textContent = `Loaded preset "${preset.name}".`;
      }
    });

    const deleteButton = document.createElement("button");
    deleteButton.className = "button";
    deleteButton.type = "button";
    deleteButton.textContent = "Delete";
    deleteButton.addEventListener("click", () => {
      state.presets = state.presets.filter((entry) => entry.name !== preset.name);
      persistPresets();
      if (ui.presetStatus) {
        ui.presetStatus.textContent = `Deleted preset "${preset.name}".`;
      }
      renderPresets();
    });

    row.append(name, loadButton, deleteButton);
    ui.presetList.append(row);
  });
}

function renderProfileSummary(profile) {
  const debugCount = profile.debugSubsystems.length;
  const loggedCount = profile.loggedSignals.length;
  const publishedCount = profile.publishedSignals.length;

  if (ui.summaryMode) {
    ui.summaryMode.textContent = profile.loggingMode;
  }
  if (ui.summaryModeDetail) {
    ui.summaryModeDetail.textContent =
      profile.loggingMode === "DEBUG"
        ? "Full debug defaults are active unless you narrow them below."
        : "Compact defaults for match use.";
  }
  if (ui.loggingModeHelp) {
    ui.loggingModeHelp.textContent =
      profile.loggingMode === "DEBUG"
        ? "DEBUG falls back to the full signal set when lists are empty."
        : "COMP keeps the compact match-safe defaults.";
  }

  if (ui.summaryTuning) {
    ui.summaryTuning.textContent = profile.tuningEnabled ? "Enabled" : "Disabled";
  }
  if (ui.summaryTuningDetail) {
    ui.summaryTuningDetail.textContent = profile.tuningEnabled
      ? "Robot-side tunable numbers stay live."
      : "Robot-side tunable numbers stay locked.";
  }

  if (ui.summaryDebug) {
    ui.summaryDebug.textContent = `${debugCount} Selected`;
  }
  if (ui.summaryDebugDetail) {
    ui.summaryDebugDetail.textContent = debugCount
      ? `Expanded debug visibility for ${debugCount} subsystem(s).`
      : "Extra debug coverage can be scoped by subsystem.";
  }
  if (ui.subsystemDebugCount) {
    ui.subsystemDebugCount.textContent = `${debugCount} selected`;
  }
  if (ui.subsystemDebugHelp) {
    ui.subsystemDebugHelp.textContent =
      profile.loggingMode === "COMP"
        ? "In COMP mode, checked subsystems get expanded debug logging and publishing."
        : "DEBUG mode already exposes all subsystems, so this list acts as documentation.";
  }

  if (ui.summarySignals) {
    ui.summarySignals.textContent =
      loggedCount === 0 && publishedCount === 0 ? "Defaults" : `${loggedCount}/${publishedCount}`;
  }
  if (ui.summarySignalsDetail) {
    ui.summarySignalsDetail.textContent =
      loggedCount === 0 && publishedCount === 0
        ? "Empty signal lists fall back to the active mode defaults."
        : `${loggedCount} logged group(s), ${publishedCount} published group(s).`;
  }
  if (ui.loggedSignalCount) {
    ui.loggedSignalCount.textContent = loggedCount === 0 ? "Defaults" : `${loggedCount} selected`;
  }
  if (ui.loggedSignalHelp) {
    ui.loggedSignalHelp.textContent =
      loggedCount === 0
        ? "Choose an explicit log set, or leave everything unchecked to use the mode defaults."
        : "Only the selected mechanism telemetry groups will be explicitly logged.";
  }
  if (ui.publishedSignalCount) {
    ui.publishedSignalCount.textContent =
      publishedCount === 0 ? "Defaults" : `${publishedCount} selected`;
  }
  if (ui.publishedSignalHelp) {
    ui.publishedSignalHelp.textContent =
      publishedCount === 0
        ? "Choose which mechanism telemetry groups should be pushed live to NetworkTables."
        : "Only the selected mechanism telemetry groups will be explicitly published.";
  }
}

function renderTopicViewer() {
  if (!ui.topicList || !ui.topicEmpty || !ui.ntMetrics) {
    return;
  }
  if (!state.topicViewerEnabled) {
    ui.ntMetrics.textContent = state.ntConnected
      ? "Viewer disabled"
      : "Viewer disabled • offline";
    ui.topicList.innerHTML = "";
    ui.topicEmpty.style.display = "block";
    ui.topicEmpty.textContent =
      "Live topic viewer is disabled. Enable it only when you need full namespace profiling.";
    return;
  }
  const filter = (ui.topicFilter?.value || "").trim().toLowerCase();
  const topics = Array.from(state.ntTopics.values())
    .filter((topic) => topic && topic.name)
    .filter((topic) => {
      if (!filter) {
        return true;
      }
      const haystack = `${topic.name} ${formatTopicValue(topic.value)}`.toLowerCase();
      return haystack.includes(filter);
    })
    .sort((a, b) => a.name.localeCompare(b.name));

  const latencyUs =
    ntClient && typeof ntClient.getNetworkLatency_us === "function"
      ? ntClient.getNetworkLatency_us()
      : null;
  const latencyMs = Number.isFinite(latencyUs) ? latencyUs / 1000.0 : null;
  ui.ntMetrics.textContent =
    `${topics.length} topic${topics.length === 1 ? "" : "s"}`
    + (latencyMs !== null ? ` • ${latencyMs.toFixed(1)} ms latency` : "")
    + (state.ntConnected ? "" : " • offline");

  ui.topicList.innerHTML = "";
  ui.topicEmpty.style.display = topics.length ? "none" : "block";
  ui.topicEmpty.textContent = "Waiting for NetworkTables topics.";
  topics.forEach((topic) => {
    const row = document.createElement("div");
    row.className = "topic-row";

    const header = document.createElement("div");
    header.className = "topic-row__header";

    const name = document.createElement("div");
    name.className = "topic-row__name";
    name.textContent = topic.name;

    const meta = document.createElement("div");
    meta.className = "topic-row__meta";
    meta.textContent =
      `${topic.type || "unknown"}`
      + (Number.isFinite(topic.updatedAt) ? ` • ${formatAge(Date.now() - topic.updatedAt)}` : "");

    const value = document.createElement("pre");
    value.className = "topic-row__value";
    value.textContent = formatTopicValue(topic.value);

    header.append(name, meta);
    row.append(header, value);
    ui.topicList.append(row);
  });
}

function loadPresets() {
  try {
    const raw = window.localStorage.getItem(PROFILE_STORAGE_KEY);
    if (!raw) {
      return [];
    }
    const parsed = JSON.parse(raw);
    return Array.isArray(parsed) ? parsed : [];
  } catch {
    return [];
  }
}

function persistPresets() {
  window.localStorage.setItem(PROFILE_STORAGE_KEY, JSON.stringify(state.presets));
}

function formatTopicValue(value) {
  if (value === undefined) {
    return "(no sample yet)";
  }
  if (typeof value === "string") {
    return value;
  }
  if (typeof value === "number" || typeof value === "boolean") {
    return String(value);
  }
  try {
    return JSON.stringify(value);
  } catch {
    return String(value);
  }
}

function formatAge(ageMs) {
  if (!Number.isFinite(ageMs) || ageMs < 0) {
    return "--";
  }
  if (ageMs < 1000) {
    return `${Math.round(ageMs)} ms ago`;
  }
  return `${(ageMs / 1000).toFixed(1)} s ago`;
}

function resolveNtConnectionParams(params) {
  const host = params.get("ntHost") || params.get("host") || window.location.hostname || "localhost";
  const portRaw = params.get("ntPort") || params.get("port");
  const portParsed = portRaw ? Number(portRaw) : NaN;
  const port = Number.isFinite(portParsed) ? portParsed : defaultNt4Port;
  return { host, port };
}
