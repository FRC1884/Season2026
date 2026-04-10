import { NT4_Client } from "./NT4.js";

const defaultNt4Port = 5810;

let FIELD_LENGTH_METERS = 17.548;
let FIELD_WIDTH_METERS = 8.052;
let SAFE_AUTO_MAX_X_METERS = 8.15;
let ROBOT_LENGTH_METERS = 0.6985;
let ROBOT_WIDTH_METERS = 0.6985;
const FIELD_MARGIN_METERS = 0.28;
const ROUTE_CORNER_MARGIN_METERS = 0.18;
const STALE_MS = 1500;
const PRESET_STORAGE_KEY = "operatorboard.rebuilt.presets.v1";
const SELECTED_AUTO_STORAGE_KEY = "operatorboard.rebuilt.selectedAuto.v1";
const CUSTOM_SPOT_STORAGE_KEY = "operatorboard.rebuilt.customSpots.v1";
const CUSTOM_ZONE_STORAGE_KEY = "operatorboard.rebuilt.customZones.v1";
const DRAFT_STORAGE_KEY = "operatorboard.rebuilt.autoDraft.v1";

const contract = {
  base: "/OperatorBoard/v1",
  toRobot: "/OperatorBoard/v1/ToRobot/",
  toDashboard: "/OperatorBoard/v1/ToDashboard/",
  keys: {
    requestedState: "RequestedState",
    autoStateEnable: "AutoStateEnable",
    playSwerveMusic: "PlaySwerveMusic",
    stopSwerveMusic: "StopSwerveMusic",
    swerveMusicVolume: "SwerveMusicVolume",
    rollLogs: "RollLogs",
    cleanLogs: "CleanLogs",
    requestIntakeDeployRezero: "RequestIntakeDeployRezero",
    cancelIntakeDeployRezero: "CancelIntakeDeployRezero",
    requestManualIntakeDeployZeroSeek: "RequestManualIntakeDeployZeroSeek",
    cancelManualIntakeDeployZeroSeek: "CancelManualIntakeDeployZeroSeek",
    selectedAutoId: "SelectedAutoId",
    autoQueueSpec: "AutoQueueSpec",
    autoQueueCommand: "AutoQueueCommand",
    sysIdDrivePhase: "SysIdDrivePhase",
    sysIdDriveActive: "SysIdDriveActive",
    sysIdDriveLastCompleted: "SysIdDriveLastCompleted",
    sysIdDriveLastCompletedPhase: "SysIdDriveLastCompletedPhase",
    sysIdTurnPhase: "SysIdTurnPhase",
    sysIdTurnActive: "SysIdTurnActive",
    sysIdTurnLastCompleted: "SysIdTurnLastCompleted",
    sysIdTurnLastCompletedPhase: "SysIdTurnLastCompletedPhase",
    logRollStatus: "LogRollStatus",
    logRollLastTimestamp: "LogRollLastTimestamp",
    logRollCount: "LogRollCount",
    logCleanStatus: "LogCleanStatus",
    logCleanLastTimestamp: "LogCleanLastTimestamp",
    logCleanCount: "LogCleanCount",
    logCleanDeletedEntries: "LogCleanDeletedEntries",
    currentState: "CurrentState",
    requestAccepted: "RequestAccepted",
    requestReason: "RequestReason",
    targetType: "TargetType",
    targetPose: "TargetPose",
    targetPoseValid: "TargetPoseValid",
    robotPose: "RobotPose",
    autoQueueState: "AutoQueueState",
    autoQueuePreviewPose: "AutoQueuePreviewPose",
    autoQueuePreviewPoseValid: "AutoQueuePreviewPoseValid",
    selectedAutoState: "SelectedAutoState",
    systemCheckState: "SystemCheckState",
    autoCheckState: "AutoCheckState",
    autoQuickRunState: "AutoQuickRunState",
    ntDiagnosticsState: "NtDiagnosticsState",
    mechanismStatusState: "MechanismStatusState",
    actionTraceState: "ActionTraceState",
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
    visionPoseVisible: "VisionPoseVisible",
    shootEnabled: "ShootEnabled",
    intakeRollersHeld: "IntakeRollersHeld",
    intakeDeployed: "IntakeDeployed",
    teleopOverrideActive: "TeleopOverrideActive",
    driverControllerControlActive: "DriverControllerControlActive",
    shootReadyLatched: "ShootReadyLatched",
    intakeDeployRezeroInProgress: "IntakeDeployRezeroInProgress",
    manualIntakeDeployZeroSeekInProgress: "ManualIntakeDeployZeroSeekInProgress",
  },
};

const STATES = [
  "IDLING",
  "INTAKING",
  "SHOOTING",
  "SHOOT_INTAKE",
  "FERRYING",
  "TESTING",
];

const QUEUE_STATUSES = ["PENDING", "READY", "ACTIVE", "QUEUED"];
const SPOT_GROUPS = ["HUB", "TOWER", "DEPOT", "OUTPOST", "BUMP", "TRENCH", "LANE"];
const GROUP_COLORS = {
  HUB: "#ffd166",
  TOWER: "#90cdf4",
  DEPOT: "#39d98a",
  OUTPOST: "#f78fb3",
  BUMP: "#ff8c69",
  TRENCH: "#a29bfe",
  LANE: "#ffe66d",
};
const STATE_COLORS = {
  IDLING: "#e8eefc",
  INTAKING: "#39d98a",
  SHOOTING: "#ffd166",
  SHOOT_INTAKE: "#ff8c69",
  FERRYING: "#90cdf4",
  TESTING: "#f78fb3",
};

const layoutState = {
  referenceAlliance: "BLUE",
  spots: [],
  defaultZones: [],
  defaultPresets: [],
};

const state = {
  requestedState: null,
  currentState: null,
  requestAccepted: null,
  requestReason: null,
  targetType: null,
  targetPose: null,
  targetPoseValid: false,
  robotPose: null,
  autoQueueState: null,
  autoQueuePreviewPose: null,
  autoQueuePreviewPoseValid: false,
  selectedAutoState: null,
  systemCheckState: null,
  autoCheckState: null,
  autoQuickRunState: null,
  ntDiagnosticsState: null,
  mechanismStatusState: null,
  actionTraceState: null,
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
  sysIdDrivePhase: null,
  sysIdDriveActive: null,
  sysIdDriveLastCompleted: null,
  sysIdDriveLastCompletedPhase: null,
  sysIdTurnPhase: null,
  sysIdTurnActive: null,
  sysIdTurnLastCompleted: null,
  sysIdTurnLastCompletedPhase: null,
  logRollStatus: null,
  logRollLastTimestamp: null,
  logRollCount: null,
  logCleanStatus: null,
  logCleanLastTimestamp: null,
  logCleanCount: null,
  logCleanDeletedEntries: null,
  visionStatus: null,
  visionPoseVisible: null,
  shootEnabled: null,
  intakeRollersHeld: null,
  intakeDeployed: null,
  teleopOverrideActive: null,
  driverControllerControlActive: null,
  shootReadyLatched: null,
  intakeDeployRezeroInProgress: null,
  manualIntakeDeployZeroSeekInProgress: null,
  lastSystemCheckReport: null,
  lastAutoCheckReport: null,
  lastAutoQuickRunReport: null,
  subsystemDescriptions: null,
  storageInventory: null,
  latestDiagnosticBundle: null,
};

const queueModel = {
  revision: 0,
  steps: [],
  selectedAction: "",
  selectedStepIndex: -1,
  spotGroupFilter: "ALL",
  hoveredSpotId: null,
  presets: [],
  currentPresetId: null,
  startPose: null,
  customZones: [],
  editingZoneId: null,
  zoneCaptureAnchor: null,
  zoneCaptureMode: false,
  customSpots: [],
  editingCustomSpotId: null,
  customSpotCaptureMode: false,
  startPoseCaptureMode: false,
};

const ui = {
  tabButtons: [],
  tabPages: [],
  alertsBar: null,
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
  visionPoseVisible: null,
  visionStatus: null,
  turretStatus: null,
  sysIdDrive: null,
  sysIdTurn: null,
  sysIdDriveOptions: null,
  sysIdTurnOptions: null,
  robotPose: null,
  target: null,
  logRollStatus: null,
  logRollLast: null,
  logRollCount: null,
  logCleanStatus: null,
  logCleanLast: null,
  logCleanCount: null,
  logCleanDeleted: null,
  toast: null,
  stateButtons: [],
  autoStateButton: null,
  musicButton: null,
  musicStopButton: null,
  musicFile: null,
  musicUploadButton: null,
  musicUploadStatus: null,
  musicVolume: null,
  musicVolumeValue: null,
  debugTabMusicButton: null,
  debugTabMusicStopButton: null,
  debugTabMusicFile: null,
  debugTabMusicUploadButton: null,
  debugTabMusicUploadStatus: null,
  debugTabMusicVolume: null,
  debugTabMusicVolumeValue: null,
  rollLogsButton: null,
  cleanLogsButton: null,
  debugTabRollLogsButton: null,
  debugTabCleanLogsButton: null,
  fieldImage: null,
  fieldCanvas: null,
  driverShootIndicator: null,
  driverIntakeRollersIndicator: null,
  driverIntakeDeployIndicator: null,
  teleopOverrideIndicator: null,
  driverControlIndicator: null,
  intakeRezeroStatus: null,
  intakeRezeroButton: null,
  intakeRezeroCancelButton: null,
  intakeManualZeroSeekStatus: null,
  intakeManualZeroSeekButton: null,
  intakeManualZeroSeekCancelButton: null,
  debugTabIntakeManualZeroSeekButton: null,
  debugTabIntakeManualZeroSeekCancelButton: null,
  queueActionSelect: null,
  spotGroupFilter: null,
  queueStartButton: null,
  queueStopButton: null,
  queueSkipButton: null,
  queueClearButton: null,
  queueVisibleSpots: null,
  queueBuilderHint: null,
  queuePhase: null,
  queueRunning: null,
  queueActiveLabel: null,
  queueRevision: null,
  queueMessage: null,
  queueStatusMessage: null,
  queueSteps: null,
  queueEmptyState: null,
  queueDeselectStepButton: null,
  fieldAllianceView: null,
  fieldSyncMode: null,
  fieldPreviewPose: null,
  plannerAutosFile: null,
  plannerAutosImportButton: null,
  plannerAutosImportStatus: null,
  presetList: null,
  presetEmptyState: null,
  selectedAutoName: null,
  selectedAutoSummary: null,
  stageSelectedAutoButton: null,
  startPoseX: null,
  startPoseY: null,
  startPoseHeading: null,
  startPoseCaptureButton: null,
  startPoseSaveButton: null,
  startPoseClearButton: null,
  startPoseMode: null,
  startPoseSummary: null,
  zoneName: null,
  zoneXMin: null,
  zoneYMin: null,
  zoneXMax: null,
  zoneYMax: null,
  zoneCaptureButton: null,
  zoneSaveButton: null,
  zoneResetButton: null,
  zoneMode: null,
  zoneCount: null,
  zoneList: null,
  zoneEmptyState: null,
  customSpotName: null,
  customSpotGroup: null,
  customSpotX: null,
  customSpotY: null,
  customSpotHeading: null,
  customSpotSaveButton: null,
  customSpotResetButton: null,
  customSpotCaptureButton: null,
  customSpotMode: null,
  customSpotCount: null,
  customSpotList: null,
  customSpotEmptyState: null,
  runSystemChecksButton: null,
  runAutoChecksButton: null,
  runAutoQuickRunButton: null,
  checksOverall: null,
  checksLastRun: null,
  nextAutoSummary: null,
  mechanismsSummary: null,
  lastActionSummary: null,
  systemChecksList: null,
  autoChecksList: null,
  autoQuickRunList: null,
  mechanismStatusList: null,
  actionTraceList: null,
  systemsCatalog: null,
  systemsStorageInventory: null,
  systemsDiagnosticSummary: null,
  systemsDiagnosticFiles: null,
  systemsExportButton: null,
  systemsReloadButton: null,
};

let fieldCtx = null;
let fieldCanvasInitialized = false;
let lastAnyData = 0;
let ntConnected = false;
let sysIdDriveInitialized = false;
let sysIdTurnInitialized = false;
let logRollInitialized = false;
let logCleanInitialized = false;

const queryParams = new URLSearchParams(window.location.search);
const previewOverrides = resolvePreviewOverrides(queryParams);
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

window.addEventListener("DOMContentLoaded", async () => {
  cacheUi();
  updateMusicVolumeUi(Number(ui.musicVolume?.value || ui.debugTabMusicVolume?.value || 70));
  await loadLayoutSpec();
  queueModel.customZones = [];
  queueModel.presets = [];
  queueModel.customSpots = loadCustomSpots();
  setupTabs(queryParams.get("tab"));
  buildStateButtons();
  setupQueueBuilder();
  setupFieldCanvas();
  renderQueueSteps();
  renderPresets();
  renderStartPoseEditor();
  renderZoneEditor();
  renderZoneList();
  renderCustomSpots();
  renderCustomSpotEditor();
  renderQueueMeta();
  renderQueueStatus();
  await loadDeployAutoLibrary();
  await loadPersistedOperatorBoardData();
  startNetworkTables();
  window.addEventListener("resize", () => {
    setupFieldCanvas();
    renderField();
  });
  setInterval(() => {
    updateChips();
    render();
  }, 200);
});

function cacheUi() {
  ui.tabButtons = Array.from(document.querySelectorAll("[data-tab-target]"));
  ui.tabPages = Array.from(document.querySelectorAll("[data-tab-page]"));
  ui.alertsBar = document.getElementById("alerts-bar");
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
  ui.visionPoseVisible = document.getElementById("vision-pose-visible");
  ui.visionStatus = document.getElementById("vision-status");
  ui.turretStatus = document.getElementById("turret-status");
  ui.sysIdDrive = document.getElementById("sysid-drive");
  ui.sysIdTurn = document.getElementById("sysid-turn");
  ui.sysIdDriveOptions = document.getElementById("sysid-drive-options");
  ui.sysIdTurnOptions = document.getElementById("sysid-turn-options");
  ui.robotPose = document.getElementById("robot-pose");
  ui.target = document.getElementById("target");
  ui.logRollStatus = document.getElementById("log-roll-status");
  ui.logRollLast = document.getElementById("log-roll-last");
  ui.logRollCount = document.getElementById("log-roll-count");
  ui.logCleanStatus = document.getElementById("log-clean-status");
  ui.logCleanLast = document.getElementById("log-clean-last");
  ui.logCleanCount = document.getElementById("log-clean-count");
  ui.logCleanDeleted = document.getElementById("log-clean-deleted");
  ui.driverShootIndicator = document.getElementById("driver-shoot-indicator");
  ui.driverIntakeRollersIndicator = document.getElementById("driver-intake-rollers-indicator");
  ui.driverIntakeDeployIndicator = document.getElementById("driver-intake-deploy-indicator");
  ui.teleopOverrideIndicator = document.getElementById("teleop-override-indicator");
  ui.driverControlIndicator = document.getElementById("driver-control-indicator");
  ui.intakeRezeroStatus = document.getElementById("intake-rezero-status");
  ui.intakeManualZeroSeekStatus = document.getElementById("intake-manual-zero-seek-status");
  ui.toast = document.getElementById("toast");
  ui.fieldImage = document.getElementById("field-image");
  ui.fieldCanvas = document.getElementById("field-canvas");
  ui.autoStateButton = document.getElementById("auto-state-button");
  ui.musicButton = document.getElementById("music-button");
  ui.musicStopButton = document.getElementById("music-stop-button");
  ui.musicFile = document.getElementById("music-file");
  ui.musicUploadButton = document.getElementById("music-upload-button");
  ui.musicUploadStatus = document.getElementById("music-upload-status");
  ui.musicVolume = document.getElementById("music-volume");
  ui.musicVolumeValue = document.getElementById("music-volume-value");
  ui.debugTabMusicButton = document.getElementById("debug-tab-music-button");
  ui.debugTabMusicStopButton = document.getElementById("debug-tab-music-stop-button");
  ui.debugTabMusicFile = document.getElementById("debug-tab-music-file");
  ui.debugTabMusicUploadButton = document.getElementById("debug-tab-music-upload-button");
  ui.debugTabMusicUploadStatus = document.getElementById("debug-tab-music-upload-status");
  ui.debugTabMusicVolume = document.getElementById("debug-tab-music-volume");
  ui.debugTabMusicVolumeValue = document.getElementById("debug-tab-music-volume-value");
  ui.rollLogsButton = document.getElementById("roll-logs-button");
  ui.cleanLogsButton = document.getElementById("clean-logs-button");
  ui.debugTabRollLogsButton = document.getElementById("debug-tab-roll-logs-button");
  ui.debugTabCleanLogsButton = document.getElementById("debug-tab-clean-logs-button");
  ui.intakeRezeroButton = document.getElementById("intake-rezero-button");
  ui.intakeRezeroCancelButton = document.getElementById("intake-rezero-cancel-button");
  ui.intakeManualZeroSeekButton = document.getElementById("intake-manual-zero-seek-button");
  ui.intakeManualZeroSeekCancelButton = document.getElementById(
    "intake-manual-zero-seek-cancel-button"
  );
  ui.debugTabIntakeManualZeroSeekButton = document.getElementById(
    "debug-tab-intake-manual-zero-seek-button"
  );
  ui.debugTabIntakeManualZeroSeekCancelButton = document.getElementById(
    "debug-tab-intake-manual-zero-seek-cancel-button"
  );
  ui.queueActionSelect = document.getElementById("queue-action-select");
  ui.spotGroupFilter = document.getElementById("spot-group-filter");
  ui.queueStartButton = document.getElementById("queue-start-button");
  ui.queueStopButton = document.getElementById("queue-stop-button");
  ui.queueSkipButton = document.getElementById("queue-skip-button");
  ui.queueClearButton = document.getElementById("queue-clear-button");
  ui.queueVisibleSpots = document.getElementById("queue-visible-spots");
  ui.queueBuilderHint = document.getElementById("queue-builder-hint");
  ui.queuePhase = document.getElementById("queue-phase");
  ui.queueRunning = document.getElementById("queue-running");
  ui.queueActiveLabel = document.getElementById("queue-active-label");
  ui.queueRevision = document.getElementById("queue-revision");
  ui.queueMessage = document.getElementById("queue-message");
  ui.queueStatusMessage = document.getElementById("queue-status-message");
  ui.queueSteps = document.getElementById("queue-steps");
  ui.queueEmptyState = document.getElementById("queue-empty-state");
  ui.queueDeselectStepButton = document.getElementById("queue-deselect-step-button");
  ui.fieldAllianceView = document.getElementById("field-alliance-view");
  ui.fieldSyncMode = document.getElementById("field-sync-mode");
  ui.fieldPreviewPose = document.getElementById("field-preview-pose");
  ui.plannerAutosImportButton = document.getElementById("planner-autos-reload-button");
  ui.plannerAutosImportStatus = document.getElementById("planner-autos-import-status");
  ui.presetList = document.getElementById("preset-list");
  ui.presetEmptyState = document.getElementById("preset-empty-state");
  ui.selectedAutoName = document.getElementById("selected-auto-name");
  ui.selectedAutoSummary = document.getElementById("selected-auto-summary");
  ui.stageSelectedAutoButton = document.getElementById("select-selected-auto-button");
  ui.startPoseX = document.getElementById("start-pose-x");
  ui.startPoseY = document.getElementById("start-pose-y");
  ui.startPoseHeading = document.getElementById("start-pose-heading");
  ui.startPoseCaptureButton = document.getElementById("start-pose-capture-button");
  ui.startPoseSaveButton = document.getElementById("start-pose-save-button");
  ui.startPoseClearButton = document.getElementById("start-pose-clear-button");
  ui.startPoseMode = document.getElementById("start-pose-mode");
  ui.startPoseSummary = document.getElementById("start-pose-summary");
  ui.zoneName = document.getElementById("zone-name");
  ui.zoneXMin = document.getElementById("zone-x-min");
  ui.zoneYMin = document.getElementById("zone-y-min");
  ui.zoneXMax = document.getElementById("zone-x-max");
  ui.zoneYMax = document.getElementById("zone-y-max");
  ui.zoneCaptureButton = document.getElementById("zone-capture-button");
  ui.zoneSaveButton = document.getElementById("zone-save-button");
  ui.zoneResetButton = document.getElementById("zone-reset-button");
  ui.zoneMode = document.getElementById("zone-mode");
  ui.zoneCount = document.getElementById("zone-count");
  ui.zoneList = document.getElementById("zone-list");
  ui.zoneEmptyState = document.getElementById("zone-empty-state");
  ui.customSpotName = document.getElementById("custom-spot-name");
  ui.customSpotGroup = document.getElementById("custom-spot-group");
  ui.customSpotX = document.getElementById("custom-spot-x");
  ui.customSpotY = document.getElementById("custom-spot-y");
  ui.customSpotHeading = document.getElementById("custom-spot-heading");
  ui.customSpotSaveButton = document.getElementById("custom-spot-save-button");
  ui.customSpotResetButton = document.getElementById("custom-spot-reset-button");
  ui.customSpotCaptureButton = document.getElementById("custom-spot-capture-button");
  ui.customSpotMode = document.getElementById("custom-spot-mode");
  ui.customSpotCount = document.getElementById("custom-spot-count");
  ui.customSpotList = document.getElementById("custom-spot-list");
  ui.customSpotEmptyState = document.getElementById("custom-spot-empty-state");
  ui.runSystemChecksButton = document.getElementById("run-system-checks-button");
  ui.runAutoChecksButton = document.getElementById("run-auto-checks-button");
  ui.runAutoQuickRunButton = document.getElementById("run-auto-quick-run-button");
  ui.checksOverall = document.getElementById("checks-overall");
  ui.checksLastRun = document.getElementById("checks-last-run");
  ui.nextAutoSummary = document.getElementById("next-auto-summary");
  ui.mechanismsSummary = document.getElementById("mechanisms-summary");
  ui.lastActionSummary = document.getElementById("last-action-summary");
  ui.systemChecksList = document.getElementById("system-checks-list");
  ui.autoChecksList = document.getElementById("auto-checks-list");
  ui.autoQuickRunList = document.getElementById("auto-quick-run-list");
  ui.mechanismStatusList = document.getElementById("mechanism-status-list");
  ui.actionTraceList = document.getElementById("action-trace-list");
  ui.systemsCatalog = document.getElementById("systems-catalog");
  ui.systemsStorageInventory = document.getElementById("systems-storage-inventory");
  ui.systemsDiagnosticSummary = document.getElementById("systems-diagnostic-summary");
  ui.systemsDiagnosticFiles = document.getElementById("systems-diagnostic-files");
  ui.systemsExportButton = document.getElementById("systems-export-button");
  ui.systemsReloadButton = document.getElementById("systems-reload-button");

  if (ui.autoStateButton) {
    ui.autoStateButton.addEventListener("click", sendAutoStateEnable);
  }
  if (ui.musicButton) {
    ui.musicButton.addEventListener("click", sendPlaySwerveMusic);
  }
  if (ui.debugTabMusicButton) {
    ui.debugTabMusicButton.addEventListener("click", sendPlaySwerveMusic);
  }
  if (ui.musicStopButton) {
    ui.musicStopButton.addEventListener("click", sendStopSwerveMusic);
  }
  if (ui.debugTabMusicStopButton) {
    ui.debugTabMusicStopButton.addEventListener("click", sendStopSwerveMusic);
  }
  if (ui.musicUploadButton) {
    ui.musicUploadButton.addEventListener("click", () =>
      uploadMusicFileFrom(ui.musicFile, ui.musicUploadStatus)
    );
  }
  if (ui.debugTabMusicUploadButton) {
    ui.debugTabMusicUploadButton.addEventListener("click", () =>
      uploadMusicFileFrom(ui.debugTabMusicFile, ui.debugTabMusicUploadStatus)
    );
  }
  if (ui.musicVolume) {
    ui.musicVolume.addEventListener("input", () => {
      const value = Number(ui.musicVolume.value || 0);
      updateMusicVolumeUi(value);
      sendSwerveMusicVolume(value / 100.0);
    });
  }
  if (ui.debugTabMusicVolume) {
    ui.debugTabMusicVolume.addEventListener("input", () => {
      const value = Number(ui.debugTabMusicVolume.value || 0);
      updateMusicVolumeUi(value);
      sendSwerveMusicVolume(value / 100.0);
    });
  }
  if (ui.rollLogsButton) {
    ui.rollLogsButton.addEventListener("click", sendRollLogs);
  }
  if (ui.debugTabRollLogsButton) {
    ui.debugTabRollLogsButton.addEventListener("click", sendRollLogs);
  }
  if (ui.cleanLogsButton) {
    ui.cleanLogsButton.addEventListener("click", sendCleanLogs);
  }
  if (ui.debugTabCleanLogsButton) {
    ui.debugTabCleanLogsButton.addEventListener("click", sendCleanLogs);
  }
  if (ui.intakeRezeroButton) {
    ui.intakeRezeroButton.addEventListener("click", sendRequestIntakeDeployRezero);
  }
  if (ui.intakeRezeroCancelButton) {
    ui.intakeRezeroCancelButton.addEventListener("click", sendCancelIntakeDeployRezero);
  }
  if (ui.intakeManualZeroSeekButton) {
    ui.intakeManualZeroSeekButton.addEventListener("click", sendRequestManualIntakeDeployZeroSeek);
  }
  if (ui.debugTabIntakeManualZeroSeekButton) {
    ui.debugTabIntakeManualZeroSeekButton.addEventListener(
      "click",
      sendRequestManualIntakeDeployZeroSeek
    );
  }
  if (ui.intakeManualZeroSeekCancelButton) {
    ui.intakeManualZeroSeekCancelButton.addEventListener(
      "click",
      sendCancelManualIntakeDeployZeroSeek
    );
  }
  if (ui.debugTabIntakeManualZeroSeekCancelButton) {
    ui.debugTabIntakeManualZeroSeekCancelButton.addEventListener(
      "click",
      sendCancelManualIntakeDeployZeroSeek
    );
  }
  if (ui.startPoseCaptureButton) {
    ui.startPoseCaptureButton.addEventListener("click", () => {
      queueModel.startPoseCaptureMode = !queueModel.startPoseCaptureMode;
      queueModel.customSpotCaptureMode = false;
      queueModel.zoneCaptureMode = false;
      queueModel.zoneCaptureAnchor = null;
      renderStartPoseEditor();
      renderZoneEditor();
      renderCustomSpotEditor();
      renderField();
    });
  }
  if (ui.startPoseSaveButton) {
    ui.startPoseSaveButton.addEventListener("click", saveStartPose);
  }
  if (ui.startPoseClearButton) {
    ui.startPoseClearButton.addEventListener("click", clearStartPose);
  }
  if (ui.zoneCaptureButton) {
    ui.zoneCaptureButton.addEventListener("click", () => {
      queueModel.zoneCaptureMode = !queueModel.zoneCaptureMode;
      queueModel.zoneCaptureAnchor = null;
      queueModel.startPoseCaptureMode = false;
      queueModel.customSpotCaptureMode = false;
      renderZoneEditor();
      renderStartPoseEditor();
      renderCustomSpotEditor();
      renderField();
    });
  }
  if (ui.zoneSaveButton) {
    ui.zoneSaveButton.addEventListener("click", saveCustomZone);
  }
  if (ui.zoneResetButton) {
    ui.zoneResetButton.addEventListener("click", resetZoneEditor);
  }
}

function setupTabs(defaultTab) {
  activateTab(normalizeTabName(defaultTab));
  ui.tabButtons.forEach((button) => {
    button.addEventListener("click", () => {
      activateTab(normalizeTabName(button.dataset.tabTarget));
    });
  });
}

function activateTab(tabName) {
  const target = normalizeTabName(tabName);
  ui.tabButtons.forEach((button) => {
    const isActive = (button.dataset.tabTarget || "").toLowerCase() === target;
    button.classList.toggle("is-active", isActive);
  });
  ui.tabPages.forEach((page) => {
    const isActive = (page.dataset.tabPage || "").toLowerCase() === target;
    page.classList.toggle("is-active", isActive);
  });
  if (target === "auto") {
    window.requestAnimationFrame(() => {
      setupFieldCanvas();
      renderField();
    });
  }
  if (target === "systems") {
    renderSystemsCatalog();
    renderStorageInventory();
    renderDiagnosticBundleSummary();
  }
}

function normalizeTabName(tabName) {
  const normalized = String(tabName || "")
    .trim()
    .toLowerCase();
  if (normalized === "systems" || normalized === "system") {
    return "systems";
  }
  if (normalized === "auto") {
    return "auto";
  }
  if (normalized === "debug" || normalized === "debugging") {
    return "debug";
  }
  if (normalized === "game" || normalized === "teleop") {
    return "teleop";
  }
  return "teleop";
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

function setupQueueBuilder() {
  if (ui.queueActionSelect) {
    ui.queueActionSelect.value = queueModel.selectedAction;
    ui.queueActionSelect.addEventListener("change", () => {
      queueModel.selectedAction = normalizeRequestedState(ui.queueActionSelect.value);
      renderQueueMeta();
    });
  }
  if (ui.spotGroupFilter) {
    ui.spotGroupFilter.value = queueModel.spotGroupFilter;
    ui.spotGroupFilter.addEventListener("change", () => {
      queueModel.spotGroupFilter = parseString(ui.spotGroupFilter.value) || "ALL";
      renderQueueMeta();
      renderField();
    });
  }
  if (ui.queueStartButton) {
    ui.queueStartButton.addEventListener("click", () => sendQueueCommand("START"));
  }
  if (ui.queueStopButton) {
    ui.queueStopButton.addEventListener("click", () => sendQueueCommand("STOP"));
  }
  if (ui.queueSkipButton) {
    ui.queueSkipButton.addEventListener("click", () => sendQueueCommand("SKIP"));
  }
  if (ui.queueClearButton) {
    ui.queueClearButton.addEventListener("click", clearQueueSteps);
  }
  if (ui.queueDeselectStepButton) {
    ui.queueDeselectStepButton.addEventListener("click", () => {
      queueModel.selectedStepIndex = -1;
      renderQueueMeta();
      renderQueueSteps();
      renderField();
    });
  }
  if (ui.plannerAutosImportButton) {
    ui.plannerAutosImportButton.addEventListener("click", () => {
      void loadDeployAutoLibrary(true);
    });
  }
  if (ui.stageSelectedAutoButton) {
    ui.stageSelectedAutoButton.addEventListener("click", selectCurrentAutoForRobot);
  }
  if (ui.queueClearButton) {
    ui.queueClearButton.addEventListener("click", clearSelectedAutoOnRobot);
  }
  if (ui.customSpotSaveButton) {
    ui.customSpotSaveButton.addEventListener("click", saveCustomSpot);
  }
  if (ui.customSpotResetButton) {
    ui.customSpotResetButton.addEventListener("click", resetCustomSpotEditor);
  }
  if (ui.customSpotCaptureButton) {
    ui.customSpotCaptureButton.addEventListener("click", () => {
      queueModel.customSpotCaptureMode = !queueModel.customSpotCaptureMode;
      queueModel.startPoseCaptureMode = false;
      queueModel.zoneCaptureMode = false;
      queueModel.zoneCaptureAnchor = null;
      renderCustomSpotEditor();
      renderStartPoseEditor();
      renderZoneEditor();
      renderField();
    });
  }
  if (ui.runSystemChecksButton) {
    ui.runSystemChecksButton.addEventListener("click", runSystemChecks);
  }
  if (ui.runAutoChecksButton) {
    ui.runAutoChecksButton.addEventListener("click", runAutoChecks);
  }
  if (ui.runAutoQuickRunButton) {
    ui.runAutoQuickRunButton.addEventListener("click", runAutoQuickRun);
  }
  if (ui.systemsExportButton) {
    ui.systemsExportButton.addEventListener("click", () => {
      void exportDiagnosticBundle();
    });
  }
  if (ui.systemsReloadButton) {
    ui.systemsReloadButton.addEventListener("click", () => {
      void loadPersistedOperatorBoardData(true);
    });
  }
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

function sendStopSwerveMusic() {
  ntClient.addSample(contract.toRobot + contract.keys.stopSwerveMusic, true);
}

function sendSwerveMusicVolume(value) {
  if (!Number.isFinite(value)) return;
  ntClient.addSample(contract.toRobot + contract.keys.swerveMusicVolume, value);
}

function sendRollLogs() {
  ntClient.addSample(contract.toRobot + contract.keys.rollLogs, true);
}

function sendCleanLogs() {
  ntClient.addSample(contract.toRobot + contract.keys.cleanLogs, true);
}

function sendRequestIntakeDeployRezero() {
  ntClient.addSample(contract.toRobot + contract.keys.requestIntakeDeployRezero, true);
}

function sendCancelIntakeDeployRezero() {
  ntClient.addSample(contract.toRobot + contract.keys.cancelIntakeDeployRezero, true);
}

function sendRequestManualIntakeDeployZeroSeek() {
  ntClient.addSample(contract.toRobot + contract.keys.requestManualIntakeDeployZeroSeek, true);
}

function sendCancelManualIntakeDeployZeroSeek() {
  ntClient.addSample(contract.toRobot + contract.keys.cancelManualIntakeDeployZeroSeek, true);
}

function sendQueueCommand(command) {
  if (!command) return;
  ntClient.addSample(
    contract.toRobot + contract.keys.autoQueueCommand,
    JSON.stringify({ command: String(command).toUpperCase() })
  );
}

function publishQueueSpec() {
  queueModel.steps = queueModel.steps.map((step) => ({
    ...step,
    status: "PENDING",
  }));
  queueModel.revision += 1;
  const payload = {
    revision: queueModel.revision,
    startPose: serializePose(queueModel.startPose),
    noGoZones: getAllNoGoZones().map(serializeNoGoZone),
    steps: queueModel.steps.map((step, index) => serializeQueueStep(step, index)),
  };
  persistDraft();
  if (ntConnected) {
    ntClient.addSample(contract.toRobot + contract.keys.autoQueueSpec, JSON.stringify(payload));
  }
  renderQueueStatus();
  renderQueueMeta();
  renderQueueSteps();
  renderField();
}

async function uploadMusicFileFrom(fileInput, statusElement) {
  if (!fileInput || !fileInput.files || fileInput.files.length === 0) {
    setText(statusElement, "Select a .chrp file first");
    return;
  }
  const file = fileInput.files[0];
  setText(statusElement, "Uploading...");
  try {
    const response = await fetch("./music-upload", {
      method: "POST",
      headers: { "Content-Type": "application/octet-stream" },
      body: file,
    });
    if (!response.ok) {
      setText(statusElement, `Upload failed (${response.status})`);
      return;
    }
    setText(statusElement, "Upload complete");
  } catch (_err) {
    setText(statusElement, "Upload error");
  }
}

function updateMusicVolumeUi(value) {
  const safeValue = Number.isFinite(value) ? Math.max(0, Math.min(100, value)) : 0;
  if (ui.musicVolume) {
    ui.musicVolume.value = String(safeValue);
  }
  if (ui.debugTabMusicVolume) {
    ui.debugTabMusicVolume.value = String(safeValue);
  }
  setText(ui.musicVolumeValue, `${safeValue}%`);
  setText(ui.debugTabMusicVolumeValue, `${safeValue}%`);
}

async function loadPersistedOperatorBoardData(showToastOnSuccess = false) {
  const [subsystemDescriptions, storageInventory, latestDiagnosticBundle] = await Promise.all([
    fetchJsonDocument("./api/subsystem-descriptions"),
    fetchJsonDocument("./api/storage/inventory"),
    fetchJsonDocument("./api/diagnostics/latest"),
  ]);
  if (subsystemDescriptions) {
    state.subsystemDescriptions = subsystemDescriptions;
  }
  if (storageInventory) {
    state.storageInventory = storageInventory;
  }
  if (latestDiagnosticBundle) {
    state.latestDiagnosticBundle = latestDiagnosticBundle;
  }

  renderSystemsCatalog();
  renderStorageInventory();
  renderDiagnosticBundleSummary();

  if (showToastOnSuccess) {
    showToast("Persistent dashboard data refreshed");
  }
}

async function fetchJsonDocument(url, options) {
  try {
    const response = await fetch(url, {
      cache: "no-store",
      ...options,
    });
    if (!response.ok) {
      return null;
    }
    return await response.json();
  } catch (_err) {
    return null;
  }
}

function cloneJson(value) {
  return value == null ? value : JSON.parse(JSON.stringify(value));
}

function renderSystemsCatalog() {
  if (!ui.systemsCatalog) {
    return;
  }
  ui.systemsCatalog.innerHTML = "";
  const subsystems = state.subsystemDescriptions?.subsystems;
  if (!Array.isArray(subsystems) || subsystems.length === 0) {
    const empty = document.createElement("div");
    empty.className = "checks-list__empty";
    empty.textContent = "No subsystem descriptions available.";
    ui.systemsCatalog.append(empty);
    return;
  }

  subsystems.forEach((entry) => {
    const card = document.createElement("article");
    card.className = "system-card";

    const heading = document.createElement("div");
    heading.className = "system-card__title";
    heading.textContent = entry.name;

    const purpose = document.createElement("div");
    purpose.className = "system-card__purpose";
    purpose.textContent = entry.purpose;

    const meta = document.createElement("div");
    meta.className = "system-card__meta";
    meta.innerHTML = `<strong>Actuators:</strong> ${(entry.actuators || []).join(", ") || "--"}<br><strong>Sensors:</strong> ${(entry.sensors || []).join(", ") || "--"}<br><strong>Safety:</strong> ${(entry.safetyConditions || []).join(" ") || "--"}`;

    const states = document.createElement("div");
    states.className = "system-card__states";
    (entry.states || []).forEach((stateEntry) => {
      const item = document.createElement("div");
      item.className = "system-state";
      item.innerHTML = `<div class="system-state__title">${stateEntry.name}</div><div class="system-state__detail">${stateEntry.purpose}</div><div class="system-state__detail">${stateEntry.expectedMotion}</div><div class="system-state__detail"><strong>Ready:</strong> ${stateEntry.readyCondition}</div><div class="system-state__detail"><strong>Transitions:</strong> ${stateEntry.transitionConditions}</div><div class="system-state__detail"><strong>Blocks:</strong> ${stateEntry.faultBlockers}</div>`;
      states.append(item);
    });

    card.append(heading, purpose, meta, states);
    ui.systemsCatalog.append(card);
  });
}

function renderStorageInventory() {
  if (!ui.systemsStorageInventory) {
    return;
  }
  ui.systemsStorageInventory.innerHTML = "";
  const assets = state.storageInventory?.assets;
  if (!Array.isArray(assets) || assets.length === 0) {
    const empty = document.createElement("div");
    empty.className = "checks-list__empty";
    empty.textContent = "Storage inventory unavailable.";
    ui.systemsStorageInventory.append(empty);
    return;
  }
  assets.forEach((asset) => {
    const row = document.createElement("div");
    row.className = "check-row";
    row.innerHTML = `<div class="check-row__badge check-row__badge--pass">${asset.format.toUpperCase()}</div><div class="check-row__body"><div class="check-row__title">${asset.key}</div><div class="check-row__detail">${asset.description}</div><div class="check-row__detail">Authority: ${asset.authority} • Conflict: ${asset.conflictPolicy}</div><div class="check-row__detail">Local: ${asset.localPath}</div><div class="check-row__detail">Robot: ${asset.robotPath || "--"}</div></div>`;
    ui.systemsStorageInventory.append(row);
  });
}

function renderDiagnosticBundleSummary() {
  const manifest = state.latestDiagnosticBundle;
  setText(
    ui.systemsDiagnosticSummary,
    manifest ? `${manifest.bundleId} • ${manifest.overallStatus}` : "No diagnostic bundle yet"
  );
  if (ui.systemsDiagnosticFiles) {
    ui.systemsDiagnosticFiles.innerHTML = "";
    const files = Array.isArray(manifest?.files) ? manifest.files : [];
    if (!files.length) {
      const empty = document.createElement("div");
      empty.className = "checks-list__empty";
      empty.textContent = "Run an export to create a bundle.";
      ui.systemsDiagnosticFiles.append(empty);
    } else {
      files.forEach((file) => {
        const row = document.createElement("div");
        row.className = "checks-list__empty";
        row.textContent = file;
        ui.systemsDiagnosticFiles.append(row);
      });
    }
  }
}

async function exportDiagnosticBundle() {
  const manifest = await fetchJsonDocument("./api/diagnostics/export", {
    method: "POST",
  });
  if (!manifest) {
    showToast("Diagnostic export failed");
    return;
  }
  state.latestDiagnosticBundle = manifest;
  renderDiagnosticBundleSummary();
  showToast("Diagnostic bundle exported");
}

function startNetworkTables() {
  const dashboardTopics = [
    contract.keys.requestedState,
    contract.keys.currentState,
    contract.keys.requestAccepted,
    contract.keys.requestReason,
    contract.keys.targetType,
    contract.keys.targetPose,
    contract.keys.targetPoseValid,
    contract.keys.robotPose,
    contract.keys.autoQueueState,
    contract.keys.autoQueuePreviewPose,
    contract.keys.autoQueuePreviewPoseValid,
    contract.keys.selectedAutoState,
    contract.keys.systemCheckState,
    contract.keys.autoCheckState,
    contract.keys.autoQuickRunState,
    contract.keys.ntDiagnosticsState,
    contract.keys.mechanismStatusState,
    contract.keys.actionTraceState,
    contract.keys.hasBall,
    contract.keys.dsMode,
    contract.keys.batteryVoltage,
    contract.keys.brownout,
    contract.keys.alliance,
    contract.keys.matchTime,
    contract.keys.hubTimeframe,
    contract.keys.hubStatusValid,
    contract.keys.redHubStatus,
    contract.keys.blueHubStatus,
    contract.keys.ourHubStatus,
    contract.keys.ourHubActive,
    contract.keys.autoWinnerAlliance,
    contract.keys.gameDataRaw,
    contract.keys.hubRecommendation,
    contract.keys.turretAtSetpoint,
    contract.keys.turretMode,
    contract.keys.visionStatus,
    contract.keys.visionPoseVisible,
    contract.keys.shootEnabled,
    contract.keys.intakeRollersHeld,
    contract.keys.intakeDeployed,
    contract.keys.teleopOverrideActive,
    contract.keys.driverControllerControlActive,
    contract.keys.shootReadyLatched,
    contract.keys.intakeDeployRezeroInProgress,
    contract.keys.manualIntakeDeployZeroSeekInProgress,
    contract.keys.sysIdDrivePhase,
    contract.keys.sysIdDriveActive,
    contract.keys.sysIdDriveLastCompleted,
    contract.keys.sysIdDriveLastCompletedPhase,
    contract.keys.sysIdTurnPhase,
    contract.keys.sysIdTurnActive,
    contract.keys.sysIdTurnLastCompleted,
    contract.keys.sysIdTurnLastCompletedPhase,
    contract.keys.logRollStatus,
    contract.keys.logRollLastTimestamp,
    contract.keys.logRollCount,
    contract.keys.logCleanStatus,
    contract.keys.logCleanLastTimestamp,
    contract.keys.logCleanCount,
    contract.keys.logCleanDeletedEntries,
  ].map((key) => contract.toDashboard + key);

  ntClient.subscribe(dashboardTopics, false, false, 0.05);

  ntClient.publishTopic(contract.toRobot + contract.keys.requestedState, "string");
  ntClient.publishTopic(contract.toRobot + contract.keys.autoStateEnable, "boolean");
  ntClient.publishTopic(contract.toRobot + contract.keys.playSwerveMusic, "boolean");
  ntClient.publishTopic(contract.toRobot + contract.keys.stopSwerveMusic, "boolean");
  ntClient.publishTopic(contract.toRobot + contract.keys.swerveMusicVolume, "double");
  ntClient.publishTopic(contract.toRobot + contract.keys.rollLogs, "boolean");
  ntClient.publishTopic(contract.toRobot + contract.keys.cleanLogs, "boolean");
  ntClient.publishTopic(contract.toRobot + contract.keys.requestIntakeDeployRezero, "boolean");
  ntClient.publishTopic(contract.toRobot + contract.keys.cancelIntakeDeployRezero, "boolean");
  ntClient.publishTopic(
    contract.toRobot + contract.keys.requestManualIntakeDeployZeroSeek,
    "boolean"
  );
  ntClient.publishTopic(
    contract.toRobot + contract.keys.cancelManualIntakeDeployZeroSeek,
    "boolean"
  );
  ntClient.publishTopic(contract.toRobot + contract.keys.selectedAutoId, "string");
  ntClient.publishTopic(contract.toRobot + contract.keys.autoQueueSpec, "string");
  ntClient.publishTopic(contract.toRobot + contract.keys.autoQueueCommand, "string");
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
      state.requestAccepted = value === null ? null : !!value;
      break;
    case contract.toDashboard + contract.keys.requestReason:
      state.requestReason = parseString(value);
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
    case contract.toDashboard + contract.keys.autoQueueState: {
      const parsed = parseQueueState(value);
      if (parsed) {
        state.autoQueueState = parsed;
        syncQueueFromDashboard(parsed);
      }
      break;
    }
    case contract.toDashboard + contract.keys.autoQueuePreviewPose:
      state.autoQueuePreviewPose = parsePose(value);
      break;
    case contract.toDashboard + contract.keys.autoQueuePreviewPoseValid:
      state.autoQueuePreviewPoseValid = !!value;
      break;
    case contract.toDashboard + contract.keys.selectedAutoState:
      state.selectedAutoState = parseJsonObject(value);
      if (state.selectedAutoState?.id) {
        persistSelectedAutoId(state.selectedAutoState.id);
      }
      if (state.selectedAutoState?.id && state.selectedAutoState.id !== queueModel.currentPresetId) {
        if (queueModel.presets.some((entry) => entry.id === state.selectedAutoState.id)) {
          previewPlannerAuto(state.selectedAutoState.id, false);
        }
      } else if (!state.selectedAutoState?.id) {
        clearStoredSelectedAutoId();
      }
      break;
    case contract.toDashboard + contract.keys.systemCheckState:
      state.systemCheckState = parseJsonObject(value);
      break;
    case contract.toDashboard + contract.keys.autoCheckState:
      state.autoCheckState = parseJsonObject(value);
      break;
    case contract.toDashboard + contract.keys.autoQuickRunState:
      state.autoQuickRunState = parseJsonObject(value);
      break;
    case contract.toDashboard + contract.keys.ntDiagnosticsState:
      state.ntDiagnosticsState = parseJsonObject(value);
      break;
    case contract.toDashboard + contract.keys.mechanismStatusState:
      state.mechanismStatusState = parseJsonObject(value);
      break;
    case contract.toDashboard + contract.keys.actionTraceState:
      state.actionTraceState = parseJsonObject(value);
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
      state.alliance = normalizeAlliance(value);
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
    case contract.toDashboard + contract.keys.sysIdDrivePhase:
      state.sysIdDrivePhase = parseString(value);
      break;
    case contract.toDashboard + contract.keys.sysIdDriveActive:
      state.sysIdDriveActive = !!value;
      break;
    case contract.toDashboard + contract.keys.sysIdDriveLastCompleted: {
      const parsed = Number.isFinite(value) ? value : null;
      if (parsed !== null) {
        if (sysIdDriveInitialized && state.sysIdDriveLastCompleted !== parsed) {
          showToast("Drive SysId complete");
        }
        sysIdDriveInitialized = true;
      }
      state.sysIdDriveLastCompleted = parsed;
      break;
    }
    case contract.toDashboard + contract.keys.sysIdDriveLastCompletedPhase:
      state.sysIdDriveLastCompletedPhase = parseString(value);
      break;
    case contract.toDashboard + contract.keys.sysIdTurnPhase:
      state.sysIdTurnPhase = parseString(value);
      break;
    case contract.toDashboard + contract.keys.sysIdTurnActive:
      state.sysIdTurnActive = !!value;
      break;
    case contract.toDashboard + contract.keys.sysIdTurnLastCompleted: {
      const parsed = Number.isFinite(value) ? value : null;
      if (parsed !== null) {
        if (sysIdTurnInitialized && state.sysIdTurnLastCompleted !== parsed) {
          showToast("Turn SysId complete");
        }
        sysIdTurnInitialized = true;
      }
      state.sysIdTurnLastCompleted = parsed;
      break;
    }
    case contract.toDashboard + contract.keys.sysIdTurnLastCompletedPhase:
      state.sysIdTurnLastCompletedPhase = parseString(value);
      break;
    case contract.toDashboard + contract.keys.logRollStatus:
      state.logRollStatus = parseString(value);
      break;
    case contract.toDashboard + contract.keys.logRollLastTimestamp:
      state.logRollLastTimestamp = Number.isFinite(value) ? value : null;
      break;
    case contract.toDashboard + contract.keys.logRollCount: {
      const parsed = Number.isFinite(value) ? value : null;
      if (parsed !== null) {
        if (logRollInitialized && state.logRollCount !== parsed) {
          showToast("Logs rolled");
        }
        logRollInitialized = true;
      }
      state.logRollCount = parsed;
      break;
    }
    case contract.toDashboard + contract.keys.logCleanStatus:
      state.logCleanStatus = parseString(value);
      break;
    case contract.toDashboard + contract.keys.logCleanLastTimestamp:
      state.logCleanLastTimestamp = Number.isFinite(value) ? value : null;
      break;
    case contract.toDashboard + contract.keys.logCleanCount: {
      const parsed = Number.isFinite(value) ? value : null;
      if (parsed !== null) {
        if (logCleanInitialized && state.logCleanCount !== parsed) {
          showToast("Logs folder cleaned");
        }
        logCleanInitialized = true;
      }
      state.logCleanCount = parsed;
      break;
    }
    case contract.toDashboard + contract.keys.logCleanDeletedEntries:
      state.logCleanDeletedEntries = Number.isFinite(value) ? value : null;
      break;
    case contract.toDashboard + contract.keys.visionStatus:
      state.visionStatus = parseString(value);
      break;
    case contract.toDashboard + contract.keys.visionPoseVisible:
      state.visionPoseVisible = !!value;
      break;
    case contract.toDashboard + contract.keys.shootEnabled:
      state.shootEnabled = !!value;
      break;
    case contract.toDashboard + contract.keys.intakeRollersHeld:
      state.intakeRollersHeld = !!value;
      break;
    case contract.toDashboard + contract.keys.intakeDeployed:
      state.intakeDeployed = !!value;
      break;
    case contract.toDashboard + contract.keys.teleopOverrideActive:
      state.teleopOverrideActive = !!value;
      break;
    case contract.toDashboard + contract.keys.driverControllerControlActive:
      state.driverControllerControlActive = !!value;
      break;
    case contract.toDashboard + contract.keys.shootReadyLatched:
      state.shootReadyLatched = !!value;
      break;
    case contract.toDashboard + contract.keys.intakeDeployRezeroInProgress:
      state.intakeDeployRezeroInProgress = !!value;
      break;
    case contract.toDashboard + contract.keys.manualIntakeDeployZeroSeekInProgress:
      state.manualIntakeDeployZeroSeekInProgress = !!value;
      break;
    default:
      break;
  }
}

function render() {
  const effectiveDsMode = previewOverrides.dsMode ?? state.dsMode;
  const effectiveMatchTime = previewOverrides.matchTime ?? state.matchTime;
  const effectiveVisionPoseVisible =
    previewOverrides.visionPoseVisible ?? state.visionPoseVisible;

  setText(ui.requestedState, state.requestedState || "--");
  setText(ui.currentState, state.currentState || "--");

  if (state.requestAccepted === null) {
    setText(ui.requestStatus, "--");
  } else {
    setText(ui.requestStatus, state.requestAccepted ? "ACCEPTED" : "REJECTED");
  }
  setText(ui.requestReason, state.requestReason || "--");
  applyDriverIndicator(ui.driverShootIndicator, state.shootEnabled);
  applyDriverIndicator(ui.driverIntakeRollersIndicator, state.intakeRollersHeld);
  applyDriverIndicator(ui.driverIntakeDeployIndicator, state.intakeDeployed);
  applyDriverIndicator(ui.teleopOverrideIndicator, state.teleopOverrideActive);
  applyDriverIndicator(ui.driverControlIndicator, state.driverControllerControlActive);

  setText(
    ui.intakeRezeroStatus,
    state.intakeDeployRezeroInProgress === null
      ? "--"
      : state.intakeDeployRezeroInProgress
        ? "IN PROGRESS"
        : "IDLE"
  );
  if (ui.intakeRezeroStatus) {
    ui.intakeRezeroStatus.classList.remove("kv__v--ok", "kv__v--bad", "kv__v--warn");
    ui.intakeRezeroStatus.classList.add(
      state.intakeDeployRezeroInProgress ? "kv__v--warn" : "kv__v--ok"
    );
  }

  setText(
    ui.intakeManualZeroSeekStatus,
    state.manualIntakeDeployZeroSeekInProgress === null
      ? "--"
      : state.manualIntakeDeployZeroSeekInProgress
        ? "RUNNING"
        : "IDLE"
  );
  if (ui.intakeManualZeroSeekStatus) {
    ui.intakeManualZeroSeekStatus.classList.remove("kv__v--ok", "kv__v--bad", "kv__v--warn");
    ui.intakeManualZeroSeekStatus.classList.add(
      state.manualIntakeDeployZeroSeekInProgress ? "kv__v--warn" : "kv__v--ok"
    );
  }

  setText(ui.alliance, state.alliance || "--");
  setText(ui.matchTime, formatMatchTime(effectiveMatchTime));
  document.body.classList.toggle(
    "body--match-alert",
    effectiveDsMode === "TELEOP" &&
      Number.isFinite(effectiveMatchTime) &&
      effectiveMatchTime <= 30.0 &&
      effectiveMatchTime > 0.0
  );
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

  if (ui.visionPoseVisible) {
    if (effectiveVisionPoseVisible === null) {
      setText(ui.visionPoseVisible, "--");
      ui.visionPoseVisible.classList.remove("is-on", "is-off");
    } else {
      const hasPose = !!effectiveVisionPoseVisible;
      setText(ui.visionPoseVisible, hasPose ? "TRUE" : "FALSE");
      ui.visionPoseVisible.classList.toggle("is-on", hasPose);
      ui.visionPoseVisible.classList.toggle("is-off", !hasPose);
    }
  }

  setText(ui.visionStatus, state.visionStatus || "--");

  const turret = state.turretMode ? state.turretMode : "UNAVAILABLE";
  const turretExtra =
    state.turretAtSetpoint === null
      ? ""
      : state.turretAtSetpoint
        ? " (AT GOAL)"
        : " (MOVING)";
  setText(ui.turretStatus, turret + turretExtra);

  const driveSysIdText = formatSysId(
    state.sysIdDrivePhase,
    state.sysIdDriveActive,
    state.sysIdDriveLastCompleted,
    state.sysIdDriveLastCompletedPhase
  );
  setText(ui.sysIdDrive, driveSysIdText);
  applySysIdClass(ui.sysIdDrive, state.sysIdDrivePhase, state.sysIdDriveActive);
  setText(
    ui.sysIdDriveOptions,
    formatSysIdOptions(
      state.sysIdDrivePhase,
      state.sysIdDriveActive,
      state.sysIdDriveLastCompletedPhase
    )
  );

  const turnSysIdText = formatSysId(
    state.sysIdTurnPhase,
    state.sysIdTurnActive,
    state.sysIdTurnLastCompleted,
    state.sysIdTurnLastCompletedPhase
  );
  setText(ui.sysIdTurn, turnSysIdText);
  applySysIdClass(ui.sysIdTurn, state.sysIdTurnPhase, state.sysIdTurnActive);
  setText(
    ui.sysIdTurnOptions,
    formatSysIdOptions(
      state.sysIdTurnPhase,
      state.sysIdTurnActive,
      state.sysIdTurnLastCompletedPhase
    )
  );

  setText(ui.robotPose, formatPose(state.robotPose, true));
  setText(ui.target, formatTarget(state.targetType, state.targetPose, state.targetPoseValid));
  setText(ui.logRollStatus, state.logRollStatus || "--");
  setText(ui.logRollLast, formatTimestampSeconds(state.logRollLastTimestamp));
  setText(
    ui.logRollCount,
    Number.isFinite(state.logRollCount) ? String(Math.trunc(state.logRollCount)) : "--"
  );
  applyLogRollClass(ui.logRollStatus, state.logRollStatus);
  setText(ui.logCleanStatus, state.logCleanStatus || "--");
  setText(ui.logCleanLast, formatTimestampSeconds(state.logCleanLastTimestamp));
  setText(
    ui.logCleanCount,
    Number.isFinite(state.logCleanCount) ? String(Math.trunc(state.logCleanCount)) : "--"
  );
  setText(
    ui.logCleanDeleted,
    Number.isFinite(state.logCleanDeletedEntries)
      ? String(Math.trunc(state.logCleanDeletedEntries))
      : "--"
  );
  applyLogRollClass(ui.logCleanStatus, state.logCleanStatus);

  ui.stateButtons.forEach((button) => {
    const name = button.dataset.state;
    button.classList.toggle("is-requested", name && name === state.requestedState);
    button.classList.toggle("is-current", name && name === state.currentState);
  });

  renderAlerts();
  renderReadiness();
  renderQueueStatus();
  renderQueueMeta();
  renderField();
  renderDiagnosticBundleSummary();
}

function runSystemChecks() {
  state.lastSystemCheckReport =
    normalizeBackendCheckReport(state.systemCheckState) || evaluateChecks("system");
  renderReadiness();
  showToast("System checks updated");
}

function runAutoChecks() {
  state.lastAutoCheckReport =
    normalizeBackendCheckReport(state.autoCheckState) || evaluateChecks("auto");
  renderReadiness();
  showToast("Auto checks updated");
}

function runAutoQuickRun() {
  sendQueueCommand("QUICK_RUN");
  state.lastAutoQuickRunReport =
    normalizeBackendCheckReport(state.autoQuickRunState) || state.lastAutoQuickRunReport;
  renderReadiness();
  showToast("Auto quick run requested");
}

function renderAlerts() {
  if (!ui.alertsBar) return;
  const alerts = computeAlerts();
  ui.alertsBar.innerHTML = "";
  if (!alerts.length) {
    const empty = document.createElement("div");
    empty.className = "alert-pill alert-pill--muted";
    empty.textContent = "No active alerts";
    ui.alertsBar.append(empty);
    return;
  }
  alerts.forEach((alert) => {
    const pill = document.createElement("div");
    pill.className = `alert-pill alert-pill--${alert.severity}`;
    pill.textContent = alert.message;
    ui.alertsBar.append(pill);
  });
}

function renderReadiness() {
  const queueState = getEffectiveQueueState();
  const systemReport =
    normalizeBackendCheckReport(state.systemCheckState) || state.lastSystemCheckReport;
  const autoReport = normalizeBackendCheckReport(state.autoCheckState) || state.lastAutoCheckReport;
  const quickRunReport =
    normalizeBackendCheckReport(state.autoQuickRunState) || state.lastAutoQuickRunReport;
  const mechanismReport = normalizeMechanismStatusReport(state.mechanismStatusState);
  const actionTraceReport = normalizeActionTraceReport(state.actionTraceState);

  renderCheckList(ui.systemChecksList, systemReport, "Run checks to snapshot robot readiness.");
  renderCheckList(ui.autoChecksList, autoReport, "Run auto checks to validate the staged queue.");
  renderCheckList(
    ui.autoQuickRunList,
    quickRunReport,
    "Run quick validation to build every staged auto step without executing it."
  );
  renderCheckList(
    ui.mechanismStatusList,
    mechanismReport,
    "Waiting for mechanism telemetry."
  );
  renderCheckList(
    ui.actionTraceList,
    actionTraceReport,
    "Waiting for dashboard action traces."
  );

  const headlineReport = quickRunReport || autoReport || systemReport;
  setText(ui.checksOverall, headlineReport ? headlineReport.summary : "NOT RUN");
  setText(ui.checksLastRun, headlineReport ? formatCheckTimestamp(headlineReport.timestamp) : "--");
  applyCheckHeadlineClass(ui.checksOverall, headlineReport ? headlineReport.status : "idle");
  setText(ui.nextAutoSummary, formatNextAutoSummary(queueState));
  setText(ui.mechanismsSummary, mechanismReport ? mechanismReport.summary : "--");
  applyCheckHeadlineClass(
    ui.mechanismsSummary,
    mechanismReport ? mechanismReport.status : "idle"
  );
  setText(ui.lastActionSummary, formatActionTraceSummary(state.actionTraceState));
  applyCheckHeadlineClass(
    ui.lastActionSummary,
    actionTraceReport ? actionTraceReport.status : "idle"
  );
}

function renderCheckList(container, report, emptyMessage) {
  if (!container) return;
  container.innerHTML = "";
  if (!report || !Array.isArray(report.items) || report.items.length === 0) {
    const empty = document.createElement("div");
    empty.className = "checks-list__empty";
    empty.textContent = emptyMessage;
    container.append(empty);
    return;
  }

  report.items.forEach((item) => {
    const row = document.createElement("div");
    row.className = "check-row";

    const badge = document.createElement("div");
    badge.className = `check-row__badge check-row__badge--${item.status}`;
    badge.textContent = item.status.toUpperCase();

    const body = document.createElement("div");
    body.className = "check-row__body";

    const title = document.createElement("div");
    title.className = "check-row__title";
    title.textContent = item.label;

    const detail = document.createElement("div");
    detail.className = "check-row__detail";
    detail.textContent = item.detail;

    body.append(title, detail);
    row.append(badge, body);
    container.append(row);
  });
}

function evaluateChecks(mode) {
  const queueState = getEffectiveQueueState();
  const stale = Date.now() - lastAnyData > STALE_MS;
  const items = [];

  items.push(checkItem("NetworkTables link", ntConnected && !stale ? "pass" : ntConnected ? "warn" : "fail", ntConnected ? (stale ? "Connected but dashboard data is stale." : "Connected and receiving data.") : "Disconnected from NT4."));

  const battery = Number.isFinite(state.batteryVoltage) ? state.batteryVoltage : null;
  items.push(
    checkItem(
      "Battery",
      battery === null ? "warn" : battery >= 12.0 ? "pass" : battery >= 11.2 ? "warn" : "fail",
      battery === null ? "Battery voltage unavailable." : `${battery.toFixed(2)}V`
    )
  );

  items.push(
    checkItem(
      "Brownout state",
      state.brownout ? "fail" : "pass",
      state.brownout ? "Robot is browned out." : "No brownout reported."
    )
  );

  items.push(
    checkItem(
      "Vision pose updates",
      state.visionStatus === "DISABLED"
        ? "fail"
        : state.visionPoseVisible
          ? "pass"
          : "warn",
      state.visionStatus === "DISABLED"
        ? "Vision subsystem disabled."
        : state.visionPoseVisible
          ? "At least one camera currently has a pose."
          : "No accepted pose visible right now."
    )
  );

  items.push(
    checkItem(
      "Turret readiness",
      state.turretMode === "UNAVAILABLE"
        ? "fail"
        : state.turretAtSetpoint
          ? "pass"
          : "warn",
      state.turretMode === "UNAVAILABLE"
        ? "Turret subsystem unavailable."
        : `${state.turretMode || "UNKNOWN"}${state.turretAtSetpoint ? " and at setpoint." : " still moving."}`
    )
  );

  items.push(
    checkItem(
      "Mechanism zeroing",
      state.intakeDeployRezeroInProgress || state.manualIntakeDeployZeroSeekInProgress ? "warn" : "pass",
      state.intakeDeployRezeroInProgress
        ? "Intake deploy rezero is in progress."
        : state.manualIntakeDeployZeroSeekInProgress
          ? "Manual intake zero seek is running."
          : "No intake zeroing process active."
    )
  );

  if (mode === "system") {
    items.push(
      checkItem(
        "Request channel",
        state.requestAccepted === false ? "fail" : state.requestAccepted === true ? "pass" : "warn",
        state.requestAccepted === false
          ? state.requestReason || "Last dashboard request was rejected."
          : state.requestAccepted === true
            ? "Last dashboard request was accepted."
            : "No dashboard request status has been observed yet."
      )
    );
    items.push(
      checkItem(
        "Shoot gate",
        state.shootEnabled ? "pass" : "warn",
        state.shootEnabled ? "Shoot gate currently enabled." : "Shoot gate currently disabled."
      )
    );
  }

  if (mode === "auto") {
    const hasSteps = Array.isArray(queueState.steps) && queueState.steps.length > 0;
    items.push(
      checkItem(
        "Queue loaded",
        hasSteps ? "pass" : "fail",
        hasSteps ? `${queueState.steps.length} queued step(s) available.` : "No queued steps staged."
      )
    );
    items.push(
      checkItem(
        "Queue phase",
        queueState.phase === "ERROR"
          ? "fail"
          : queueState.phase === "READY" || queueState.phase === "RUNNING" || queueState.phase === "COMPLETE"
            ? "pass"
            : "warn",
        `Phase: ${queueState.phase || "UNKNOWN"}`
      )
    );
    items.push(
      checkItem(
        "Preview pose",
        state.autoQueuePreviewPoseValid ? "pass" : hasSteps ? "warn" : "fail",
        state.autoQueuePreviewPoseValid
          ? formatPose(state.autoQueuePreviewPose, true)
          : hasSteps
            ? "Queue has steps but no preview pose is available."
            : "Queue is empty."
      )
    );
    items.push(
      checkItem(
        "Autonomous start state",
        state.dsMode === "AUTO" || state.dsMode === "DISABLED" ? "pass" : "warn",
        state.dsMode ? `DS mode is ${state.dsMode}.` : "Driver Station mode unavailable."
      )
    );
  }

  const failCount = items.filter((item) => item.status === "fail").length;
  const warnCount = items.filter((item) => item.status === "warn").length;
  const status = failCount > 0 ? "fail" : warnCount > 0 ? "warn" : "pass";
  const summary =
    status === "fail"
      ? `${failCount} fail, ${warnCount} warn`
      : status === "warn"
        ? `${warnCount} warning${warnCount === 1 ? "" : "s"}`
        : "READY";
  return { timestamp: Date.now(), mode, status, summary, items };
}

function computeAlerts() {
  const queueState = getEffectiveQueueState();
  const alerts = [];
  const stale = Date.now() - lastAnyData > STALE_MS;
  const mechanismReport = normalizeMechanismStatusReport(state.mechanismStatusState);
  const actionTraceReport = normalizeActionTraceReport(state.actionTraceState);
  const quickRunReport = normalizeBackendCheckReport(state.autoQuickRunState);

  if (!ntConnected) {
    alerts.push({ severity: "bad", message: "NT offline" });
  } else if (stale) {
    alerts.push({ severity: "warn", message: "dashboard data stale" });
  }
  if (state.brownout) {
    alerts.push({ severity: "bad", message: "robot browned out" });
  }
  if (Number.isFinite(state.batteryVoltage) && state.batteryVoltage < 11.2) {
    alerts.push({ severity: state.batteryVoltage < 10.5 ? "bad" : "warn", message: `battery ${state.batteryVoltage.toFixed(1)}V` });
  }
  if (state.requestAccepted === false && state.requestReason) {
    alerts.push({ severity: "bad", message: `request rejected: ${state.requestReason}` });
  }
  if (state.intakeDeployRezeroInProgress) {
    alerts.push({ severity: "warn", message: "intake rezero running" });
  }
  if (state.manualIntakeDeployZeroSeekInProgress) {
    alerts.push({ severity: "warn", message: "manual zero seek running" });
  }
  if (queueState.phase === "ERROR") {
    alerts.push({ severity: "bad", message: "auto selection error" });
  } else if (!queueState.steps || queueState.steps.length === 0) {
    alerts.push({ severity: "warn", message: "no auto selected" });
  }
  if (state.visionStatus === "DISABLED") {
    alerts.push({ severity: "warn", message: "vision disabled" });
  }
  if (state.ntDiagnosticsState && Number.isFinite(state.ntDiagnosticsState.sampledPublishBytesPerSec)) {
    const sampledBytesPerSec = state.ntDiagnosticsState.sampledPublishBytesPerSec;
    if (sampledBytesPerSec >= 24 * 1024) {
      alerts.push({ severity: "bad", message: `nt out ${(sampledBytesPerSec / 1024).toFixed(1)} kB/s` });
    } else if (sampledBytesPerSec >= 8 * 1024) {
      alerts.push({ severity: "warn", message: `nt out ${(sampledBytesPerSec / 1024).toFixed(1)} kB/s` });
    }
  }
  if (state.ntDiagnosticsState && Number.isFinite(state.ntDiagnosticsState.canBusUtilization)) {
    const canUtilization = state.ntDiagnosticsState.canBusUtilization;
    if (canUtilization >= 0.9) {
      alerts.push({ severity: "bad", message: `can ${(canUtilization * 100).toFixed(0)}%` });
    } else if (canUtilization >= 0.7) {
      alerts.push({ severity: "warn", message: `can ${(canUtilization * 100).toFixed(0)}%` });
    }
    if (
      (Number.isFinite(state.ntDiagnosticsState.canTxFullCount) &&
        state.ntDiagnosticsState.canTxFullCount > 0) ||
      (Number.isFinite(state.ntDiagnosticsState.canReceiveErrorCount) &&
        state.ntDiagnosticsState.canReceiveErrorCount > 0) ||
      (Number.isFinite(state.ntDiagnosticsState.canTransmitErrorCount) &&
        state.ntDiagnosticsState.canTransmitErrorCount > 0)
    ) {
      alerts.push({ severity: "bad", message: "can errors active" });
    }
  }
  if (mechanismReport && mechanismReport.status === "fail") {
    alerts.push({ severity: "bad", message: `mechanisms: ${mechanismReport.summary}` });
  } else if (mechanismReport && mechanismReport.status === "warn") {
    alerts.push({ severity: "warn", message: `mechanisms: ${mechanismReport.summary}` });
  }
  if (actionTraceReport && actionTraceReport.status === "fail") {
    alerts.push({ severity: "bad", message: "last dashboard action failed" });
  }
  if (quickRunReport && quickRunReport.status === "fail") {
    alerts.push({ severity: "bad", message: `auto quick run: ${quickRunReport.summary}` });
  } else if (quickRunReport && quickRunReport.status === "warn") {
    alerts.push({ severity: "warn", message: `auto quick run: ${quickRunReport.summary}` });
  }
  return alerts.slice(0, 6);
}

function checkItem(label, status, detail) {
  return { label, status, detail };
}

function parseJsonObject(value) {
  if (typeof value !== "string" || !value.trim()) {
    return null;
  }
  try {
    const parsed = JSON.parse(value);
    return parsed && typeof parsed === "object" ? parsed : null;
  } catch {
    return null;
  }
}

function normalizeBackendCheckReport(report) {
  if (!report || !Array.isArray(report.items)) {
    return null;
  }
  return {
    timestamp: Number.isFinite(report.timestampSec) ? report.timestampSec * 1000 : Date.now(),
    mode: parseString(report.mode) || "",
    status: parseString(report.status) || "warn",
    summary: parseString(report.summary) || "UNKNOWN",
    previewPose: parseString(report.previewPose) || "",
    items: report.items.map((item) => ({
      label: parseString(item.label) || "--",
      status: parseString(item.status) || "warn",
      detail: parseString(item.detail) || "--",
    })),
  };
}

function normalizeMechanismStatusReport(report) {
  if (!report || !Array.isArray(report.items)) {
    return null;
  }
  const items = report.items.map((item) => {
    const health = parseString(item.health) || "UNKNOWN";
    const connected = item.connected === true;
    const atGoal = item.atGoal === true;
    return {
      label: parseString(item.label) || "--",
      status: healthToCheckStatus(health, connected),
      detail: [
        `${health}${connected ? "" : " • disconnected"}`,
        parseString(item.controlMode) || "UNKNOWN",
        atGoal ? "at goal" : "tracking",
        parseString(item.detail) || "",
      ]
        .filter(Boolean)
        .join(" • "),
    };
  });
  return {
    timestamp: Number.isFinite(report.timestampSec) ? report.timestampSec * 1000 : Date.now(),
    mode: "mechanisms",
    status: summarizeCheckStatus(items.map((item) => item.status)),
    summary: parseString(report.summary) || "UNKNOWN",
    items,
  };
}

function normalizeActionTraceReport(trace) {
  if (!trace || typeof trace !== "object") {
    return null;
  }
  const status = normalizeTraceStatus(parseString(trace.status));
  return {
    timestamp: Number.isFinite(trace.timestampSec) ? trace.timestampSec * 1000 : Date.now(),
    mode: "action",
    status,
    summary: formatActionTraceHeadline(trace),
    items: [
      {
        label: `${parseString(trace.source) || "UNKNOWN"} • ${parseString(trace.action) || "UNKNOWN"}`,
        status,
        detail: [
          parseString(trace.detail) || "No detail available.",
          `Requested ${parseString(trace.requestedState) || "UNKNOWN"}`,
          `Current ${parseString(trace.currentState) || "UNKNOWN"}`,
        ].join(" • "),
      },
    ],
  };
}

function healthToCheckStatus(health, connected) {
  if (!connected || health === "OFFLINE" || health === "FAULTED") {
    return "fail";
  }
  if (health === "DEGRADED") {
    return "warn";
  }
  return "pass";
}

function normalizeTraceStatus(status) {
  if (status === "fail" || status === "warn" || status === "pass") {
    return status;
  }
  return "warn";
}

function summarizeCheckStatus(statuses) {
  if (statuses.some((status) => status === "fail")) {
    return "fail";
  }
  if (statuses.some((status) => status === "warn")) {
    return "warn";
  }
  return "pass";
}

function formatActionTraceHeadline(trace) {
  if (!trace || typeof trace !== "object") {
    return "--";
  }
  const source = parseString(trace.source) || "UNKNOWN";
  const action = parseString(trace.action) || "UNKNOWN";
  const status = normalizeTraceStatus(parseString(trace.status)).toUpperCase();
  return `${source} ${action} ${status}`;
}

function formatActionTraceSummary(trace) {
  if (!trace || typeof trace !== "object") {
    return "--";
  }
  const headline = formatActionTraceHeadline(trace);
  const detail = parseString(trace.detail) || "No detail available.";
  const timestamp = Number.isFinite(trace.timestampSec)
    ? formatCheckTimestamp(trace.timestampSec * 1000)
    : "--";
  return `${headline}\n${detail}\n${timestamp}`;
}

function applyCheckHeadlineClass(element, status) {
  if (!element) return;
  element.classList.remove("tile__v--ok", "tile__v--bad", "tile__v--warn");
  if (status === "pass") {
    element.classList.add("tile__v--ok");
  } else if (status === "fail") {
    element.classList.add("tile__v--bad");
  } else if (status === "warn") {
    element.classList.add("tile__v--warn");
  }
}

function formatCheckTimestamp(timestampMs) {
  if (!Number.isFinite(timestampMs)) return "--";
  const date = new Date(timestampMs);
  return date.toLocaleTimeString([], { hour: "2-digit", minute: "2-digit", second: "2-digit" });
}

function formatNextAutoSummary(queueState) {
  const ntDetail = formatNtDiagnosticsSummary(state.ntDiagnosticsState);
  if (!queueState || !Array.isArray(queueState.steps) || queueState.steps.length === 0) {
    return ntDetail ? `No queue staged.\n${ntDetail}` : "No queue staged.";
  }
  const first = queueState.steps[0];
  const labels = queueState.steps.slice(0, 3).map((step) => step.label || step.spotId || "Step");
  const suffix = queueState.steps.length > 3 ? `\n+${queueState.steps.length - 3} more step(s)` : "";
  const preview = state.autoQueuePreviewPoseValid
    ? `\nStart: ${formatPose(state.autoQueuePreviewPose, true)}`
    : "";
  const ntSummary = ntDetail ? `\n${ntDetail}` : "";
  return `Phase: ${queueState.phase}\nSteps: ${queueState.steps.length}\nFirst: ${first.label || first.spotId || "--"}\nSequence: ${labels.join(" -> ")}${suffix}${preview}${ntSummary}`;
}

function formatNtDiagnosticsSummary(report) {
  if (!report || typeof report !== "object") {
    return "";
  }
  const connections = Number.isFinite(report.connectionCount) ? report.connectionCount : null;
  const topicCount = Number.isFinite(report.topicCount) ? report.topicCount : null;
  const queueBytes = Number.isFinite(report.queueStateBytes) ? report.queueStateBytes : null;
  const publishPeriod = Number.isFinite(report.publishPeriodSec) ? report.publishPeriodSec : null;
  const canBusUtilization = Number.isFinite(report.canBusUtilization)
    ? report.canBusUtilization
    : null;
  const canTxFullCount = Number.isFinite(report.canTxFullCount) ? report.canTxFullCount : null;
  const canReceiveErrorCount = Number.isFinite(report.canReceiveErrorCount)
    ? report.canReceiveErrorCount
    : null;
  const canTransmitErrorCount = Number.isFinite(report.canTransmitErrorCount)
    ? report.canTransmitErrorCount
    : null;
  const sampledPublishRate = Number.isFinite(report.sampledPublishRateHz)
    ? report.sampledPublishRateHz
    : null;
  const sampledBytesPerSec = Number.isFinite(report.sampledPublishBytesPerSec)
    ? report.sampledPublishBytesPerSec
    : null;
  const hotTopics = Array.isArray(report.hotTopics) ? report.hotTopics : [];
  const parts = [];
  if (connections !== null) {
    parts.push(`NT clients ${connections}`);
  }
  if (topicCount !== null) {
    parts.push(`topics ${topicCount}`);
  }
  if (queueBytes !== null) {
    parts.push(`queue ${queueBytes} B`);
  }
  if (publishPeriod !== null) {
    parts.push(`pub ${(publishPeriod * 1000).toFixed(0)} ms`);
  }
  if (canBusUtilization !== null) {
    parts.push(`CAN ${(canBusUtilization * 100).toFixed(0)}%`);
  }
  if (sampledPublishRate !== null) {
    parts.push(`out ${sampledPublishRate.toFixed(1)} Hz`);
  }
  if (sampledBytesPerSec !== null) {
    parts.push(`out ${(sampledBytesPerSec / 1024).toFixed(1)} kB/s`);
  }
  if (hotTopics.length > 0 && hotTopics[0] && hotTopics[0].topic) {
    const topic = String(hotTopics[0].topic).split("/").pop();
    parts.push(`hot ${topic}`);
  }
  if (
    (canTxFullCount !== null && canTxFullCount > 0) ||
    (canReceiveErrorCount !== null && canReceiveErrorCount > 0) ||
    (canTransmitErrorCount !== null && canTransmitErrorCount > 0)
  ) {
    parts.push(
      `CAN errs ${canTxFullCount || 0}/${canReceiveErrorCount || 0}/${canTransmitErrorCount || 0}`
    );
  }
  return parts.length ? `NT: ${parts.join(" • ")}` : "";
}

function renderQueueStatus() {
  const queueState = getEffectiveQueueState();
  const selectedAuto = state.selectedAutoState;
  setText(ui.queuePhase, queueState.phase);
  setText(ui.queueRunning, queueState.running ? "YES" : "NO");
  setText(ui.queueActiveLabel, queueState.activeLabel || selectedAuto?.name || "--");
  setText(ui.queueRevision, Number.isFinite(queueState.revision) ? String(queueState.revision) : "--");
  setText(ui.queueMessage, selectedAuto?.message || queueState.message || "--");
  setText(
    ui.queueStatusMessage,
    selectedAuto?.message ||
      queueState.message ||
      "Select a deployed PathPlanner auto to preview it here."
  );
  setText(ui.fieldPreviewPose, formatAuthoringPose(getQueueStartPose()));
  setText(ui.startPoseSummary, formatAuthoringPose(queueModel.startPose));
  setText(
    ui.fieldSyncMode,
    queueModel.presets.length > 0
      ? ntConnected
        ? "Deploy Library + Robot Sync"
        : "Deploy Library Loaded"
      : "Waiting for Deploy Autos"
  );
}

function renderQueueMeta() {
  const visibleSpots = getVisibleSpots();
  setText(ui.queueVisibleSpots, String(visibleSpots.length));

  if (queueModel.selectedStepIndex >= 0) {
    const step = queueModel.steps[queueModel.selectedStepIndex];
    const name = step ? describeStep(step) : "step";
    setText(ui.queueBuilderHint, `Replace ${name}`);
  } else if (queueModel.selectedAction) {
    setText(ui.queueBuilderHint, `Append with ${queueModel.selectedAction}`);
  } else {
    setText(ui.queueBuilderHint, "Append new step");
  }

  setText(ui.fieldAllianceView, "Blue Reference");
  if (ui.queueActionSelect) {
    ui.queueActionSelect.value = queueModel.selectedAction;
  }
  if (ui.spotGroupFilter) {
    ui.spotGroupFilter.value = queueModel.spotGroupFilter;
  }
}

function renderQueueSteps() {
  if (!ui.queueSteps) return;
  ui.queueSteps.innerHTML = "";
  if (ui.queueEmptyState) {
    ui.queueEmptyState.style.display = queueModel.steps.length === 0 ? "block" : "none";
  }

  queueModel.steps.forEach((step, index) => {
    const card = document.createElement("div");
    card.className = "queue-step";

    const top = document.createElement("div");
    top.className = "queue-step__top";

    const badge = document.createElement("div");
    badge.className = "queue-step__badge";
    badge.innerText = String(index + 1);

    const titleWrap = document.createElement("div");
    const title = document.createElement("div");
    title.className = "queue-step__title";
    title.innerText = describeStep(step);
    const meta = document.createElement("div");
    meta.className = "queue-step__meta";
    meta.innerText = `${step.group || describeSpotGroup(step.spotId)} • ${getStepActionLabel(step) || "No action"}`;
    titleWrap.append(title, meta);

    const status = document.createElement("div");
    status.className = `queue-step__status queue-step__status--${step.status.toLowerCase()}`;
    status.innerText = step.status;

    top.append(badge, titleWrap, status);

    const detail = document.createElement("div");
    detail.className = "queue-step__meta";
    const pose = resolveStepPose(step);
    const routeWaypoints = getStepRouteWaypoints(index, step);
    const routeDetail =
      step.type === "NAMED_COMMAND"
        ? `command ${step.commandName || step.label || "hook"}`
        : step.type === "WAIT"
          ? `wait ${Number.isFinite(step.waitSeconds) ? step.waitSeconds.toFixed(2) : "0.00"}s`
          : routeWaypoints.length > 0
            ? `${routeWaypoints.length} waypoint${routeWaypoints.length === 1 ? "" : "s"}`
            : "direct route";
    detail.innerText = [
      pose ? formatPose(pose, true) : "pose unavailable",
      routeDetail,
    ].join(" • ");

    card.append(top, detail);
    ui.queueSteps.appendChild(card);
  });
}

function renderPresets() {
  if (!ui.presetList) return;
  ui.presetList.innerHTML = "";
  if (ui.presetEmptyState) {
    ui.presetEmptyState.style.display = queueModel.presets.length === 0 ? "block" : "none";
  }

  queueModel.presets.forEach((preset) => {
    const item = document.createElement("div");
    item.className = "preset";
    item.classList.toggle("is-active", preset.id === queueModel.currentPresetId);
    item.classList.toggle("is-selected-for-robot", state.selectedAutoState?.id === preset.id);

    const title = document.createElement("div");
    title.className = "preset__title";
    title.innerText = preset.name;

    const meta = document.createElement("div");
    meta.className = "preset__meta";
    const metaParts = [`${preset.steps.length} steps`, formatPresetTimestamp(preset.updatedAt)];
    if (preset.folder) {
      metaParts.unshift(preset.folder);
    }
    if (state.selectedAutoState?.id === preset.id) {
      metaParts.unshift("CHOSEN");
    } else if (preset.id === queueModel.currentPresetId) {
      metaParts.unshift("PREVIEW");
    }
    meta.innerText = metaParts.join(" • ");

    const swatches = document.createElement("div");
    swatches.className = "preset__swatches";
    uniquePresetStates(preset).forEach((stateName) => {
      const swatch = document.createElement("div");
      swatch.className = "preset__swatch";
      swatch.style.background = getStateColor(stateName);
      swatch.title = stateName || "No Action";
      swatches.appendChild(swatch);
    });

    const actions = document.createElement("div");
    actions.className = "preset__actions";
    actions.append(
      buildPresetButton("Preview", () => previewPlannerAuto(preset.id, true)),
      buildPresetButton("Select", () => selectPlannerAuto(preset.id))
    );

    item.append(title, meta, buildPresetPreview(preset), swatches, actions);
    ui.presetList.appendChild(item);
  });
  renderSelectedAutoSummary();
}

async function loadDeployAutoLibrary(showToastOnComplete = false) {
  setText(ui.plannerAutosImportStatus, "Loading deploy autos...");
  try {
    const response = await fetch(`/planner-autos/index.json?ts=${Date.now()}`, {
      cache: "no-store",
    });
    if (!response.ok) {
      setText(ui.plannerAutosImportStatus, `Deploy autos unavailable (${response.status}).`);
      return;
    }
    const manifest = await response.json();
    const entries = Array.isArray(manifest?.autos) ? manifest.autos : [];
    const autos = (
      await Promise.all(
        entries.map(async (entry) => {
          const relativePath = parseString(entry?.relativePath);
          if (!relativePath) {
            return null;
          }
          const autoResponse = await fetch(`/planner-autos/${relativePath}?ts=${Date.now()}`, {
            cache: "no-store",
          });
          if (!autoResponse.ok) {
            return null;
          }
          const rawAuto = await autoResponse.json();
          return normalizePresetObject({
            ...rawAuto,
            id: parseString(rawAuto?.id) || parseString(entry?.id),
            name: parseString(rawAuto?.name) || parseString(entry?.name),
            folder: parseString(rawAuto?.folder) || parseString(entry?.folder),
            relativePath,
            updatedAt:
              Number.isFinite(rawAuto?.updatedAt) && rawAuto.updatedAt > 0
                ? rawAuto.updatedAt
                : Number.isFinite(entry?.updatedAt)
                  ? entry.updatedAt
                  : Date.now(),
          });
        })
      )
    ).filter(Boolean);
    queueModel.presets = autos;
    setText(
      ui.plannerAutosImportStatus,
      autos.length > 0
        ? `Loaded ${autos.length} deployed auto${autos.length === 1 ? "" : "s"}.`
        : "No deployed autos were found."
    );
    const preferredId =
      parseString(state.selectedAutoState?.id) ||
      loadStoredSelectedAutoId() ||
      queueModel.currentPresetId;
    if (preferredId && queueModel.presets.some((entry) => entry.id === preferredId)) {
      previewPlannerAuto(preferredId, false);
    } else if (autos.length > 0) {
      previewPlannerAuto(autos[0].id, false);
    } else {
      queueModel.currentPresetId = null;
      queueModel.steps = [];
      queueModel.startPose = null;
      queueModel.customZones = [];
    }
    renderPresets();
    renderQueueStatus();
    renderQueueMeta();
    renderQueueSteps();
    renderField();
    if (showToastOnComplete) {
      showToast(
        autos.length > 0
          ? `Loaded ${autos.length} deployed auto${autos.length === 1 ? "" : "s"}`
          : "No deployed autos found"
      );
    }
  } catch (_err) {
    setText(ui.plannerAutosImportStatus, "Failed to load deployed autos.");
  }
}

function previewPlannerAuto(presetId, showToastOnPreview) {
  const preset = queueModel.presets.find((entry) => entry.id === presetId);
  if (!preset) return;
  queueModel.currentPresetId = preset.id;
  queueModel.steps = cloneSteps(preset.steps);
  queueModel.startPose = clonePose(preset.startPose);
  queueModel.customZones = cloneZones(preset.customZones);
  queueModel.selectedStepIndex = -1;
  renderZoneList();
  renderStartPoseEditor();
  renderPresets();
  renderQueueStatus();
  renderQueueMeta();
  renderQueueSteps();
  renderField();
  if (showToastOnPreview) {
    showToast(`Previewing ${preset.name}`);
  }
}

function selectPlannerAuto(presetId) {
  previewPlannerAuto(presetId, false);
  selectCurrentAutoForRobot();
}

function selectCurrentAutoForRobot() {
  if (!queueModel.currentPresetId) {
    showToast("Select an auto first");
    return;
  }
  persistSelectedAutoId(queueModel.currentPresetId);
  ntClient.addSample(contract.toRobot + contract.keys.selectedAutoId, queueModel.currentPresetId);
  showToast(`Selected ${queueModel.presets.find((entry) => entry.id === queueModel.currentPresetId)?.name || "auto"}`);
}

function clearSelectedAutoOnRobot() {
  clearStoredSelectedAutoId();
  ntClient.addSample(contract.toRobot + contract.keys.selectedAutoId, "");
  showToast("Cleared robot auto selection");
}

function renderSelectedAutoSummary() {
  const preset = queueModel.presets.find((entry) => entry.id === queueModel.currentPresetId) || null;
  const chosenAuto =
    queueModel.presets.find((entry) => entry.id === state.selectedAutoState?.id) || null;
  setText(ui.selectedAutoName, chosenAuto ? chosenAuto.name : preset ? `${preset.name} (Preview Only)` : "--");
  if (!preset && !chosenAuto) {
    setText(ui.selectedAutoSummary, "--");
    return;
  }
  const summaryTarget = chosenAuto || preset;
  const states = Array.from(
    new Set((summaryTarget.steps || []).map((step) => getStepActionLabel(step)).filter(Boolean))
  );
  const parts = [`${summaryTarget.steps.length} steps`];
  if (summaryTarget.folder) {
    parts.unshift(summaryTarget.folder);
  }
  if (summaryTarget.startPose) {
    parts.push(formatAuthoringPose(summaryTarget.startPose));
  }
  if (states.length > 0) {
    parts.push(states.join(", "));
  }
  if (chosenAuto) {
    parts.push(state.selectedAutoState?.loaded ? "Robot selected" : "Robot rejected");
  } else if (preset) {
    parts.push("Preview only");
  }
  setText(ui.selectedAutoSummary, parts.join(" • "));
}

function setupFieldCanvas() {
  if (!ui.fieldCanvas || !ui.fieldImage) return;
  fieldCtx = ui.fieldCanvas.getContext("2d");
  resizeCanvasToDisplaySize(ui.fieldCanvas);

  if (!fieldCanvasInitialized) {
    ui.fieldCanvas.addEventListener("click", handleFieldCanvasClick);
    ui.fieldCanvas.addEventListener("mousemove", handleFieldCanvasMove);
    ui.fieldCanvas.addEventListener("mouseleave", () => {
      queueModel.hoveredSpotId = null;
      if (ui.fieldCanvas) {
        ui.fieldCanvas.style.cursor = "default";
      }
      renderField();
    });
    fieldCanvasInitialized = true;
  }

  if (!ui.fieldImage.complete) {
    ui.fieldImage.addEventListener(
      "load",
      () => {
        resizeCanvasToDisplaySize(ui.fieldCanvas);
        renderField();
      },
      { once: true }
    );
  }
}

function handleFieldCanvasClick(event) {
  if (!ui.fieldCanvas) return;
  void event;
}

function handleFieldCanvasMove(event) {
  if (!ui.fieldCanvas) return;
  void event;
  queueModel.hoveredSpotId = null;
  ui.fieldCanvas.style.cursor = "default";
}

function renderField() {
  if (!fieldCtx || !ui.fieldCanvas) return;

  const { width, height } = getCanvasSize(ui.fieldCanvas);
  fieldCtx.clearRect(0, 0, width, height);
  const fieldRect = getFieldRenderRect(width, height);
  const visibleSpots = getVisibleSpots();
  const robotPose = toBlueReferencePose(state.robotPose);
  const targetPose = state.targetPoseValid ? toBlueReferencePose(state.targetPose) : null;
  const previewPose = getQueueStartPose();

  fieldCtx.save();
  fieldCtx.beginPath();
  fieldCtx.rect(fieldRect.x, fieldRect.y, fieldRect.width, fieldRect.height);
  fieldCtx.clip();

  drawNoGoZones(fieldRect);
  drawQueuePath(fieldRect);

  if (robotPose) {
    drawRobotBox(robotPose, fieldRect, {
      fill: "rgba(57,217,138,0.26)",
      stroke: "rgba(57,217,138,0.98)",
      label: "Robot",
      labelColor: "#e8eefc",
      labelBackground: "rgba(57,217,138,0.22)",
    });
  }

  if (targetPose) {
    drawRobotBox(targetPose, fieldRect, {
      fill: "rgba(255,255,255,0.12)",
      stroke: "rgba(255,255,255,0.92)",
      label: state.targetType || "Target",
      labelColor: "#f8fbff",
      labelBackground: "rgba(255,255,255,0.18)",
    });
  }

  if (previewPose) {
    drawRobotBox(previewPose, fieldRect, {
      fill: "rgba(255,209,102,0.16)",
      stroke: "rgba(255,209,102,0.9)",
      dashed: true,
      label: "Start",
      labelColor: "#ffe8ad",
      labelBackground: "rgba(255,209,102,0.22)",
    });
  }

  visibleSpots.forEach((spot) => {
    const canvasPose = fieldToCanvas(spot, fieldRect);
    const queuedIndex = findQueueIndexForSpot(spot.id);
    const emphasized =
      queuedIndex >= 0 || queueModel.hoveredSpotId === spot.id || isSelectedSpot(spot.id);
    drawSpotMarker(canvasPose.x, canvasPose.y, spot, queuedIndex, emphasized);
  });

  if (queueModel.zoneCaptureMode && queueModel.zoneCaptureAnchor) {
    drawPendingZoneAnchor(fieldToCanvas(queueModel.zoneCaptureAnchor, fieldRect));
  }

  fieldCtx.restore();
}

function drawQueuePath(fieldRect) {
  const previewSegments = buildPreviewSegments();
  if (previewSegments.length === 0) {
    return;
  }
  previewSegments.forEach((segment) => {
    const pathPoints = segment.poses.map((pose) => fieldToCanvas(pose, fieldRect));
    if (pathPoints.length >= 2) {
      drawSmoothPath(pathPoints, segment.color, 5);
      drawDashedPath(pathPoints, "rgba(255,255,255,0.18)", 1.5);
    }
    segment.waypoints.forEach((pose, index) => {
      drawQueueWaypoint(pose, fieldRect, `${segment.stepIndex + 1}.${index + 1}`, segment.color, true);
    });
    drawQueueWaypoint(segment.finalPose, fieldRect, String(segment.stepIndex + 1), segment.color, false);
  });
}

function drawNoGoZones(fieldRect) {
  getAllNoGoZones().forEach((zone) => {
    const topLeft = fieldToCanvas({ x: zone.xMinMeters, y: zone.yMaxMeters }, fieldRect);
    const bottomRight = fieldToCanvas({ x: zone.xMaxMeters, y: zone.yMinMeters }, fieldRect);
    const width = bottomRight.x - topLeft.x;
    const height = bottomRight.y - topLeft.y;
    fieldCtx.save();
    fieldCtx.fillStyle = zone.locked ? "rgba(255, 99, 132, 0.18)" : "rgba(255, 184, 77, 0.14)";
    fieldCtx.strokeStyle = zone.locked ? "rgba(255, 99, 132, 0.78)" : "rgba(255, 184, 77, 0.82)";
    fieldCtx.lineWidth = zone.locked ? 2 : 2.5;
    fieldCtx.fillRect(topLeft.x, topLeft.y, width, height);
    fieldCtx.strokeRect(topLeft.x, topLeft.y, width, height);
    fieldCtx.restore();
    drawLabel(
      topLeft.x + 8,
      topLeft.y + 20,
      zone.locked ? `${zone.label} • LOCKED` : zone.label,
      "rgba(8,11,17,0.78)",
      "#fce7d6",
      zone.locked ? "#ff6384" : "#ffb84d"
    );
  });
}

function drawPendingZoneAnchor(point) {
  drawRing(point.x, point.y, 12, "rgba(255,184,77,0.9)");
  drawLabel(point.x + 14, point.y - 14, "Zone Corner 1", "rgba(8,11,17,0.78)", "#ffe7c2", "#ffb84d");
}

function buildPreviewSegments() {
  const segments = [];
  const startPose = getQueueStartPose();
  let previousPose = startPose;

  queueModel.steps.forEach((step, index) => {
    const currentStep = normalizeQueueStep(step);
    if (currentStep.type && currentStep.type !== "PATH") {
      return;
    }
    const finalPose = resolveStepPose(currentStep);
    if (!finalPose || !previousPose) {
      return;
    }
    const routeWaypoints = getStepRouteWaypoints(index, currentStep);
    const poses = [previousPose, ...routeWaypoints.map(authoringPoseToRuntimePose), finalPose];
    segments.push({
      stepIndex: index,
      requestedState: currentStep.requestedState,
      color: getStateColor(currentStep.requestedState),
      waypoints: routeWaypoints,
      finalPose,
      poses: dedupePath(poses),
    });
    previousPose = finalPose;
  });

  return segments;
}

function drawSmoothPath(points, color, width) {
  fieldCtx.save();
  fieldCtx.beginPath();
  fieldCtx.moveTo(points[0].x, points[0].y);

  if (points.length === 2) {
    fieldCtx.lineTo(points[1].x, points[1].y);
  } else {
    for (let i = 1; i < points.length - 1; i += 1) {
      const midX = (points[i].x + points[i + 1].x) / 2;
      const midY = (points[i].y + points[i + 1].y) / 2;
      fieldCtx.quadraticCurveTo(points[i].x, points[i].y, midX, midY);
    }
    const secondLast = points[points.length - 2];
    const last = points[points.length - 1];
    fieldCtx.quadraticCurveTo(secondLast.x, secondLast.y, last.x, last.y);
  }

  fieldCtx.strokeStyle = color;
  fieldCtx.lineWidth = width;
  fieldCtx.lineCap = "round";
  fieldCtx.lineJoin = "round";
  fieldCtx.shadowColor = "rgba(57,217,138,0.25)";
  fieldCtx.shadowBlur = 18;
  fieldCtx.stroke();
  fieldCtx.restore();
}

function drawDashedPath(points, color, width) {
  fieldCtx.save();
  fieldCtx.beginPath();
  fieldCtx.moveTo(points[0].x, points[0].y);
  points.slice(1).forEach((point) => fieldCtx.lineTo(point.x, point.y));
  fieldCtx.setLineDash([10, 10]);
  fieldCtx.strokeStyle = color;
  fieldCtx.lineWidth = width;
  fieldCtx.lineCap = "round";
  fieldCtx.stroke();
  fieldCtx.restore();
}

function drawQueueWaypoint(pose, fieldRect, indexLabel, color, isRoutePoint) {
  drawRobotBox(pose, fieldRect, {
    fill: isRoutePoint ? "rgba(8,11,17,0.55)" : "rgba(8,11,17,0.72)",
    stroke: color,
    dashed: isRoutePoint,
    scale: isRoutePoint ? 0.55 : 0.72,
    badgeText: indexLabel,
    badgeColor: color,
  });
}

function drawSpotMarker(x, y, spot, queuedIndex, emphasized) {
  const groupColor = GROUP_COLORS[spot.group] || "#ffffff";
  const fillColor = "rgba(80, 140, 255, 0.42)";
  const radius = emphasized ? 10 : 7;

  fieldCtx.save();
  fieldCtx.beginPath();
  fieldCtx.arc(x, y, radius, 0, Math.PI * 2);
  fieldCtx.fillStyle = fillColor;
  fieldCtx.fill();
  fieldCtx.lineWidth = emphasized ? 3 : 2;
  fieldCtx.strokeStyle = groupColor;
  fieldCtx.stroke();
  fieldCtx.restore();

  if (queuedIndex >= 0) {
    fieldCtx.save();
    fieldCtx.beginPath();
    fieldCtx.arc(x + 10, y - 10, 9, 0, Math.PI * 2);
    fieldCtx.fillStyle = "rgba(255,209,102,0.95)";
    fieldCtx.fill();
    fieldCtx.fillStyle = "#241800";
    fieldCtx.font = `${Math.max(10, Math.round(11 * (window.devicePixelRatio || 1)))}px sans-serif`;
    fieldCtx.textAlign = "center";
    fieldCtx.textBaseline = "middle";
    fieldCtx.fillText(String(queuedIndex + 1), x + 10, y - 9);
    fieldCtx.restore();
  }

  const label = getFieldSpotLabel(spot, false);
  const labelBg = emphasized ? "rgba(14,17,22,0.85)" : "rgba(14,17,22,0.68)";
  drawLabel(x + 14, y - 14, label, labelBg, "#e8eefc", groupColor);
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
  return { width: canvas.width, height: canvas.height };
}

function getFieldRenderRect(canvasWidth, canvasHeight) {
  const imageAspect =
    ui.fieldImage && ui.fieldImage.naturalWidth > 0 && ui.fieldImage.naturalHeight > 0
      ? ui.fieldImage.naturalWidth / ui.fieldImage.naturalHeight
      : FIELD_LENGTH_METERS / FIELD_WIDTH_METERS;
  const canvasAspect = canvasWidth / canvasHeight;

  if (canvasAspect > imageAspect) {
    const height = canvasHeight;
    const width = height * imageAspect;
    return {
      x: (canvasWidth - width) / 2,
      y: 0,
      width,
      height,
    };
  }

  const width = canvasWidth;
  const height = width / imageAspect;
  return {
    x: 0,
    y: (canvasHeight - height) / 2,
    width,
    height,
  };
}

function fieldToCanvas(pose, fieldRect) {
  const x = fieldRect.x + clamp(pose.x / FIELD_LENGTH_METERS, 0, 1) * fieldRect.width;
  const y = fieldRect.y + (1 - clamp(pose.y / FIELD_WIDTH_METERS, 0, 1)) * fieldRect.height;
  return { x, y };
}

function canvasToField(x, y, fieldRect) {
  return {
    x: clamp((x - fieldRect.x) / fieldRect.width, 0, 1) * FIELD_LENGTH_METERS,
    y: (1 - clamp((y - fieldRect.y) / fieldRect.height, 0, 1)) * FIELD_WIDTH_METERS,
    theta: 0,
  };
}

function getCanvasPointerPosition(event, canvas) {
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width;
  const scaleY = canvas.height / rect.height;
  return {
    x: (event.clientX - rect.left) * scaleX,
    y: (event.clientY - rect.top) * scaleY,
  };
}

function findNearestVisibleSpot(x, y, fieldRect, maxDistance) {
  let best = null;
  let bestDistance = Number.POSITIVE_INFINITY;

  getVisibleSpots().forEach((spot) => {
    const point = fieldToCanvas(spot, fieldRect);
    const distance = Math.hypot(point.x - x, point.y - y);
    if (distance <= maxDistance && distance < bestDistance) {
      best = spot;
      bestDistance = distance;
    }
  });

  return best;
}

function addOrReplaceQueueStep(spot) {
  if (!spot) return;
  const nextStep = stepFromSpot(spot, queueModel.selectedAction);

  if (queueModel.selectedStepIndex >= 0 && queueModel.selectedStepIndex < queueModel.steps.length) {
    queueModel.steps[queueModel.selectedStepIndex] = nextStep;
    showToast(`Updated step ${queueModel.selectedStepIndex + 1}`);
    queueModel.selectedStepIndex = -1;
  } else {
    queueModel.steps.push(nextStep);
    showToast(`Added ${getSpotDisplayName(spot)}`);
  }

  queueModel.currentPresetId = null;
  publishQueueSpec();
}

function clearQueueSteps() {
  queueModel.steps = [];
  queueModel.selectedStepIndex = -1;
  queueModel.currentPresetId = null;
  publishQueueSpec();
  showToast("Queue cleared");
}

function updateStepRequestedState(index, requestedState) {
  if (!queueModel.steps[index]) return;
  queueModel.steps[index].requestedState = requestedState;
  queueModel.currentPresetId = null;
  publishQueueSpec();
}

function updateStepPoseValue(index, key, value) {
  if (!queueModel.steps[index]) return;
  const numeric = Number(value);
  if (!Number.isFinite(numeric)) {
    showToast("Pose value must be numeric");
    return;
  }
  queueModel.steps[index][key] =
    key === "headingDeg" ? wrapDegrees(numeric) : clampToFieldBounds(key, numeric);
  queueModel.currentPresetId = null;
  publishQueueSpec();
}

function ensureEditableRouteWaypoints(index) {
  if (!queueModel.steps[index]) return [];
  if (!Array.isArray(queueModel.steps[index].routeWaypoints) || queueModel.steps[index].routeWaypoints.length === 0) {
    queueModel.steps[index].routeWaypoints = getStepRouteWaypoints(index, queueModel.steps[index]).map(clonePose);
  }
  return queueModel.steps[index].routeWaypoints;
}

function updateRouteWaypointValue(stepIndex, waypointIndex, key, value) {
  if (!queueModel.steps[stepIndex]) return;
  const numeric = Number(value);
  if (!Number.isFinite(numeric)) {
    showToast("Waypoint value must be numeric");
    return;
  }
  const waypoints = ensureEditableRouteWaypoints(stepIndex);
  if (!waypoints[waypointIndex]) return;
  waypoints[waypointIndex][key] =
    key === "headingDeg" ? wrapDegrees(numeric) : clampToFieldBounds(key === "x" ? "xMeters" : "yMeters", numeric);
  queueModel.steps[stepIndex].routeWaypoints = waypoints;
  queueModel.currentPresetId = null;
  publishQueueSpec();
}

function insertRouteWaypoint(stepIndex, afterIndex) {
  if (!queueModel.steps[stepIndex]) return;
  const waypoints = ensureEditableRouteWaypoints(stepIndex);
  const previousPoseRaw =
    afterIndex >= 0
      ? authoringPoseToRuntimePose(waypoints[afterIndex])
      : getPreviousStepPose(stepIndex);
  const nextPoseRaw =
    afterIndex + 1 < waypoints.length
      ? authoringPoseToRuntimePose(waypoints[afterIndex + 1])
      : resolveStepPose(queueModel.steps[stepIndex]);
  const previousPose =
    previousPoseRaw && Number.isFinite(previousPoseRaw.headingDeg)
      ? authoringPoseToRuntimePose(previousPoseRaw)
      : previousPoseRaw;
  const nextPose =
    nextPoseRaw && Number.isFinite(nextPoseRaw.headingDeg)
      ? authoringPoseToRuntimePose(nextPoseRaw)
      : nextPoseRaw;
  if (!previousPose || !nextPose) return;
  const midpoint = {
    x: round((previousPose.x + nextPose.x) / 2, 3),
    y: round((previousPose.y + nextPose.y) / 2, 3),
    headingDeg: wrapDegrees((((previousPose.theta ?? 0) + (nextPose.theta ?? 0)) * 180) / (2 * Math.PI)),
  };
  waypoints.splice(afterIndex + 1, 0, midpoint);
  queueModel.steps[stepIndex].routeWaypoints = waypoints;
  queueModel.currentPresetId = null;
  publishQueueSpec();
}

function removeRouteWaypoint(stepIndex, waypointIndex) {
  if (!queueModel.steps[stepIndex]) return;
  const waypoints = ensureEditableRouteWaypoints(stepIndex).filter((_pose, index) => index !== waypointIndex);
  queueModel.steps[stepIndex].routeWaypoints = waypoints;
  queueModel.currentPresetId = null;
  publishQueueSpec();
}

function rebuildRouteWaypoints(stepIndex) {
  if (!queueModel.steps[stepIndex]) return;
  queueModel.steps[stepIndex].routeWaypoints = computeRouteWaypoints(
    getPreviousStepPose(stepIndex),
    queueModel.steps[stepIndex]
  ).map(clonePose);
  queueModel.currentPresetId = null;
  publishQueueSpec();
}

function moveStep(index, direction) {
  const nextIndex = index + direction;
  if (nextIndex < 0 || nextIndex >= queueModel.steps.length) return;
  const steps = [...queueModel.steps];
  const [step] = steps.splice(index, 1);
  steps.splice(nextIndex, 0, step);
  queueModel.steps = steps;
  queueModel.selectedStepIndex = nextIndex;
  queueModel.currentPresetId = null;
  publishQueueSpec();
}

function removeStep(index) {
  if (!queueModel.steps[index]) return;
  queueModel.steps = queueModel.steps.filter((_step, stepIndex) => stepIndex !== index);
  if (queueModel.selectedStepIndex === index) {
    queueModel.selectedStepIndex = -1;
  } else if (queueModel.selectedStepIndex > index) {
    queueModel.selectedStepIndex -= 1;
  }
  queueModel.currentPresetId = null;
  publishQueueSpec();
}

function buildStepActionButton(text, onClick) {
  const button = document.createElement("button");
  button.type = "button";
  button.className = "queue-step__action";
  button.innerText = text;
  button.addEventListener("click", (event) => {
    event.stopPropagation();
    onClick();
  });
  return button;
}

function buildStepPoseEditor(step, index) {
  const editor = document.createElement("div");
  editor.className = "queue-step__pose-editor";

  const fields = [
    { key: "xMeters", label: "X", value: step.xMeters },
    { key: "yMeters", label: "Y", value: step.yMeters },
    { key: "headingDeg", label: "H", value: step.headingDeg, stepValue: 1 },
  ];

  fields.forEach((field) => {
    const wrap = document.createElement("label");
    wrap.className = "queue-step__pose-field";

    const label = document.createElement("span");
    label.className = "queue-step__pose-label";
    label.innerText = field.label;

    const input = document.createElement("input");
    input.className = "queue-step__pose-input";
    input.type = "number";
    input.step = String(field.stepValue || 0.01);
    input.value = Number.isFinite(field.value) ? String(round(field.value, field.stepValue ? 1 : 3)) : "";
    input.addEventListener("change", () => {
      updateStepPoseValue(index, field.key, input.value);
    });

    wrap.append(label, input);
    editor.appendChild(wrap);
  });

  const routeWrap = document.createElement("div");
  routeWrap.className = "queue-step__route";

  const routeTop = document.createElement("div");
  routeTop.className = "queue-step__route-top";
  const routeTitle = document.createElement("div");
  routeTitle.className = "queue-step__route-title";
  routeTitle.innerText = "Route Waypoints";
  const routeControls = document.createElement("div");
  routeControls.className = "queue-step__route-controls";
  routeControls.append(
    buildPresetButton("Add", () => insertRouteWaypoint(index, getStepRouteWaypoints(index, step).length - 1)),
    buildPresetButton("Rebuild", () => rebuildRouteWaypoints(index))
  );
  routeTop.append(routeTitle, routeControls);

  const routeNote = document.createElement("div");
  routeNote.className = "queue-step__route-note";
  routeNote.innerText =
    step.routeWaypoints && step.routeWaypoints.length > 0
      ? "Editing authored intermediate poses."
      : "Generated from keep-out boxes. Change a value or press Add to make it editable.";

  const routeList = document.createElement("div");
  routeList.className = "queue-step__route-list";
  const routeWaypoints = getStepRouteWaypoints(index, step);
  if (routeWaypoints.length === 0) {
    const empty = document.createElement("div");
    empty.className = "queue-step__route-empty";
    empty.innerText = "No intermediate route points needed for this segment.";
    routeList.appendChild(empty);
  } else {
    routeWaypoints.forEach((waypoint, waypointIndex) => {
      const row = document.createElement("div");
      row.className = "queue-step__route-row";

      const badge = document.createElement("div");
      badge.className = "queue-step__route-index";
      badge.innerText = `${waypointIndex + 1}`;
      row.appendChild(badge);

      [
        { key: "x", label: "X", value: waypoint.x },
        { key: "y", label: "Y", value: waypoint.y },
        { key: "headingDeg", label: "H", value: waypoint.headingDeg, stepValue: 1 },
      ].forEach((field) => {
        const wrap = document.createElement("label");
        wrap.className = "queue-step__pose-field";
        const label = document.createElement("span");
        label.className = "queue-step__pose-label";
        label.innerText = field.label;
        const input = document.createElement("input");
        input.className = "queue-step__pose-input";
        input.type = "number";
        input.step = String(field.stepValue || 0.01);
        input.value = Number.isFinite(field.value)
          ? String(round(field.value, field.stepValue ? 1 : 3))
          : "";
        input.addEventListener("change", () => updateRouteWaypointValue(index, waypointIndex, field.key, input.value));
        wrap.append(label, input);
        row.appendChild(wrap);
      });

      row.appendChild(buildPresetButton("X", () => removeRouteWaypoint(index, waypointIndex)));
      routeList.appendChild(row);
    });
  }

  routeWrap.append(routeTop, routeNote, routeList);

  const wrap = document.createElement("div");
  wrap.append(editor, routeWrap);
  return wrap;
}

function buildPresetButton(text, onClick) {
  const button = document.createElement("button");
  button.type = "button";
  button.className = "preset__button";
  button.innerText = text;
  button.addEventListener("click", onClick);
  return button;
}

function buildActionOptions(select, selectedValue) {
  select.innerHTML = "";
  const options = [{ value: "", label: "No action" }, ...STATES.map((stateName) => ({
    value: stateName,
    label: stateName,
  }))];

  options.forEach((option) => {
    const element = document.createElement("option");
    element.value = option.value;
    element.textContent = option.label;
    element.selected = option.value === (selectedValue || "");
    select.appendChild(element);
  });
}

function saveNewPreset() {
  const name = sanitizePresetName(ui.presetNameInput ? ui.presetNameInput.value : "");
  if (!name) {
    showToast("Enter an auto name");
    return;
  }
  if (queueModel.steps.length === 0) {
    showToast("Queue is empty");
    return;
  }
  const alreadyExists = queueModel.presets.some(
    (preset) => preset.name.toLowerCase() === name.toLowerCase()
  );
  if (alreadyExists) {
    showToast("Auto name already exists");
    return;
  }

  const preset = {
    id: makePresetId(),
    name,
    updatedAt: Date.now(),
    startPose: clonePose(queueModel.startPose),
    customZones: cloneZones(queueModel.customZones),
    steps: cloneSteps(queueModel.steps),
  };
  queueModel.presets = [preset, ...queueModel.presets];
  queueModel.currentPresetId = preset.id;
  persistPresets();
  persistDraft();
  renderPresets();
  showToast(`Saved auto ${name}`);
}

function loadPreset(presetId) {
  const preset = queueModel.presets.find((entry) => entry.id === presetId);
  if (!preset) return;
  queueModel.steps = cloneSteps(preset.steps);
  queueModel.startPose = clonePose(preset.startPose);
  queueModel.customZones = cloneZones(preset.customZones);
  queueModel.selectedStepIndex = -1;
  queueModel.currentPresetId = preset.id;
  if (ui.presetNameInput) {
    ui.presetNameInput.value = preset.name;
  }
  persistCustomZones();
  renderZoneEditor();
  renderZoneList();
  renderStartPoseEditor();
  publishQueueSpec();
  renderPresets();
  renderField();
  showToast(`Loaded ${preset.name}`);
}

function overwritePreset(presetId) {
  const index = queueModel.presets.findIndex((entry) => entry.id === presetId);
  if (index < 0) return;
  if (queueModel.steps.length === 0) {
    showToast("Queue is empty");
    return;
  }
  queueModel.presets[index] = {
    ...queueModel.presets[index],
    updatedAt: Date.now(),
    startPose: clonePose(queueModel.startPose),
    customZones: cloneZones(queueModel.customZones),
    steps: cloneSteps(queueModel.steps),
  };
  queueModel.currentPresetId = presetId;
  persistPresets();
  renderPresets();
  persistDraft();
  showToast(`Overwrote ${queueModel.presets[index].name}`);
}

function renamePreset(presetId) {
  const index = queueModel.presets.findIndex((entry) => entry.id === presetId);
  if (index < 0) return;
  const current = queueModel.presets[index];
  const nextName = sanitizePresetName(window.prompt("Rename preset", current.name) || "");
  if (!nextName || nextName === current.name) return;
  queueModel.presets[index] = {
    ...current,
    name: nextName,
    updatedAt: Date.now(),
  };
  if (ui.presetNameInput && queueModel.currentPresetId === presetId) {
    ui.presetNameInput.value = nextName;
  }
  persistPresets();
  renderPresets();
  persistDraft();
  showToast(`Renamed to ${nextName}`);
}

function deletePreset(presetId) {
  const preset = queueModel.presets.find((entry) => entry.id === presetId);
  if (!preset) return;
  const shouldDelete = window.confirm(`Delete auto "${preset.name}"?`);
  if (!shouldDelete) return;
  queueModel.presets = queueModel.presets.filter((entry) => entry.id !== presetId);
  if (queueModel.currentPresetId === presetId) {
    queueModel.currentPresetId = null;
    queueModel.steps = [];
    queueModel.startPose = null;
  }
  persistPresets();
  renderPresets();
  persistDraft();
  renderQueueStatus();
  renderQueueSteps();
  renderField();
  showToast(`Deleted ${preset.name}`);
}

function persistPresets() {
  try {
    localStorage.setItem(PRESET_STORAGE_KEY, JSON.stringify(queueModel.presets));
  } catch (_err) {
    showToast("Preset save failed");
  }
}

function loadPresets() {
  try {
    const raw = localStorage.getItem(PRESET_STORAGE_KEY);
    if (!raw) {
      return clonePresets(layoutState.defaultPresets);
    }
    const parsed = JSON.parse(raw);
    if (!Array.isArray(parsed)) return clonePresets(layoutState.defaultPresets);
    return parsed
      .map(normalizePresetObject)
      .filter((preset) => preset.steps.length > 0);
  } catch (_err) {
    return clonePresets(layoutState.defaultPresets);
  }
}

function persistSelectedAutoId(autoId) {
  try {
    if (autoId) {
      localStorage.setItem(SELECTED_AUTO_STORAGE_KEY, autoId);
    } else {
      localStorage.removeItem(SELECTED_AUTO_STORAGE_KEY);
    }
  } catch (_err) {
    void _err;
  }
}

function loadStoredSelectedAutoId() {
  try {
    return parseString(localStorage.getItem(SELECTED_AUTO_STORAGE_KEY));
  } catch (_err) {
    return null;
  }
}

function clearStoredSelectedAutoId() {
  persistSelectedAutoId("");
}

function persistDraft() {
  try {
    localStorage.setItem(
      DRAFT_STORAGE_KEY,
      JSON.stringify({
        currentPresetId: queueModel.currentPresetId,
        selectedAction: queueModel.selectedAction,
        spotGroupFilter: queueModel.spotGroupFilter,
        startPose: queueModel.startPose,
        customZones: queueModel.customZones,
        steps: queueModel.steps,
      })
    );
  } catch (_err) {
    showToast("Local auto draft save failed");
  }
}

function loadDraft() {
  try {
    const raw = localStorage.getItem(DRAFT_STORAGE_KEY);
    if (!raw) return null;
    const parsed = JSON.parse(raw);
    if (!parsed || typeof parsed !== "object") return null;
    return {
      currentPresetId: parseString(parsed.currentPresetId),
      selectedAction: normalizeRequestedState(parsed.selectedAction),
      spotGroupFilter: parseString(parsed.spotGroupFilter) || "ALL",
      startPose: normalizeAuthoringPose(parsed.startPose),
      customZones: Array.isArray(parsed.customZones)
        ? parsed.customZones.map(normalizeNoGoZone).filter(Boolean)
        : [],
      steps: Array.isArray(parsed.steps)
        ? parsed.steps
            .map(normalizeQueueStep)
            .filter((step) => step.spotId || (Number.isFinite(step.xMeters) && Number.isFinite(step.yMeters)))
        : [],
    };
  } catch (_err) {
    return null;
  }
}

function applyDraft(draft) {
  if (!draft) return;
  queueModel.currentPresetId = draft.currentPresetId;
  queueModel.selectedAction = draft.selectedAction;
  queueModel.spotGroupFilter = draft.spotGroupFilter;
  queueModel.startPose = draft.startPose;
  queueModel.customZones = draft.customZones;
  queueModel.steps = cloneSteps(draft.steps || []);
  if (ui.presetNameInput && queueModel.currentPresetId) {
    const preset = queueModel.presets.find((entry) => entry.id === queueModel.currentPresetId);
    if (preset) {
      ui.presetNameInput.value = preset.name;
    }
  }
}

function createNewDraftAuto() {
  queueModel.steps = [];
  queueModel.startPose = null;
  queueModel.selectedStepIndex = -1;
  queueModel.currentPresetId = null;
  queueModel.customZones = loadCustomZones();
  if (ui.presetNameInput) {
    ui.presetNameInput.value = "";
    ui.presetNameInput.focus();
  }
  persistDraft();
  renderPresets();
  renderStartPoseEditor();
  renderZoneList();
  renderQueueStatus();
  renderQueueMeta();
  renderQueueSteps();
  renderField();
  showToast("Started a new local auto");
}

function syncQueueFromDashboard(queueState) {
  if (!queueState || !Array.isArray(queueState.steps)) return;
  queueModel.revision = Number.isFinite(queueState.revision)
    ? queueState.revision
    : queueModel.revision;
  queueModel.steps = queueState.steps.map((step) => ({
    type: parseString(step.type) || "PATH",
    spotId: step.spotId,
    commandName: sanitizePresetName(step.commandName) || "",
    label: step.label || "",
    requestedState: normalizeRequestedState(step.requestedState),
    status: normalizeQueueStatus(step.status),
    group: normalizeSpotGroup(step.group),
    xMeters: parseFiniteOrNull(step.xMeters),
    yMeters: parseFiniteOrNull(step.yMeters),
    headingDeg: parseFiniteOrNull(step.headingDeg),
    waitSeconds: parseFiniteOrNull(step.waitSeconds),
    routeWaypoints: Array.isArray(step.routeWaypoints)
      ? step.routeWaypoints.map(normalizeAuthoringPose).filter(Boolean)
      : [],
  }));
  queueModel.startPose = normalizeAuthoringPose(queueState.startPose) || queueModel.startPose;
  queueModel.customZones = Array.isArray(queueState.noGoZones)
    ? queueState.noGoZones.map(normalizeNoGoZone).filter(Boolean)
    : queueModel.customZones;
  if (queueModel.selectedStepIndex >= queueModel.steps.length) {
    queueModel.selectedStepIndex = -1;
  }
  persistCustomZones();
  renderZoneList();
  renderStartPoseEditor();
  renderQueueStatus();
  renderQueueMeta();
  renderQueueSteps();
  renderField();
}

function getEffectiveQueueState() {
  const dashboard = state.autoQueueState;
  if (dashboard) {
    return dashboard;
  }
  return {
    revision: queueModel.revision,
    running: false,
    phase: queueModel.steps.length > 0 ? "READY" : "IDLE",
    activeIndex: -1,
    message:
      queueModel.steps.length > 0
        ? ntConnected
          ? "Auto selected on dashboard."
          : "Planner auto loaded from deploy. Connect the robot to select it live."
        : "Select a deployed PathPlanner auto.",
    activeLabel: "",
    startPose: queueModel.startPose,
    noGoZones: getAllNoGoZones(),
    steps: queueModel.steps,
  };
}

function renderStartPoseEditor() {
  setText(
    ui.startPoseMode,
    queueModel.startPoseCaptureMode ? "Click the field to capture X/Y" : "Manual entry"
  );
  if (ui.startPoseCaptureButton) {
    ui.startPoseCaptureButton.classList.toggle("is-active", queueModel.startPoseCaptureMode);
  }
  if (ui.startPoseX && queueModel.startPose) {
    ui.startPoseX.value = String(round(queueModel.startPose.x, 3));
  }
  if (ui.startPoseY && queueModel.startPose) {
    ui.startPoseY.value = String(round(queueModel.startPose.y, 3));
  }
  if (ui.startPoseHeading && queueModel.startPose) {
    ui.startPoseHeading.value = String(round(queueModel.startPose.headingDeg, 1));
  } else if (ui.startPoseHeading) {
    ui.startPoseHeading.value = "";
  }
  if (!queueModel.startPose) {
    if (ui.startPoseX) ui.startPoseX.value = "";
    if (ui.startPoseY) ui.startPoseY.value = "";
  }
}

function renderZoneEditor() {
  const captureLabel = queueModel.zoneCaptureAnchor ? "Click second corner" : "Manual entry";
  setText(ui.zoneMode, queueModel.zoneCaptureMode ? captureLabel : "Manual entry");
  if (ui.zoneCaptureButton) {
    ui.zoneCaptureButton.classList.toggle("is-active", queueModel.zoneCaptureMode);
  }
}

function renderZoneList() {
  if (ui.zoneList) {
    ui.zoneList.innerHTML = "";
  }
  const zones = getAllNoGoZones();
  if (ui.zoneCount) {
    setText(ui.zoneCount, String(zones.length));
  }
  if (ui.zoneEmptyState) {
    ui.zoneEmptyState.style.display = zones.length === 0 ? "block" : "none";
  }
  if (!ui.zoneList) return;
  zones.forEach((zone) => {
    const item = document.createElement("div");
    item.className = "custom-spot";
    item.classList.toggle("is-editing", zone.id === queueModel.editingZoneId);

    const title = document.createElement("div");
    title.className = "custom-spot__title";
    title.innerText = zone.locked ? `${zone.label} (Locked)` : zone.label;

    const meta = document.createElement("div");
    meta.className = "custom-spot__meta";
    meta.innerText = `x=${round(zone.xMinMeters, 2)}..${round(zone.xMaxMeters, 2)} • y=${round(
      zone.yMinMeters,
      2
    )}..${round(zone.yMaxMeters, 2)}`;

    const actions = document.createElement("div");
    actions.className = "custom-spot__actions";
    actions.append(buildCustomSpotButton(zone.locked ? "Unlock" : "Lock", () => toggleZoneLock(zone.id)));
    if (!zone.locked) {
      actions.append(buildCustomSpotButton("Edit", () => loadZoneIntoEditor(zone.id)));
    }
    if (isCustomZone(zone.id)) {
      actions.append(buildCustomSpotButton("Delete", () => deleteCustomZone(zone.id)));
    }

    item.append(title, meta, actions);
    ui.zoneList.appendChild(item);
  });
}

function saveStartPose() {
  const pose = readStartPoseForm();
  if (!pose) return;
  queueModel.startPose = pose;
  queueModel.startPoseCaptureMode = false;
  renderStartPoseEditor();
  publishQueueSpec();
  showToast("Start pose updated");
}

function clearStartPose() {
  queueModel.startPose = null;
  queueModel.startPoseCaptureMode = false;
  if (ui.startPoseX) ui.startPoseX.value = "";
  if (ui.startPoseY) ui.startPoseY.value = "";
  if (ui.startPoseHeading) ui.startPoseHeading.value = "";
  renderStartPoseEditor();
  publishQueueSpec();
  showToast("Start pose cleared");
}

function readStartPoseForm() {
  const x = Number(ui.startPoseX?.value);
  const y = Number(ui.startPoseY?.value);
  const headingDeg = Number(ui.startPoseHeading?.value || 0);
  if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(headingDeg)) {
    showToast("Start pose needs X, Y, and heading");
    return null;
  }
  return {
    x: clamp(x, 0, FIELD_LENGTH_METERS),
    y: clamp(y, 0, FIELD_WIDTH_METERS),
    headingDeg: wrapDegrees(headingDeg),
  };
}

function handleZoneCaptureClick(pose) {
  if (!queueModel.zoneCaptureAnchor) {
    queueModel.zoneCaptureAnchor = pose;
    renderZoneEditor();
    showToast("Captured first corner");
    return;
  }
  setZoneBounds(queueModel.zoneCaptureAnchor, pose);
  queueModel.zoneCaptureAnchor = null;
  queueModel.zoneCaptureMode = false;
  renderZoneEditor();
  showToast("Captured keep-out box");
}

function saveCustomZone() {
  const zone = readZoneForm();
  if (!zone) return;
  if (queueModel.editingZoneId) {
    queueModel.customZones = queueModel.customZones.map((entry) =>
      entry.id === queueModel.editingZoneId ? { ...zone, id: entry.id, locked: false } : entry
    );
  } else {
    queueModel.customZones = [
      { ...zone, id: `zone-${Date.now()}-${Math.floor(Math.random() * 100000)}`, locked: false },
      ...queueModel.customZones,
    ];
  }
  persistCustomZones();
  resetZoneEditor();
  renderZoneList();
  publishQueueSpec();
  showToast("Keep-out box saved");
}

function readZoneForm() {
  const label = sanitizePresetName(ui.zoneName?.value || "") || "Keep-Out";
  const xMin = Number(ui.zoneXMin?.value);
  const xMax = Number(ui.zoneXMax?.value);
  const yMin = Number(ui.zoneYMin?.value);
  const yMax = Number(ui.zoneYMax?.value);
  if (![xMin, xMax, yMin, yMax].every(Number.isFinite)) {
    showToast("Keep-out box needs all four bounds");
    return null;
  }
  return normalizeNoGoZone({ id: makePresetId(), label, xMin, xMax, yMin, yMax, locked: false });
}

function resetZoneEditor() {
  queueModel.editingZoneId = null;
  queueModel.zoneCaptureMode = false;
  queueModel.zoneCaptureAnchor = null;
  if (ui.zoneName) ui.zoneName.value = "";
  if (ui.zoneXMin) ui.zoneXMin.value = "";
  if (ui.zoneXMax) ui.zoneXMax.value = "";
  if (ui.zoneYMin) ui.zoneYMin.value = "";
  if (ui.zoneYMax) ui.zoneYMax.value = "";
  renderZoneEditor();
}

function loadZoneIntoEditor(zoneId) {
  const zone = getAllNoGoZones().find((entry) => entry.id === zoneId);
  if (!zone) return;
  if (!isCustomZone(zone.id)) {
    queueModel.customZones = [
      ...queueModel.customZones,
      { ...zone },
    ];
  }
  queueModel.editingZoneId = zone.id;
  if (ui.zoneName) ui.zoneName.value = zone.label;
  if (ui.zoneXMin) ui.zoneXMin.value = String(round(zone.xMinMeters, 3));
  if (ui.zoneXMax) ui.zoneXMax.value = String(round(zone.xMaxMeters, 3));
  if (ui.zoneYMin) ui.zoneYMin.value = String(round(zone.yMinMeters, 3));
  if (ui.zoneYMax) ui.zoneYMax.value = String(round(zone.yMaxMeters, 3));
  renderZoneEditor();
}

function deleteCustomZone(zoneId) {
  queueModel.customZones = queueModel.customZones.filter((zone) => zone.id !== zoneId);
  if (queueModel.editingZoneId === zoneId) {
    resetZoneEditor();
  }
  persistCustomZones();
  renderZoneList();
  publishQueueSpec();
}

function toggleZoneLock(zoneId) {
  const existingIndex = queueModel.customZones.findIndex((zone) => zone.id === zoneId);
  if (existingIndex >= 0) {
    queueModel.customZones[existingIndex] = {
      ...queueModel.customZones[existingIndex],
      locked: !queueModel.customZones[existingIndex].locked,
    };
  } else {
    const defaultZone = layoutState.defaultZones.find((zone) => zone.id === zoneId);
    if (!defaultZone) return;
    queueModel.customZones = [
      ...queueModel.customZones,
      { ...defaultZone, locked: !defaultZone.locked },
    ];
  }
  persistCustomZones();
  renderZoneList();
  publishQueueSpec();
}

function renderCustomSpotEditor() {
  setText(
    ui.customSpotMode,
    queueModel.customSpotCaptureMode
      ? "Click the field to capture X/Y"
      : queueModel.editingCustomSpotId
        ? "Editing saved spot"
      : "Manual entry"
  );
  if (ui.customSpotSaveButton) {
    ui.customSpotSaveButton.innerText = queueModel.editingCustomSpotId ? "Update Spot" : "Save Spot";
  }
  if (ui.customSpotCaptureButton) {
    ui.customSpotCaptureButton.classList.toggle("is-active", queueModel.customSpotCaptureMode);
  }
  if (ui.fieldCanvas && queueModel.customSpotCaptureMode) {
    ui.fieldCanvas.style.cursor = "crosshair";
  }
}

function renderCustomSpots() {
  if (ui.customSpotList) {
    ui.customSpotList.innerHTML = "";
  }
  if (ui.customSpotCount) {
    setText(ui.customSpotCount, String(queueModel.customSpots.length));
  }
  if (ui.customSpotEmptyState) {
    ui.customSpotEmptyState.style.display = queueModel.customSpots.length === 0 ? "block" : "none";
  }
  if (!ui.customSpotList) return;

  queueModel.customSpots.forEach((spot) => {
    const item = document.createElement("div");
    item.className = "custom-spot";
    item.classList.toggle("is-editing", spot.id === queueModel.editingCustomSpotId);

    const title = document.createElement("div");
    title.className = "custom-spot__title";
    title.innerText = spot.label;

    const meta = document.createElement("div");
    meta.className = "custom-spot__meta";
    meta.innerText = `${spot.group} • x=${round(spot.x, 2)} y=${round(spot.y, 2)} h=${round(
      spot.headingDeg,
      0
    )}deg`;

    const actions = document.createElement("div");
    actions.className = "custom-spot__actions";
    actions.append(
      buildCustomSpotButton("Edit", () => loadCustomSpotIntoEditor(spot.id)),
      buildCustomSpotButton("Queue", () => addOrReplaceQueueStep(spot)),
      buildCustomSpotButton("Delete", () => deleteCustomSpot(spot.id))
    );

    item.append(title, meta, actions);
    ui.customSpotList.appendChild(item);
  });
}

function buildCustomSpotButton(text, onClick) {
  const button = document.createElement("button");
  button.type = "button";
  button.className = "custom-spot__button";
  button.innerText = text;
  button.addEventListener("click", (event) => {
    event.stopPropagation();
    onClick();
  });
  return button;
}

function getStateColor(stateName) {
  return STATE_COLORS[stateName || ""] || "rgba(255,255,255,0.78)";
}

function uniquePresetStates(preset) {
  const unique = Array.from(
    new Set((preset.steps || []).map((step) => normalizeRequestedState(step.requestedState)).filter(Boolean))
  );
  return unique.length > 0 ? unique : [""];
}

function buildPresetPreview(preset) {
  const container = document.createElement("div");
  container.className = "preset__preview";
  container.innerHTML = buildPresetPreviewSvg(preset);
  return container;
}

function buildPresetPreviewSvg(preset) {
  const segments = buildPresetSegmentsForPreview(preset);
  const zones = buildZonesForPreview(preset);
  const zoneSvg = zones
    .map((zone) => {
      const x = (zone.xMinMeters / FIELD_LENGTH_METERS) * 100;
      const y = (1 - zone.yMaxMeters / FIELD_WIDTH_METERS) * 100;
      const width = ((zone.xMaxMeters - zone.xMinMeters) / FIELD_LENGTH_METERS) * 100;
      const height = ((zone.yMaxMeters - zone.yMinMeters) / FIELD_WIDTH_METERS) * 100;
      return `<rect x="${x}" y="${y}" width="${width}" height="${height}" rx="1.5" fill="rgba(255,99,132,0.12)" stroke="rgba(255,99,132,0.45)" stroke-width="0.6" />`;
    })
    .join("");
  const linesSvg = segments
    .map((segment) => {
      const points = segment.poses
        .map((pose) => `${(pose.x / FIELD_LENGTH_METERS) * 100},${(1 - pose.y / FIELD_WIDTH_METERS) * 100}`)
        .join(" ");
      return `<polyline points="${points}" fill="none" stroke="${getStateColor(segment.requestedState)}" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round" />`;
    })
    .join("");
  const robotSvg = segments
    .flatMap((segment) => [segment.finalPose, ...segment.waypoints])
    .map((pose, index) => buildPreviewFootprintSvg(pose, index === 0 ? 0.7 : 0.5))
    .join("");
  const startPose = normalizeAuthoringPose(preset.startPose);
  const startSvg = startPose ? buildPreviewFootprintSvg(authoringPoseToRuntimePose(startPose), 0.85, "#ffe8ad") : "";
  return `
    <svg viewBox="0 0 100 46" preserveAspectRatio="none" aria-hidden="true">
      <rect x="0.5" y="0.5" width="99" height="45" rx="4" fill="rgba(5,8,12,0.02)" stroke="rgba(255,255,255,0.08)" />
      <line x1="${(SAFE_AUTO_MAX_X_METERS / FIELD_LENGTH_METERS) * 100}" y1="0" x2="${(SAFE_AUTO_MAX_X_METERS / FIELD_LENGTH_METERS) * 100}" y2="46" stroke="rgba(255,255,255,0.12)" stroke-dasharray="2 2" />
      ${zoneSvg}
      ${linesSvg}
      ${startSvg}
      ${robotSvg}
    </svg>
  `;
}

function buildZonesForPreview(preset) {
  const merged = new Map();
  layoutState.defaultZones.forEach((zone) => merged.set(zone.id, { ...zone }));
  (preset.customZones || []).forEach((zone) => {
    const normalized = normalizeNoGoZone(zone);
    if (normalized) {
      merged.set(normalized.id, normalized);
    }
  });
  return [...merged.values()];
}

function buildPreviewFootprintSvg(pose, scale, stroke = "rgba(255,255,255,0.65)") {
  if (!pose) return "";
  const centerX = (pose.x / FIELD_LENGTH_METERS) * 100;
  const centerY = (1 - pose.y / FIELD_WIDTH_METERS) * 100;
  const width = (ROBOT_LENGTH_METERS / FIELD_LENGTH_METERS) * 100 * scale;
  const height = (ROBOT_WIDTH_METERS / FIELD_WIDTH_METERS) * 46 * scale;
  const rotation = -wrapDegrees(((pose.theta ?? 0) * 180) / Math.PI);
  return `<g transform="translate(${centerX} ${centerY}) rotate(${rotation})"><rect x="${-width / 2}" y="${-height / 2}" width="${width}" height="${height}" rx="1.2" fill="rgba(255,255,255,0.06)" stroke="${stroke}" stroke-width="0.7" /><line x1="0" y1="0" x2="${width / 2}" y2="0" stroke="${stroke}" stroke-width="0.7" /></g>`;
}

function buildPresetSegmentsForPreview(preset) {
  const steps = Array.isArray(preset.steps) ? preset.steps.map(normalizeQueueStep) : [];
  let previousPose = normalizeAuthoringPose(preset.startPose)
    ? authoringPoseToRuntimePose(normalizeAuthoringPose(preset.startPose))
    : null;
  const segments = [];
  steps.forEach((step, index) => {
    const finalPose = resolveStepPose(step);
    if (!finalPose || !previousPose) return;
    const waypoints = (Array.isArray(step.routeWaypoints) && step.routeWaypoints.length > 0
      ? step.routeWaypoints.map(normalizeAuthoringPose).filter(Boolean)
      : computeRouteWaypoints(index === 0 ? previousPose : segments[segments.length - 1]?.finalPose || previousPose, step)
    ).map(clonePose);
    const poses = [previousPose, ...waypoints.map(authoringPoseToRuntimePose), finalPose];
    segments.push({
      requestedState: step.requestedState,
      finalPose,
      waypoints,
      poses: dedupePath(poses),
    });
    previousPose = finalPose;
  });
  return segments;
}

function saveCustomSpot() {
  const spot = readCustomSpotForm();
  if (!spot) return;
  if (queueModel.editingCustomSpotId) {
    queueModel.customSpots = queueModel.customSpots.map((entry) =>
      entry.id === queueModel.editingCustomSpotId ? { ...spot, id: entry.id } : entry
    );
    showToast(`Updated ${spot.label}`);
  } else {
    queueModel.customSpots = [
      { ...spot, id: `custom-${Date.now()}-${Math.floor(Math.random() * 100000)}` },
      ...queueModel.customSpots,
    ];
    showToast(`Saved ${spot.label}`);
  }
  persistCustomSpots();
  resetCustomSpotEditor();
  renderCustomSpots();
  renderQueueMeta();
  renderField();
}

function readCustomSpotForm() {
  const label = sanitizePresetName(ui.customSpotName?.value || "");
  const group = normalizeSpotGroup(ui.customSpotGroup?.value || "LANE");
  const x = Number(ui.customSpotX?.value);
  const y = Number(ui.customSpotY?.value);
  const headingDeg = Number(ui.customSpotHeading?.value || 0);
  if (!label) {
    showToast("Custom spot needs a label");
    return null;
  }
  if (!Number.isFinite(x) || !Number.isFinite(y)) {
    showToast("Custom spot needs X and Y");
    return null;
  }
  if (!Number.isFinite(headingDeg)) {
    showToast("Heading must be a number");
    return null;
  }
  return {
    label,
    shortLabel: label.length > 10 ? label.slice(0, 10) : label,
    group,
    x: clamp(x, 0, FIELD_LENGTH_METERS),
    y: clamp(y, 0, FIELD_WIDTH_METERS),
    headingDeg,
    approachDistanceMeters: 0,
  };
}

function resetCustomSpotEditor() {
  queueModel.editingCustomSpotId = null;
  queueModel.customSpotCaptureMode = false;
  if (ui.customSpotName) ui.customSpotName.value = "";
  if (ui.customSpotGroup) ui.customSpotGroup.value = "LANE";
  if (ui.customSpotX) ui.customSpotX.value = "";
  if (ui.customSpotY) ui.customSpotY.value = "";
  if (ui.customSpotHeading) ui.customSpotHeading.value = "";
  if (ui.fieldCanvas) ui.fieldCanvas.style.cursor = "default";
  renderCustomSpotEditor();
  renderCustomSpots();
}

function loadCustomSpotIntoEditor(spotId) {
  const spot = getSpotById(spotId);
  if (!spot) return;
  queueModel.editingCustomSpotId = spot.id;
  if (ui.customSpotName) ui.customSpotName.value = spot.label;
  if (ui.customSpotGroup) ui.customSpotGroup.value = spot.group;
  if (ui.customSpotX) ui.customSpotX.value = String(round(spot.x, 3));
  if (ui.customSpotY) ui.customSpotY.value = String(round(spot.y, 3));
  if (ui.customSpotHeading) ui.customSpotHeading.value = String(round(spot.headingDeg, 1));
  renderCustomSpotEditor();
  renderCustomSpots();
}

function deleteCustomSpot(spotId) {
  const stepUsesSpot = queueModel.steps.some((step) => step.spotId === spotId);
  queueModel.customSpots = queueModel.customSpots.filter((spot) => spot.id !== spotId);
  persistCustomSpots();
  if (queueModel.editingCustomSpotId === spotId) {
    resetCustomSpotEditor();
  } else {
    renderCustomSpots();
  }
  renderQueueMeta();
  renderField();
  if (stepUsesSpot) {
    showToast("Custom spot deleted. Queue still retains inline pose data.");
  }
}

function persistCustomSpots() {
  try {
    localStorage.setItem(CUSTOM_SPOT_STORAGE_KEY, JSON.stringify(queueModel.customSpots));
  } catch (_err) {
    showToast("Custom spot save failed");
  }
}

function loadCustomSpots() {
  try {
    const raw = localStorage.getItem(CUSTOM_SPOT_STORAGE_KEY);
    if (!raw) return [];
    const parsed = JSON.parse(raw);
    if (!Array.isArray(parsed)) return [];
    return parsed
      .map((spot) => normalizeSpotObject(spot))
      .filter(Boolean)
      .map((spot) => ({ ...spot, id: parseString(spot.id) || makePresetId() }));
  } catch (_err) {
    return [];
  }
}

function setCustomSpotCoordinates(x, y) {
  if (ui.customSpotX) ui.customSpotX.value = String(round(x, 3));
  if (ui.customSpotY) ui.customSpotY.value = String(round(y, 3));
}

function persistCustomZones() {
  try {
    localStorage.setItem(CUSTOM_ZONE_STORAGE_KEY, JSON.stringify(queueModel.customZones));
  } catch (_err) {
    showToast("Keep-out save failed");
  }
}

function loadCustomZones() {
  try {
    const raw = localStorage.getItem(CUSTOM_ZONE_STORAGE_KEY);
    if (!raw) return [];
    const parsed = JSON.parse(raw);
    if (!Array.isArray(parsed)) return [];
    return parsed.map(normalizeNoGoZone).filter(Boolean);
  } catch (_err) {
    return [];
  }
}

function setStartPoseCoordinates(x, y) {
  if (ui.startPoseX) ui.startPoseX.value = String(round(x, 3));
  if (ui.startPoseY) ui.startPoseY.value = String(round(y, 3));
}

function setZoneBounds(anchor, pose) {
  if (ui.zoneXMin) ui.zoneXMin.value = String(round(Math.min(anchor.x, pose.x), 3));
  if (ui.zoneXMax) ui.zoneXMax.value = String(round(Math.max(anchor.x, pose.x), 3));
  if (ui.zoneYMin) ui.zoneYMin.value = String(round(Math.min(anchor.y, pose.y), 3));
  if (ui.zoneYMax) ui.zoneYMax.value = String(round(Math.max(anchor.y, pose.y), 3));
}

function getAllNoGoZones() {
  const merged = new Map();
  layoutState.defaultZones.forEach((zone) => merged.set(zone.id, { ...zone }));
  queueModel.customZones.forEach((zone) => merged.set(zone.id, { ...zone }));
  return [...merged.values()];
}

function isCustomZone(zoneId) {
  return queueModel.customZones.some((zone) => zone.id === zoneId);
}

function getVisibleSpots() {
  return getAllSpots().filter((spot) => {
    return queueModel.spotGroupFilter === "ALL" || spot.group === queueModel.spotGroupFilter;
  });
}

function getDisplayAlliance() {
  return normalizeAlliance(state.alliance);
}

function resolveStepPose(step) {
  if (Number.isFinite(step.xMeters) && Number.isFinite(step.yMeters)) {
    return {
      x: step.xMeters,
      y: step.yMeters,
      theta: degreesToRadians(Number.isFinite(step.headingDeg) ? step.headingDeg : 0),
    };
  }
  const spot = getSpotById(step.spotId);
  return spot ? { x: spot.x, y: spot.y, theta: degreesToRadians(spot.headingDeg) } : null;
}

function describeStep(step) {
  const spot = getSpotById(step.spotId);
  if (spot) {
    return getSpotDisplayName(spot);
  }
  return step.label || step.spotId || "Unknown Spot";
}

function describeSpotGroup(spotId) {
  const spot = getSpotById(spotId);
  return spot ? spot.group : "UNKNOWN";
}

function isSelectedSpot(spotId) {
  if (queueModel.selectedStepIndex < 0) return false;
  const step = queueModel.steps[queueModel.selectedStepIndex];
  return !!step && step.spotId === spotId;
}

function findQueueIndexForSpot(spotId) {
  return queueModel.steps.findIndex((step) => step.spotId === spotId);
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

function formatOurHubStatus(ourHubStatus, ourHubActive, hubStatusValid) {
  if (!hubStatusValid) {
    return ourHubStatus || "UNKNOWN";
  }
  if (ourHubStatus) {
    return ourHubStatus;
  }
  return ourHubActive ? "ACTIVE" : "INACTIVE";
}

function formatHubFms(currentState) {
  const raw = (currentState.gameDataRaw || "").trim();
  const winner = currentState.autoWinnerAlliance || "UNKNOWN";
  const red = currentState.redHubStatus || "UNKNOWN";
  const blue = currentState.blueHubStatus || "UNKNOWN";

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

function applyDriverIndicator(element, value) {
  if (!element) return;
  if (value === null) {
    setText(element, "--");
    element.classList.remove("is-on", "is-off");
    return;
  }
  const isOn = !!value;
  setText(element, isOn ? "ON" : "OFF");
  element.classList.toggle("is-on", isOn);
  element.classList.toggle("is-off", !isOn);
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

function applySysIdClass(el, phase, active) {
  if (!el) return;
  el.classList.remove("tile__v--ok", "tile__v--bad", "tile__v--warn");
  if (active) {
    el.classList.add("tile__v--ok");
    return;
  }
  const token = (phase || "").toUpperCase();
  if (token === "DONE") {
    el.classList.add("tile__v--ok");
  } else if (token && token !== "IDLE" && token !== "UNAVAILABLE") {
    el.classList.add("tile__v--warn");
  }
}

function applyLogRollClass(el, status) {
  if (!el) return;
  el.classList.remove("kv__v--ok", "kv__v--bad", "kv__v--warn");
  const token = (status || "").toUpperCase();
  if (token === "ROLLED" || token === "READY" || token === "CLEANED") {
    el.classList.add("kv__v--ok");
  } else if (token === "FAILED" || token === "UNAVAILABLE") {
    el.classList.add("kv__v--bad");
  } else if (token) {
    el.classList.add("kv__v--warn");
  }
}

function formatSysId(phase, active, lastCompleted, lastCompletedPhase) {
  const phaseText = decodeSysIdPhase(phase || "UNKNOWN");
  const statusText = active ? "ACTIVE" : "IDLE";
  const lastPhaseText = lastCompletedPhase ? decodeSysIdPhase(lastCompletedPhase) : "--";
  const lastTimeText = formatTimestampSeconds(lastCompleted);
  return `${phaseText} • ${statusText} • Last=${lastPhaseText} @ ${lastTimeText}`;
}

function formatSysIdOptions(currentPhase, active, lastCompletedPhase) {
  const options = [
    ["Full Routine", "FULL"],
    ["Quasistatic Forward", "QS_FWD"],
    ["Quasistatic Reverse", "QS_REV"],
    ["Dynamic Forward", "DYN_FWD"],
    ["Dynamic Reverse", "DYN_REV"],
  ];
  const current = normalizeSysIdPhase(currentPhase);
  const last = normalizeSysIdPhase(lastCompletedPhase);
  const lines = options.map(([label, phase]) => {
    let status = "PENDING";
    if (phase === "FULL") {
      if (active && current !== "IDLE" && current !== "DONE" && current !== "UNKNOWN") {
        status = "RUNNING";
      } else if (last === "DYN_REV" || last === "SINGLE_DYN_REV") {
        status = "LAST COMPLETED";
      }
    } else if (active && current === phase) {
      status = "RUNNING";
    } else if (last === phase || last === `SINGLE_${phase}`) {
      status = "LAST COMPLETED";
    }
    return `${label}: ${status}`;
  });
  return lines.join("\n");
}

function formatTimestampSeconds(value) {
  if (!Number.isFinite(value)) return "--";
  return `t=${round(value, 1)}s`;
}

function normalizeSysIdPhase(phase) {
  if (!phase) return "UNKNOWN";
  const token = String(phase).toUpperCase().trim();
  if (token === "") return "UNKNOWN";
  return token;
}

function decodeSysIdPhase(phase) {
  const token = normalizeSysIdPhase(phase);
  switch (token) {
    case "QS_FWD":
      return "Full QS Forward";
    case "QS_REV":
      return "Full QS Reverse";
    case "DYN_FWD":
      return "Full Dynamic Forward";
    case "DYN_REV":
      return "Full Dynamic Reverse";
    case "SINGLE_QS_FWD":
      return "Single QS Forward";
    case "SINGLE_QS_REV":
      return "Single QS Reverse";
    case "SINGLE_DYN_FWD":
      return "Single Dynamic Forward";
    case "SINGLE_DYN_REV":
      return "Single Dynamic Reverse";
    case "DONE":
      return "Done";
    case "IDLE":
      return "Idle";
    case "UNAVAILABLE":
      return "Unavailable";
    default:
      return token;
  }
}

function formatPose(pose, valid) {
  if (!pose || valid === false) return "--";
  const x = round(pose.x, 3);
  const y = round(pose.y, 3);
  const deg = round((pose.theta ?? 0) * (180 / Math.PI), 1);
  return `x=${x} y=${y} th=${deg}deg`;
}

function formatAuthoringPose(pose) {
  if (!pose) return "Not set";
  return `x=${round(pose.x, 3)} y=${round(pose.y, 3)} h=${round(pose.headingDeg, 1)}deg`;
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
  const minutes = Math.floor(clamped / 60);
  const seconds = Math.floor(clamped % 60);
  return `${minutes}:${String(seconds).padStart(2, "0")}`;
}

function formatPresetTimestamp(timestamp) {
  if (!Number.isFinite(timestamp)) return "--";
  try {
    return new Date(timestamp).toLocaleString();
  } catch (_err) {
    return "--";
  }
}

function prettify(token) {
  return String(token)
    .toLowerCase()
    .replaceAll("_", " ")
    .replace(/\b\w/g, (match) => match.toUpperCase());
}

function parseString(value) {
  if (typeof value === "string") return value;
  if (value === null || value === undefined) return null;
  return String(value);
}

function parsePose(value) {
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

function parseQueueState(value) {
  const raw = parseString(value);
  if (!raw) return null;
  try {
    const parsed = JSON.parse(raw);
    if (!parsed || typeof parsed !== "object") return null;
    return {
      revision: Number.isFinite(parsed.revision) ? parsed.revision : queueModel.revision,
      running: !!parsed.running,
      phase: parseString(parsed.phase) || "IDLE",
      activeIndex: Number.isFinite(parsed.activeIndex) ? parsed.activeIndex : -1,
      message: parseString(parsed.message) || "",
      activeLabel: parseString(parsed.activeLabel) || "",
      startPose: normalizeAuthoringPose(parsed.startPose),
      noGoZones: Array.isArray(parsed.noGoZones)
        ? parsed.noGoZones.map(normalizeNoGoZone).filter(Boolean)
        : [],
      steps: Array.isArray(parsed.steps)
        ? parsed.steps
            .map(normalizeQueueStep)
            .filter((step) => step.spotId || (Number.isFinite(step.xMeters) && Number.isFinite(step.yMeters)))
        : [],
    };
  } catch (_err) {
    return null;
  }
}

function normalizeRequestedState(value) {
  const token = parseString(value);
  if (!token) return "";
  const normalized = token.trim().toUpperCase();
  return STATES.includes(normalized) ? normalized : "";
}

function normalizeQueueStatus(value) {
  const token = parseString(value);
  if (!token) return "PENDING";
  const normalized = token.trim().toUpperCase();
  return QUEUE_STATUSES.includes(normalized) ? normalized : "PENDING";
}

function normalizeAlliance(value) {
  const token = parseString(value);
  if (!token) return null;
  const normalized = token.trim().toUpperCase();
  return normalized === "BLUE" || normalized === "RED" ? normalized : null;
}

function normalizeSpotGroup(value) {
  const token = parseString(value);
  if (!token) return "LANE";
  const normalized = token.trim().toUpperCase();
  return SPOT_GROUPS.includes(normalized) ? normalized : "LANE";
}

function parseFiniteOrNull(value) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : null;
}

function normalizeSpotObject(raw) {
  if (!raw) return null;
  const id = parseString(raw.id);
  const label = sanitizePresetName(raw.label);
  const x = parseFiniteOrNull(raw.x ?? raw.xMeters);
  const y = parseFiniteOrNull(raw.y ?? raw.yMeters);
  const headingDeg = parseFiniteOrNull(raw.headingDeg);
  if (!id || !label || x === null || y === null || headingDeg === null) {
    return null;
  }
  return {
    id,
    label,
    shortLabel: sanitizePresetName(raw.shortLabel) || label,
    group: normalizeSpotGroup(raw.group),
    x,
    y,
    headingDeg,
    approachDistanceMeters: parseFiniteOrNull(raw.approachDistanceMeters) || 0,
  };
}

function normalizeAuthoringPose(raw) {
  if (!raw) return null;
  const x = parseFiniteOrNull(raw.x ?? raw.xMeters);
  const y = parseFiniteOrNull(raw.y ?? raw.yMeters);
  const headingDeg = parseFiniteOrNull(raw.headingDeg ?? raw.heading);
  if (x === null || y === null) {
    return null;
  }
  return {
    x: clamp(x, 0, FIELD_LENGTH_METERS),
    y: clamp(y, 0, FIELD_WIDTH_METERS),
    headingDeg: wrapDegrees(headingDeg ?? 0),
  };
}

function normalizeNoGoZone(raw) {
  if (!raw) return null;
  const id = parseString(raw.id) || `zone-${Date.now()}-${Math.floor(Math.random() * 100000)}`;
  const label = sanitizePresetName(raw.label) || "Keep-Out";
  const xMin = parseFiniteOrNull(raw.xMin ?? raw.xMinMeters);
  const xMax = parseFiniteOrNull(raw.xMax ?? raw.xMaxMeters);
  const yMin = parseFiniteOrNull(raw.yMin ?? raw.yMinMeters);
  const yMax = parseFiniteOrNull(raw.yMax ?? raw.yMaxMeters);
  if (xMin === null || xMax === null || yMin === null || yMax === null) {
    return null;
  }
  return {
    id,
    label,
    xMinMeters: clamp(Math.min(xMin, xMax), 0, FIELD_LENGTH_METERS),
    xMaxMeters: clamp(Math.max(xMin, xMax), 0, FIELD_LENGTH_METERS),
    yMinMeters: clamp(Math.min(yMin, yMax), 0, FIELD_WIDTH_METERS),
    yMaxMeters: clamp(Math.max(yMin, yMax), 0, FIELD_WIDTH_METERS),
    locked: !!raw.locked,
  };
}

async function loadLayoutSpec() {
  try {
    const response = await fetch("./rebuilt-spots.json", { cache: "no-store" });
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }
    const spec = await response.json();
    applyLayoutSpec(spec);
  } catch (_err) {
    applyLayoutSpec(buildDefaultLayoutSpec());
    showToast("Using fallback auto layout");
  }
}

function applyLayoutSpec(spec) {
  const field = spec?.field || {};
  FIELD_LENGTH_METERS = parseFiniteOrNull(field.lengthMeters) || FIELD_LENGTH_METERS;
  FIELD_WIDTH_METERS = parseFiniteOrNull(field.widthMeters) || FIELD_WIDTH_METERS;
  SAFE_AUTO_MAX_X_METERS = parseFiniteOrNull(field.safeAutoMaxXMeters) || SAFE_AUTO_MAX_X_METERS;
  ROBOT_LENGTH_METERS = parseFiniteOrNull(field.robotLengthMeters) || ROBOT_LENGTH_METERS;
  ROBOT_WIDTH_METERS = parseFiniteOrNull(field.robotWidthMeters) || ROBOT_WIDTH_METERS;
  layoutState.referenceAlliance = parseString(field.referenceAlliance) || "BLUE";
  layoutState.spots = Array.isArray(spec?.spots) ? spec.spots.map(normalizeSpotObject).filter(Boolean) : [];
  layoutState.defaultZones = Array.isArray(spec?.noGoZones)
    ? spec.noGoZones.map(normalizeNoGoZone).filter(Boolean)
    : [];
  layoutState.defaultPresets = Array.isArray(spec?.presets)
    ? spec.presets.map(normalizePresetObject).filter((preset) => preset.steps.length > 0)
    : [];
}

function buildDefaultLayoutSpec() {
  return {
    field: {
      lengthMeters: 17.548,
      widthMeters: 8.052,
      safeAutoMaxXMeters: 8.15,
      robotLengthMeters: 0.6985,
      robotWidthMeters: 0.6985,
      referenceAlliance: "BLUE",
    },
    noGoZones: [
      { id: "hub", label: "Hub", xMin: 3.55, yMin: 2.72, xMax: 5.72, yMax: 5.33, locked: true },
      {
        id: "tower",
        label: "Tower",
        xMin: 1.72,
        yMin: 2.78,
        xMax: 2.72,
        yMax: 4.66,
        locked: true,
      },
      {
        id: "upper-trench",
        label: "Upper Trench",
        xMin: 4.86,
        yMin: 6.9,
        xMax: 7.92,
        yMax: 8.052,
        locked: true,
      },
      {
        id: "lower-trench",
        label: "Lower Trench",
        xMin: 4.86,
        yMin: 0.0,
        xMax: 7.92,
        yMax: 1.15,
        locked: true,
      },
    ],
    spots: [
      {
        id: "hub-near",
        label: "Hub Near",
        shortLabel: "Hub N",
        group: "HUB",
        x: 3.08,
        y: 4.026,
        headingDeg: 0,
      },
      {
        id: "hub-far",
        label: "Hub Far",
        shortLabel: "Hub F",
        group: "HUB",
        x: 6.05,
        y: 4.026,
        headingDeg: 180,
      },
      {
        id: "hub-left",
        label: "Hub Left",
        shortLabel: "Hub L",
        group: "HUB",
        x: 4.61,
        y: 5.48,
        headingDeg: -90,
      },
      {
        id: "hub-right",
        label: "Hub Right",
        shortLabel: "Hub R",
        group: "HUB",
        x: 4.61,
        y: 2.58,
        headingDeg: 90,
      },
      {
        id: "depot",
        label: "Depot Intake",
        shortLabel: "Depot",
        group: "DEPOT",
        x: 1.82,
        y: 5.92,
        headingDeg: 180,
        approachDistanceMeters: 0.55,
      },
      {
        id: "outpost",
        label: "Outpost Intake",
        shortLabel: "Outpost",
        group: "OUTPOST",
        x: 1.24,
        y: 0.82,
        headingDeg: 180,
        approachDistanceMeters: 0.55,
      },
      {
        id: "tower-center",
        label: "Tower Center",
        shortLabel: "Tower C",
        group: "TOWER",
        x: 2.95,
        y: 3.95,
        headingDeg: 180,
      },
      {
        id: "tower-left",
        label: "Tower Left",
        shortLabel: "Tower L",
        group: "TOWER",
        x: 2.95,
        y: 4.56,
        headingDeg: 180,
      },
      {
        id: "tower-right",
        label: "Tower Right",
        shortLabel: "Tower R",
        group: "TOWER",
        x: 2.95,
        y: 3.32,
        headingDeg: 180,
      },
      {
        id: "left-trench",
        label: "Upper Trench Safe",
        shortLabel: "Trench U",
        group: "TRENCH",
        x: 4.78,
        y: 6.55,
        headingDeg: 0,
      },
      {
        id: "right-trench",
        label: "Lower Trench Safe",
        shortLabel: "Trench L",
        group: "TRENCH",
        x: 4.78,
        y: 1.46,
        headingDeg: 0,
      },
      {
        id: "left-bump",
        label: "Upper Bump",
        shortLabel: "Bump U",
        group: "BUMP",
        x: 4.42,
        y: 5.73,
        headingDeg: 0,
      },
      {
        id: "right-bump",
        label: "Lower Bump",
        shortLabel: "Bump L",
        group: "BUMP",
        x: 4.42,
        y: 2.32,
        headingDeg: 0,
      },
      {
        id: "mid-ball-upper",
        label: "Mid Ball Upper",
        shortLabel: "Ball U",
        group: "LANE",
        x: 7.94,
        y: 5.82,
        headingDeg: 180,
      },
      {
        id: "mid-ball-lower",
        label: "Mid Ball Lower",
        shortLabel: "Ball L",
        group: "LANE",
        x: 7.94,
        y: 2.22,
        headingDeg: 180,
      },
    ],
    presets: [
      {
        id: "hub-cycle-upper",
        name: "Hub Cycle Upper",
        startPose: { xMeters: 1.55, yMeters: 5.75, headingDeg: 180 },
        steps: [
          { spotId: "depot", requestedState: "INTAKING" },
          { spotId: "hub-left", requestedState: "SHOOTING" },
        ],
      },
      {
        id: "hub-cycle-lower",
        name: "Hub Cycle Lower",
        startPose: { xMeters: 1.1, yMeters: 1.08, headingDeg: 180 },
        steps: [
          { spotId: "outpost", requestedState: "INTAKING" },
          { spotId: "hub-right", requestedState: "SHOOTING" },
        ],
      },
    ],
  };
}

function normalizePresetObject(raw) {
  if (!raw) return null;
  const steps = Array.isArray(raw.steps)
    ? raw.steps
        .map(normalizeQueueStep)
        .filter((step) => step.spotId || (Number.isFinite(step.xMeters) && Number.isFinite(step.yMeters)))
    : [];
  return {
    id: parseString(raw.id) || makePresetId(),
    name: sanitizePresetName(raw.name || raw.label) || "Unnamed Preset",
    folder: sanitizePresetName(raw.folder) || "",
    relativePath: parseString(raw.relativePath) || "",
    updatedAt: Number.isFinite(raw.updatedAt) ? raw.updatedAt : Date.now(),
    startPose: normalizeAuthoringPose(raw.startPose),
    customZones: Array.isArray(raw.customZones)
      ? raw.customZones.map(normalizeNoGoZone).filter(Boolean)
      : [],
    steps,
  };
}

function clonePose(pose) {
  if (!pose) return null;
  return { x: pose.x, y: pose.y, headingDeg: pose.headingDeg };
}

function cloneZones(zones) {
  return Array.isArray(zones) ? zones.map((zone) => ({ ...zone })) : [];
}

function clonePresets(presets) {
  return Array.isArray(presets)
    ? presets.map((preset) => ({
        ...preset,
        folder: preset.folder || "",
        relativePath: preset.relativePath || "",
        startPose: clonePose(preset.startPose),
        customZones: cloneZones(preset.customZones),
        steps: cloneSteps(preset.steps),
      }))
    : [];
}

function serializePose(pose) {
  if (!pose) return null;
  return {
    xMeters: parseFiniteOrNull(pose.x),
    yMeters: parseFiniteOrNull(pose.y),
    headingDeg: parseFiniteOrNull(pose.headingDeg),
  };
}

function serializeNoGoZone(zone) {
  return {
    id: zone.id || "",
    label: zone.label || "",
    xMinMeters: parseFiniteOrNull(zone.xMinMeters),
    yMinMeters: parseFiniteOrNull(zone.yMinMeters),
    xMaxMeters: parseFiniteOrNull(zone.xMaxMeters),
    yMaxMeters: parseFiniteOrNull(zone.yMaxMeters),
    locked: !!zone.locked,
  };
}

function wrapDegrees(value) {
  let wrapped = Number.isFinite(value) ? value : 0;
  while (wrapped > 180) wrapped -= 360;
  while (wrapped <= -180) wrapped += 360;
  return wrapped;
}

function clampToFieldBounds(key, value) {
  if (key === "xMeters") {
    return clamp(value, FIELD_MARGIN_METERS, SAFE_AUTO_MAX_X_METERS);
  }
  if (key === "yMeters") {
    return clamp(value, FIELD_MARGIN_METERS, FIELD_WIDTH_METERS - FIELD_MARGIN_METERS);
  }
  return wrapDegrees(value);
}

function getQueueStartPose() {
  return clonePose(queueModel.startPose);
}

function getPreviousStepPose(stepOrIndex) {
  const index =
    typeof stepOrIndex === "number" ? stepOrIndex : queueModel.steps.findIndex((entry) => entry === stepOrIndex);
  if (index <= 0) {
    return getQueueStartPose();
  }
  return resolveStepPose(queueModel.steps[index - 1]);
}

function authoringPoseToRuntimePose(pose) {
  if (!pose) return null;
  return {
    x: pose.x,
    y: pose.y,
    theta: degreesToRadians(Number.isFinite(pose.headingDeg) ? pose.headingDeg : 0),
  };
}

function getStepRouteWaypoints(index, step) {
  const normalizedStep = normalizeQueueStep(step);
  if (Array.isArray(normalizedStep.routeWaypoints) && normalizedStep.routeWaypoints.length > 0) {
    return normalizedStep.routeWaypoints.map(clonePose);
  }
  return computeRouteWaypoints(getPreviousStepPose(index), normalizedStep).map(clonePose);
}

function computeRouteWaypoints(startPose, step) {
  const finalPose = resolveStepPose(step);
  if (!finalPose) {
    return [];
  }
  const rawStartPose = startPose || getQueueStartPose();
  const initialPose =
    rawStartPose && Number.isFinite(rawStartPose.headingDeg)
      ? authoringPoseToRuntimePose(rawStartPose)
      : rawStartPose;
  if (!initialPose) {
    return [];
  }
  const checkpoints = [];
  const spot = getSpotById(step.spotId);
  if (spot && Number.isFinite(spot.approachDistanceMeters) && spot.approachDistanceMeters > 0.01) {
    checkpoints.push(buildApproachPose(finalPose, spot.approachDistanceMeters));
  }
  checkpoints.push(finalPose);

  const fullPath = [initialPose];
  let cursor = initialPose;
  for (const checkpoint of checkpoints) {
    const segment = computeSegmentPath(cursor, checkpoint, getAllNoGoZones());
    if (!segment || segment.length < 2) {
      return [];
    }
    segment.slice(1).forEach((pose) => fullPath.push(pose));
    cursor = checkpoint;
  }

  return dedupePath(fullPath).slice(1, -1).map(cloneRuntimePoseAsAuthoringPose);
}

function buildApproachPose(finalPose, distanceMeters) {
  const heading = finalPose.theta ?? 0;
  return {
    x: clamp(finalPose.x + Math.cos(heading) * distanceMeters, FIELD_MARGIN_METERS, SAFE_AUTO_MAX_X_METERS),
    y: clamp(
      finalPose.y + Math.sin(heading) * distanceMeters,
      FIELD_MARGIN_METERS,
      FIELD_WIDTH_METERS - FIELD_MARGIN_METERS
    ),
    theta: heading,
  };
}

function cloneRuntimePoseAsAuthoringPose(pose) {
  return {
    x: pose.x,
    y: pose.y,
    headingDeg: wrapDegrees(((pose.theta ?? 0) * 180) / Math.PI),
  };
}

function computeSegmentPath(startPose, endPose, zones) {
  if (segmentIsSafe(startPose, endPose, zones)) {
    return [startPose, endPose];
  }

  const nodes = [
    { x: startPose.x, y: startPose.y, theta: startPose.theta ?? 0 },
    { x: endPose.x, y: endPose.y, theta: endPose.theta ?? 0 },
  ];
  zones.forEach((zone) => {
    const candidates = [
      { x: zone.xMinMeters - ROUTE_CORNER_MARGIN_METERS, y: zone.yMinMeters - ROUTE_CORNER_MARGIN_METERS },
      { x: zone.xMinMeters - ROUTE_CORNER_MARGIN_METERS, y: zone.yMaxMeters + ROUTE_CORNER_MARGIN_METERS },
      { x: zone.xMaxMeters + ROUTE_CORNER_MARGIN_METERS, y: zone.yMinMeters - ROUTE_CORNER_MARGIN_METERS },
      { x: zone.xMaxMeters + ROUTE_CORNER_MARGIN_METERS, y: zone.yMaxMeters + ROUTE_CORNER_MARGIN_METERS },
    ];
    candidates.forEach((candidate) => {
      const pose = {
        x: clamp(candidate.x, FIELD_MARGIN_METERS, SAFE_AUTO_MAX_X_METERS),
        y: clamp(candidate.y, FIELD_MARGIN_METERS, FIELD_WIDTH_METERS - FIELD_MARGIN_METERS),
        theta: endPose.theta ?? 0,
      };
      if (isBlueReferencePoseSafe(pose, zones)) {
        nodes.push(pose);
      }
    });
  });

  const distances = Array(nodes.length).fill(Number.POSITIVE_INFINITY);
  const previous = Array(nodes.length).fill(-1);
  const visited = Array(nodes.length).fill(false);
  distances[0] = 0;

  for (let iteration = 0; iteration < nodes.length; iteration += 1) {
    let currentIndex = -1;
    let bestDistance = Number.POSITIVE_INFINITY;
    for (let i = 0; i < nodes.length; i += 1) {
      if (!visited[i] && distances[i] < bestDistance) {
        bestDistance = distances[i];
        currentIndex = i;
      }
    }
    if (currentIndex < 0) break;
    if (currentIndex === 1) break;
    visited[currentIndex] = true;
    for (let neighbor = 0; neighbor < nodes.length; neighbor += 1) {
      if (neighbor === currentIndex || visited[neighbor]) continue;
      if (!segmentIsSafe(nodes[currentIndex], nodes[neighbor], zones)) continue;
      const nextDistance =
        distances[currentIndex] + distanceBetween(nodes[currentIndex], nodes[neighbor]);
      if (nextDistance < distances[neighbor]) {
        distances[neighbor] = nextDistance;
        previous[neighbor] = currentIndex;
      }
    }
  }

  if (!Number.isFinite(distances[1])) {
    return null;
  }

  const indices = [];
  let cursor = 1;
  while (cursor >= 0) {
    indices.push(cursor);
    cursor = previous[cursor];
  }
  indices.reverse();
  return indices.map((index) => ({
    x: nodes[index].x,
    y: nodes[index].y,
    theta: index === indices.length - 1 ? endPose.theta ?? 0 : startPose.theta ?? 0,
  }));
}

function dedupePath(path) {
  const deduped = [];
  path.forEach((pose) => {
    if (!pose) return;
    const last = deduped[deduped.length - 1];
    if (!last || distanceBetween(last, pose) > 0.02) {
      deduped.push(pose);
    }
  });
  return deduped;
}

function distanceBetween(a, b) {
  return Math.hypot((a.x ?? 0) - (b.x ?? 0), (a.y ?? 0) - (b.y ?? 0));
}

function segmentIsSafe(startPose, endPose, zones) {
  if (!isBlueReferencePoseSafe(startPose, zones) || !isBlueReferencePoseSafe(endPose, zones)) {
    return false;
  }
  return !zones.some((zone) => segmentHitsZone(startPose, endPose, zone));
}

function isBlueReferencePoseSafe(pose, zones) {
  if (!pose) return false;
  if (
    pose.x < FIELD_MARGIN_METERS ||
    pose.x > SAFE_AUTO_MAX_X_METERS ||
    pose.y < FIELD_MARGIN_METERS ||
    pose.y > FIELD_WIDTH_METERS - FIELD_MARGIN_METERS
  ) {
    return false;
  }
  return !zones.some((zone) => pointInZone(pose.x, pose.y, zone));
}

function pointInZone(x, y, zone) {
  return (
    x >= zone.xMinMeters - 1e-6 &&
    x <= zone.xMaxMeters + 1e-6 &&
    y >= zone.yMinMeters - 1e-6 &&
    y <= zone.yMaxMeters + 1e-6
  );
}

function segmentHitsZone(startPose, endPose, zone) {
  if (pointInZone(startPose.x, startPose.y, zone) || pointInZone(endPose.x, endPose.y, zone)) {
    return true;
  }
  return (
    lineSegmentsIntersect(startPose, endPose, { x: zone.xMinMeters, y: zone.yMinMeters }, { x: zone.xMaxMeters, y: zone.yMinMeters }) ||
    lineSegmentsIntersect(startPose, endPose, { x: zone.xMaxMeters, y: zone.yMinMeters }, { x: zone.xMaxMeters, y: zone.yMaxMeters }) ||
    lineSegmentsIntersect(startPose, endPose, { x: zone.xMaxMeters, y: zone.yMaxMeters }, { x: zone.xMinMeters, y: zone.yMaxMeters }) ||
    lineSegmentsIntersect(startPose, endPose, { x: zone.xMinMeters, y: zone.yMaxMeters }, { x: zone.xMinMeters, y: zone.yMinMeters })
  );
}

function lineSegmentsIntersect(a, b, c, d) {
  const abC = cross2d(a, b, c);
  const abD = cross2d(a, b, d);
  const cdA = cross2d(c, d, a);
  const cdB = cross2d(c, d, b);
  if ((abC > 0 && abD < 0 || abC < 0 && abD > 0) && (cdA > 0 && cdB < 0 || cdA < 0 && cdB > 0)) {
    return true;
  }
  if (Math.abs(abC) <= 1e-6 && pointOnSegment(a, b, c)) return true;
  if (Math.abs(abD) <= 1e-6 && pointOnSegment(a, b, d)) return true;
  if (Math.abs(cdA) <= 1e-6 && pointOnSegment(c, d, a)) return true;
  return Math.abs(cdB) <= 1e-6 && pointOnSegment(c, d, b);
}

function cross2d(a, b, c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

function pointOnSegment(a, b, p) {
  return (
    p.x >= Math.min(a.x, b.x) - 1e-6 &&
    p.x <= Math.max(a.x, b.x) + 1e-6 &&
    p.y >= Math.min(a.y, b.y) - 1e-6 &&
    p.y <= Math.max(a.y, b.y) + 1e-6
  );
}

function toBlueReferencePose(pose) {
  if (!pose) return null;
  if (getDisplayAlliance() !== "RED") {
    return { x: pose.x, y: pose.y, theta: pose.theta ?? 0 };
  }
  return {
    x: FIELD_LENGTH_METERS - pose.x,
    y: pose.y,
    theta: degreesToRadians(wrapDegrees(180 - ((pose.theta ?? 0) * 180) / Math.PI)),
  };
}

function getAllSpots() {
  return [...layoutState.spots, ...queueModel.customSpots];
}

function getSpotById(spotId) {
  return getAllSpots().find((spot) => spot.id === spotId) || null;
}

function normalizeQueueStep(step) {
  const normalized = {
    type: parseString(step?.type) || "PATH",
    spotId: parseString(step?.spotId) || "",
    label: sanitizePresetName(step?.label) || "",
    commandName: sanitizePresetName(step?.commandName) || "",
    requestedState: normalizeRequestedState(step?.requestedState),
    status: normalizeQueueStatus(step?.status),
    group: normalizeSpotGroup(step?.group),
    xMeters: parseFiniteOrNull(step?.xMeters),
    yMeters: parseFiniteOrNull(step?.yMeters),
    headingDeg: parseFiniteOrNull(step?.headingDeg),
    waitSeconds: parseFiniteOrNull(step?.waitSeconds),
    routeWaypoints: Array.isArray(step?.routeWaypoints)
      ? step.routeWaypoints.map(normalizeAuthoringPose).filter(Boolean)
      : [],
  };
  return normalized;
}

function stepFromSpot(spot, requestedState) {
  return {
    spotId: spot.id,
    label: spot.label || "",
    requestedState: normalizeRequestedState(requestedState),
    status: "PENDING",
    group: normalizeSpotGroup(spot.group),
    xMeters: parseFiniteOrNull(spot.x),
    yMeters: parseFiniteOrNull(spot.y),
    headingDeg: parseFiniteOrNull(spot.headingDeg),
    routeWaypoints: [],
  };
}

function serializeQueueStep(step, index) {
  const routeWaypoints = getStepRouteWaypoints(index, step);
  return {
    type: step.type || "PATH",
    spotId: step.spotId,
    commandName: step.commandName || "",
    requestedState: step.requestedState || "",
    label: step.label || "",
    group: step.group || "",
    xMeters: parseFiniteOrNull(step.xMeters),
    yMeters: parseFiniteOrNull(step.yMeters),
    headingDeg: parseFiniteOrNull(step.headingDeg),
    waitSeconds: parseFiniteOrNull(step.waitSeconds),
    routeWaypoints: routeWaypoints.map(serializePose),
  };
}

function sanitizePresetName(value) {
  return String(value || "").trim().replace(/\s+/g, " ");
}

function makePresetId() {
  return `preset-${Date.now()}-${Math.floor(Math.random() * 100000)}`;
}

function cloneSteps(steps) {
  return steps.map((step) => ({
    ...step,
    commandName: step.commandName || "",
    requestedState: step.requestedState || "",
    status: "PENDING",
    routeWaypoints: Array.isArray(step.routeWaypoints)
      ? step.routeWaypoints.map(clonePose)
      : [],
  }));
}

function getSpotDisplayName(spot) {
  if (!spot) return "Unknown Spot";
  return spot.label;
}

function getStepActionLabel(step) {
  return parseString(step?.commandName) || normalizeRequestedState(step?.requestedState) || "";
}

function getFieldSpotLabel(spot, includeAlliancePrefix) {
  if (!spot) return "Spot";
  return spot.shortLabel || spot.label;
}

function degreesToRadians(value) {
  return (value * Math.PI) / 180;
}

function drawRobotBox(pose, fieldRect, options = {}) {
  if (!pose) return;
  const center = fieldToCanvas(pose, fieldRect);
  const widthPx = ((ROBOT_LENGTH_METERS * (options.scale || 1)) / FIELD_LENGTH_METERS) * fieldRect.width;
  const heightPx = ((ROBOT_WIDTH_METERS * (options.scale || 1)) / FIELD_WIDTH_METERS) * fieldRect.height;
  const theta = pose.theta ?? 0;
  fieldCtx.save();
  fieldCtx.translate(center.x, center.y);
  fieldCtx.rotate(-theta);
  if (options.dashed) {
    fieldCtx.setLineDash([8, 6]);
  }
  fieldCtx.fillStyle = options.fill || "rgba(255,255,255,0.12)";
  fieldCtx.strokeStyle = options.stroke || "rgba(255,255,255,0.85)";
  fieldCtx.lineWidth = options.lineWidth || 2;
  fieldCtx.beginPath();
  fieldCtx.rect(-widthPx / 2, -heightPx / 2, widthPx, heightPx);
  fieldCtx.fill();
  fieldCtx.stroke();
  fieldCtx.beginPath();
  fieldCtx.moveTo(0, 0);
  fieldCtx.lineTo(widthPx / 2, 0);
  fieldCtx.strokeStyle = options.stroke || "rgba(255,255,255,0.85)";
  fieldCtx.lineWidth = Math.max(1.5, (options.lineWidth || 2) - 0.5);
  fieldCtx.stroke();
  fieldCtx.restore();

  if (options.badgeText) {
    const badgeX = center.x + widthPx * 0.36;
    const badgeY = center.y - heightPx * 0.42;
    fieldCtx.save();
    fieldCtx.beginPath();
    fieldCtx.arc(badgeX, badgeY, 11, 0, Math.PI * 2);
    fieldCtx.fillStyle = options.badgeColor || options.stroke || "rgba(255,209,102,0.95)";
    fieldCtx.fill();
    fieldCtx.fillStyle = "#241800";
    fieldCtx.font = `${Math.max(9, Math.round(10 * (window.devicePixelRatio || 1)))}px sans-serif`;
    fieldCtx.textAlign = "center";
    fieldCtx.textBaseline = "middle";
    fieldCtx.fillText(options.badgeText, badgeX, badgeY + 0.5);
    fieldCtx.restore();
  }

  if (options.label) {
    drawLabel(
      center.x + widthPx * 0.6,
      center.y - heightPx * 0.7,
      options.label,
      options.labelBackground || "rgba(14,17,22,0.82)",
      options.labelColor || "#e8eefc",
      options.stroke || null
    );
  }
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

function drawRing(x, y, r, color) {
  fieldCtx.save();
  fieldCtx.beginPath();
  fieldCtx.arc(x, y, r, 0, Math.PI * 2);
  fieldCtx.strokeStyle = color;
  fieldCtx.lineWidth = 3;
  fieldCtx.setLineDash([8, 8]);
  fieldCtx.stroke();
  fieldCtx.restore();
}

function drawHeading(x, y, thetaRad, color) {
  const len = 22;
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

function drawLabel(x, y, text, backgroundColor, textColor = "#e8eefc", borderColor = null) {
  if (!text) return;
  fieldCtx.save();
  const fontSize = Math.max(10, Math.round(11 * (window.devicePixelRatio || 1)));
  fieldCtx.font = `${fontSize}px sans-serif`;
  const metrics = fieldCtx.measureText(text);
  const paddingX = 6;
  const paddingY = 4;
  const width = metrics.width + paddingX * 2;
  const height = fontSize + paddingY * 2;

  fieldCtx.beginPath();
  roundRect(fieldCtx, x, y - height, width, height, 9);
  fieldCtx.fillStyle = backgroundColor;
  fieldCtx.fill();
  if (borderColor) {
    fieldCtx.strokeStyle = borderColor;
    fieldCtx.lineWidth = 1;
    fieldCtx.stroke();
  }

  fieldCtx.fillStyle = textColor;
  fieldCtx.textAlign = "left";
  fieldCtx.textBaseline = "alphabetic";
  fieldCtx.fillText(text, x + paddingX, y - paddingY);
  fieldCtx.restore();
}

function roundRect(ctx, x, y, width, height, radius) {
  ctx.moveTo(x + radius, y);
  ctx.lineTo(x + width - radius, y);
  ctx.quadraticCurveTo(x + width, y, x + width, y + radius);
  ctx.lineTo(x + width, y + height - radius);
  ctx.quadraticCurveTo(x + width, y + height, x + width - radius, y + height);
  ctx.lineTo(x + radius, y + height);
  ctx.quadraticCurveTo(x, y + height, x, y + height - radius);
  ctx.lineTo(x, y + radius);
  ctx.quadraticCurveTo(x, y, x + radius, y);
}

function setText(el, text) {
  if (!el) return;
  el.innerText = text;
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function round(value, digits) {
  const power = Math.pow(10, digits);
  return Math.round(value * power) / power;
}

function resolvePreviewOverrides(params) {
  const demo = parseString(params.get("demo"))?.toLowerCase() || null;
  if (demo === "endgame") {
    return { dsMode: "TELEOP", matchTime: 29.5, visionPoseVisible: true };
  }

  return {
    dsMode: parseString(params.get("dsMode"))?.toUpperCase() || null,
    matchTime: parseQueryNumber(params.get("matchTime")),
    visionPoseVisible: parseQueryBoolean(params.get("visionPoseVisible")),
  };
}

function parseQueryBoolean(value) {
  const token = parseString(value)?.toLowerCase();
  if (token === null) return null;
  if (token === "true" || token === "1" || token === "yes" || token === "on") return true;
  if (token === "false" || token === "0" || token === "no" || token === "off") return false;
  return null;
}

function parseQueryNumber(value) {
  if (value === null || value === undefined || value === "") return null;
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : null;
}

function resolveNtConnectionParams(params) {
  const host = params.get("ntHost") || params.get("host") || window.location.hostname || "localhost";
  const portRaw = params.get("ntPort") || params.get("port");
  const portParsed = portRaw ? Number(portRaw) : NaN;
  const port = Number.isFinite(portParsed) ? portParsed : defaultNt4Port;
  return { host, port };
}

function showToast(message) {
  if (!ui.toast) return;
  ui.toast.innerText = message;
  ui.toast.classList.add("toast--show");
  window.clearTimeout(ui.toast._hideTimer);
  ui.toast._hideTimer = window.setTimeout(() => {
    ui.toast.classList.remove("toast--show");
  }, 2500);
}
