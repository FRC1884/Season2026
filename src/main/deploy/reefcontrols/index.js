// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import { NT4_Client } from "./NT4.js";

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

const ntClient = new NT4_Client(
  window.location.hostname,
  "ReefControls",
  () => {
    // Topic announce
  },
  () => {
    // Topic unannounce
  },
  (topic, _, value) => {
    // New data
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
    // Disconnected
    document.body.style.backgroundColor = "red";
  }
);

// Start NT connection
window.addEventListener("load", () => {
  ntClient.subscribe(
    [
      toDashboardPrefix + selectedLevelTopicName,
      toDashboardPrefix + l1TopicName,
      toDashboardPrefix + l2TopicName,
      toDashboardPrefix + l3TopicName,
      toDashboardPrefix + l4TopicName,
      toDashboardPrefix + algaeTopicName,
      toDashboardPrefix + coopTopicName,
      toDashboardPrefix + isElimsTopicName,
      toDashboardPrefix + queueStateTopicName,
    ],
    false,
    false,
    0.02
  );

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
});

// ***** STATE CACHE *****

let selectedLevel = 2; // 0 = L2, 1 = L3, 2 = L4 (default to L4)
let l1State = 0; // Count
let l2State = 0; // Bitfield
let l3State = 0; // Bitfield
let l4State = 0; // Bitfield
let algaeState = 0; // Bitfield
let coopState = false; // Boolean
let isElims = false; // Boolean
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
  const levelIndex = getSelectedLevelIndex();

  // Update counter highlight
  Array.from(document.getElementsByClassName("counter-area")).forEach(
    (element, index) => {
      if (index > 0 && levelIndex === index - 1) {
        element.classList.add("active");
      } else {
        element.classList.remove("active");
      }
    }
  );

  // Update background color
  switch (levelIndex) {
    case 0:
      document.body.style.backgroundColor = "#5AB7ED";
      break;
    case 1:
      document.body.style.backgroundColor = "#0F486C";
      break;
    case 2:
      document.body.style.backgroundColor = "#dfbbfc";
      break;
  }

  // Update level counts
  let rpLevelCount = 0;
  Array.from(document.getElementsByClassName("counter")).forEach(
    (element, index) => {
      if (index === 0) {
        element.innerText = l1State;
        if (l1State >= 7) rpLevelCount++;
      } else {
        let count = 0;
        let levelState = [l2State, l3State, l4State][index - 1];
        for (let i = 0; i < 12; i++) {
          if (((1 << i) & levelState) > 0) {
            count++;
          }
        }
        element.innerText = count === 12 ? "\u2705" : count;
        if (count >= 7) rpLevelCount++;
      }
    }
  );

  // Update coral buttons
  Array.from(document.getElementsByClassName("branch")).forEach(
    (element, index) => {
      let levelState = [l2State, l3State, l4State][levelIndex];
      if (((1 << index) & levelState) > 0) {
        element.classList.add("active");
      } else {
        element.classList.remove("active");
      }
    }
  );

  // Update algae buttons
  Array.from(document.getElementsByClassName("algae")).forEach(
    (element, index) => {
      if (((1 << index) & algaeState) > 0) {
        element.classList.add("active");
      } else {
        element.classList.remove("active");
      }
    }
  );

  // Update coop button
  let coopDiv = document.getElementsByClassName("coop")[0];
  if (coopState) {
    coopDiv.classList.add("active");
  } else {
    coopDiv.classList.remove("active");
  }

  // Update RP flag
  document.getElementsByClassName("flag")[0].hidden =
    isElims || rpLevelCount < (coopState ? 3 : 4);

  // Update elims state
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
    manualToggle.innerText = manualEnabled ? "Enable Queue" : "Disable Queue";
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

function sendQueueCommand(command, extras) {
  if (!command) {
    return;
  }
  const payload = Object.assign(
    { command: command.toUpperCase(), at: Date.now() },
    extras || {}
  );
  ntClient.addSample(toRobotPrefix + queueCommandTopicName, JSON.stringify(payload));
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
  const levelTokens = ["L2", "L3", "L4"];
  const levelIndex = getSelectedLevelIndex();
  const level = levelTokens[levelIndex] || "L3";
  sendAddStep({ type: "SOURCE", source: side });
  sendAddStep({ type: "REEF", face, side, level });
}

function initQueueUi() {
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

window.addEventListener("load", () => {
  // Buttons to change selected level
  Array.from(document.getElementsByClassName("counter-area")).forEach(
    (element, index) => {
      if (index > 0) {
        bind(element, () => {
          const newLevel = clampSelectedLevel(index - 1);
          ntClient.addSample(toRobotPrefix + selectedLevelTopicName, newLevel);
          selectedLevel = newLevel;
          updateUI();
        });
      }
    }
  );

  // Coral toggle buttons
  Array.from(document.getElementsByClassName("branch")).forEach(
    (element, index) => {
      bind(element, () => {
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
      });
    }
  );

  // Algae toggle buttons
  Array.from(document.getElementsByClassName("algae")).forEach(
    (element, index) => {
      bind(element, () => {
        ntClient.addSample(
          toRobotPrefix + algaeTopicName,
          algaeState ^ (1 << index)
        );
      });
    }
  );

  // L1 count controls
  bind(document.getElementsByClassName("subtract")[0], () => {
    if (l1State > 0) {
      ntClient.addSample(toRobotPrefix + l1TopicName, l1State - 1);
    }
  });
  bind(document.getElementsByClassName("add")[0], () => {
    ntClient.addSample(toRobotPrefix + l1TopicName, l1State + 1);
  });

  // Coop button
  bind(document.getElementsByClassName("coop")[0], () => {
    ntClient.addSample(toRobotPrefix + coopTopicName, !coopState);
  });

  initQueueUi();
  updateUI();
});

// ***** REEF CANVAS *****

window.addEventListener("load", () => {
  const canvas = document.getElementsByTagName("canvas")[0];
  const context = canvas.getContext("2d");

  let render = () => {
    const devicePixelRatio = window.devicePixelRatio;
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    canvas.width = width * devicePixelRatio;
    canvas.height = height * devicePixelRatio;
    context.scale(devicePixelRatio, devicePixelRatio);
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
});
