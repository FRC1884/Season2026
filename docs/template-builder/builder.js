const defaultTemplate = {
  templateVersion: 1,
  templateId: "2025-default",
  metadata: {
    title: "Reef Controls",
    subtitle: "FRC 1884",
  },
  theme: {
    levelBackgrounds: ["#5AB7ED", "#0F486C", "#dfbbfc"],
    disconnectedBackground: "#8b1e3f",
  },
  overview: {
    counters: [
      { id: "level1", label: "L1", source: "Level1" },
      { id: "level2", label: "L2", source: "Level2", selectsLevel: 0 },
      { id: "level3", label: "L3", source: "Level3", selectsLevel: 1 },
      { id: "level4", label: "L4", source: "Level4", selectsLevel: 2 },
    ],
  },
  reef: {
    branchImage: "coral.png",
    branchSizeVh: 12,
    algaeImage: "algae.png",
    algaeSizeVh: 14,
    flagEmoji: "\uD83C\uDFC1",
    branchNodes: [
      { x: 60, y: 90, face: 1, side: "LEFT" },
      { x: 40, y: 90, face: 1, side: "RIGHT" },
      { x: 20, y: 80, face: 2, side: "LEFT" },
      { x: 10, y: 62, face: 2, side: "RIGHT" },
      { x: 10, y: 38, face: 3, side: "LEFT" },
      { x: 20, y: 20, face: 3, side: "RIGHT" },
      { x: 40, y: 10, face: 4, side: "LEFT" },
      { x: 60, y: 10, face: 4, side: "RIGHT" },
      { x: 80, y: 20, face: 5, side: "LEFT" },
      { x: 90, y: 38, face: 5, side: "RIGHT" },
      { x: 90, y: 62, face: 6, side: "LEFT" },
      { x: 80, y: 80, face: 6, side: "RIGHT" },
    ],
    algaeNodes: [
      { x: 50, y: 70 },
      { x: 32, y: 60 },
      { x: 32, y: 40 },
      { x: 50, y: 30 },
      { x: 68, y: 40 },
      { x: 68, y: 60 },
    ],
  },
  controls: [
    {
      id: "l1-subtract",
      label: "-1 L1",
      icon: "\u2796",
      style: {
        background: "rgba(255,255,255,0.04)",
        textColor: "#f4fbff",
      },
      action: { type: "topic-delta", target: "Level1", delta: -1, min: 0 },
    },
    {
      id: "l1-add",
      label: "+1 L1",
      icon: "\u2795",
      style: {
        background: "rgba(255,255,255,0.04)",
        textColor: "#f4fbff",
      },
      action: { type: "topic-delta", target: "Level1", delta: 1 },
    },
    {
      id: "coop-toggle",
      label: "Co-Op",
      icon: "\uD83E\uDD1D",
      classes: ["coop"],
      style: {
        background: "rgba(79,208,255,0.12)",
        textColor: "#f4fbff",
      },
      action: { type: "topic-toggle", target: "Coop" },
    },
  ],
  queue: {
    title: "Tablet Queue",
    subtitle: "Plan the robot's next moves",
    hint: "Tap reef nodes to enqueue a source \u2192 reef pair for the selected level, then hit Start. Drag items to reorder them, tap to remove, and toggle manual mode when the driver needs hands-on control.",
    manualToggle: {
      queueEnabledLabel: "Disable Queue",
      manualEnabledLabel: "Enable Queue",
    },
    commands: [
      { label: "Start", command: "start" },
      { label: "Pause", command: "stop" },
      { label: "Skip", command: "skip" },
      { label: "Reset", command: "reset" },
    ],
  },
};

let state = clone(defaultTemplate);

const sectionsEl = document.getElementById("builder-sections");
const outputEl = document.getElementById("template-output");
const copyBtn = document.getElementById("copy-json");
const downloadBtn = document.getElementById("download-json");
const resetBtn = document.getElementById("reset-template");

renderAll();

sectionsEl.addEventListener("input", handleFieldChange);
sectionsEl.addEventListener("change", handleFieldChange);
sectionsEl.addEventListener("click", handleActionClick);
copyBtn.addEventListener("click", handleCopy);
downloadBtn.addEventListener("click", handleDownload);
resetBtn.addEventListener("click", handleReset);

function handleFieldChange(event) {
  const target = event.target;
  const path = target.dataset.path;
  if (!path) {
    return;
  }
  const value = readInputValue(target);
  setByPath(state, path, value);
  if (target.dataset.rerender === "true") {
    renderAll();
  } else {
    updateOutput();
  }
}

function handleActionClick(event) {
  const action = event.target.dataset.action;
  if (!action) {
    return;
  }
  event.preventDefault();
  switch (action) {
    case "add-counter":
      ensureArray(state.overview, "counters");
      state.overview.counters.push(createCounter());
      break;
    case "remove-counter":
      removeByIndex(state.overview.counters, event.target.dataset.index);
      break;
    case "add-branch":
      ensureArray(state.reef, "branchNodes");
      state.reef.branchNodes.push(createBranchNode());
      break;
    case "remove-branch":
      removeByIndex(state.reef.branchNodes, event.target.dataset.index);
      break;
    case "add-algae":
      ensureArray(state.reef, "algaeNodes");
      state.reef.algaeNodes.push(createAlgaeNode());
      break;
    case "remove-algae":
      removeByIndex(state.reef.algaeNodes, event.target.dataset.index);
      break;
    case "add-control":
      ensureArray(state, "controls");
      state.controls.push(createControl());
      break;
    case "remove-control":
      removeByIndex(state.controls, event.target.dataset.index);
      break;
    case "add-queue-command":
      ensureArray(state.queue, "commands");
      state.queue.commands.push(createQueueCommand());
      break;
    case "remove-queue-command":
      removeByIndex(state.queue.commands, event.target.dataset.index);
      break;
    case "add-level-color":
      ensureArray(state.theme, "levelBackgrounds");
      state.theme.levelBackgrounds.push("#123456");
      break;
    case "remove-level-color":
      removeByIndex(state.theme.levelBackgrounds, event.target.dataset.index);
      break;
    default:
      break;
  }
  renderAll();
}

function handleCopy() {
  const data = JSON.stringify(state, null, 2);
  if (navigator.clipboard && navigator.clipboard.writeText) {
    navigator.clipboard.writeText(data).catch(() => {});
  }
}

function handleDownload() {
  const data = JSON.stringify(state, null, 2);
  const blob = new Blob([data], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const filename = `${state.templateId || "reef-template"}.json`;
  const link = document.createElement("a");
  link.href = url;
  link.download = filename;
  document.body.appendChild(link);
  link.click();
  document.body.removeChild(link);
  URL.revokeObjectURL(url);
}

function handleReset() {
  if (confirm("Reset the template to defaults?")) {
    state = clone(defaultTemplate);
    renderAll();
  }
}

function renderAll() {
  sectionsEl.innerHTML = "";
  sectionsEl.appendChild(createSection("Metadata", renderMetadataFields()));
  sectionsEl.appendChild(createSection("Theme", renderThemeFields()));
  sectionsEl.appendChild(createSection("Counters", renderCountersFields()));
  sectionsEl.appendChild(createSection("Reef Layout", renderReefFields()));
  sectionsEl.appendChild(createSection("Controls", renderControlsFields()));
  sectionsEl.appendChild(createSection("Queue", renderQueueFields()));
  updateOutput();
}

function updateOutput() {
  outputEl.value = JSON.stringify(state, null, 2);
}

function createSection(title, bodyHtml) {
  const section = document.createElement("section");
  section.className = "form-section";
  section.innerHTML = `<h2>${title}</h2>${bodyHtml}`;
  return section;
}

function renderMetadataFields() {
  if (!state.metadata) {
    state.metadata = {};
  }
  return `
    <div class="form-grid">
      ${renderInput("Template ID", "templateId", state.templateId || "", { placeholder: "2025-default" })}
      ${renderInput("Template Version", "templateVersion", state.templateVersion || 1, {
        type: "number",
        valueType: "int",
      })}
      ${renderInput("Title", "metadata.title", state.metadata?.title || "")}
      ${renderInput("Subtitle", "metadata.subtitle", state.metadata?.subtitle || "")}
      ${renderInput("Favicon Path", "metadata.favicon", state.metadata?.favicon || "")}
    </div>
  `;
}

function renderThemeFields() {
  if (!state.theme) {
    state.theme = {};
  }
  ensureArray(state.theme, "levelBackgrounds");
  const colors = state.theme.levelBackgrounds;
  const colorRows = colors
    .map(
      (color, index) => `
        <div class="theme-color-row">
          <input type="color" data-path="theme.levelBackgrounds[${index}]" value="${escapeAttr(
            color || "#000000"
          )}" />
          <button type="button" data-action="remove-level-color" data-index="${index}">&times;</button>
        </div>
      `
    )
    .join("");
  return `
    <div class="form-grid">
      ${renderInput(
        "Disconnected Background",
        "theme.disconnectedBackground",
        state.theme?.disconnectedBackground || "#8b1e3f",
        { type: "color" }
      )}
    </div>
    <div class="theme-color-list">
      <div class="theme-color-header">Level backgrounds</div>
      ${colorRows || '<p class="empty-row">No level colors configured.</p>'}
      <button type="button" class="add-row" data-action="add-level-color">Add Color</button>
    </div>
  `;
}

function renderCountersFields() {
  if (!state.overview) {
    state.overview = {};
  }
  ensureArray(state.overview, "counters");
  const rows = state.overview.counters
    .map((counter, index) => `
      <tr>
        <td>${renderTableInput(`overview.counters[${index}].label`, counter.label || "")}</td>
        <td>${renderSelect(
          `overview.counters[${index}].source`,
          counter.source || "Level1",
          ["Level1", "Level2", "Level3", "Level4"]
        )}</td>
        <td>${renderSelect(
          `overview.counters[${index}].selectsLevel`,
          counter.selectsLevel != null ? counter.selectsLevel : "-1",
          [
            { value: "-1", label: "None" },
            { value: "0", label: "L2" },
            { value: "1", label: "L3" },
            { value: "2", label: "L4" },
          ],
          { valueType: "optional-int" }
        )}</td>
        <td>
          <div class="table-actions">
            <button type="button" data-action="remove-counter" data-index="${index}">Remove</button>
          </div>
        </td>
      </tr>
    `)
    .join("");
  return `
    <table class="section-table">
      <thead>
        <tr>
          <th>Label</th>
          <th>Source</th>
          <th>Select Level</th>
          <th></th>
        </tr>
      </thead>
      <tbody>
        ${rows || '<tr><td colspan="4">No counters configured.</td></tr>'}
      </tbody>
    </table>
    <button type="button" class="add-row" data-action="add-counter">Add Counter</button>
  `;
}

function renderReefFields() {
  if (!state.reef) {
    state.reef = {};
  }
  ensureArray(state.reef, "branchNodes");
  ensureArray(state.reef, "algaeNodes");
  const general = `
    <div class="form-grid">
      ${renderInput("Branch Image", "reef.branchImage", state.reef?.branchImage || "coral.png")}
      ${renderInput("Branch Size (vh)", "reef.branchSizeVh", state.reef?.branchSizeVh || 12, {
        type: "number",
        valueType: "number",
      })}
      ${renderInput("Algae Image", "reef.algaeImage", state.reef?.algaeImage || "algae.png")}
      ${renderInput("Algae Size (vh)", "reef.algaeSizeVh", state.reef?.algaeSizeVh || 14, {
        type: "number",
        valueType: "number",
      })}
      ${renderInput("Flag Emoji", "reef.flagEmoji", state.reef?.flagEmoji || "\uD83C\uDFC1")}
    </div>
  `;
  const branchRows = state.reef.branchNodes
    .map((node, index) => `
      <tr>
        <td>${renderTableInput(`reef.branchNodes[${index}].face`, node.face || 1, {
          type: "number",
          valueType: "int",
        })}</td>
        <td>${renderSelect(
          `reef.branchNodes[${index}].side`,
          node.side || "LEFT",
          ["LEFT", "RIGHT"]
        )}</td>
        <td>${renderTableInput(`reef.branchNodes[${index}].x`, node.x ?? 50, {
          type: "number",
          valueType: "number",
        })}</td>
        <td>${renderTableInput(`reef.branchNodes[${index}].y`, node.y ?? 50, {
          type: "number",
          valueType: "number",
        })}</td>
        <td>${renderTableInput(`reef.branchNodes[${index}].sizeVh`, node.sizeVh || "", {
          type: "number",
          valueType: "number",
        })}</td>
        <td>
          <button type="button" data-action="remove-branch" data-index="${index}">Remove</button>
        </td>
      </tr>
    `)
    .join("");

  const algaeRows = state.reef.algaeNodes
    .map((node, index) => `
      <tr>
        <td>${renderTableInput(`reef.algaeNodes[${index}].x`, node.x ?? 50, {
          type: "number",
          valueType: "number",
        })}</td>
        <td>${renderTableInput(`reef.algaeNodes[${index}].y`, node.y ?? 50, {
          type: "number",
          valueType: "number",
        })}</td>
        <td>${renderTableInput(`reef.algaeNodes[${index}].sizeVh`, node.sizeVh || "", {
          type: "number",
          valueType: "number",
        })}</td>
        <td>
          <button type="button" data-action="remove-algae" data-index="${index}">Remove</button>
        </td>
      </tr>
    `)
    .join("");

  return `
    ${general}
    <div class="table-wrapper">
      <div class="table-title">Branch Nodes</div>
      <table class="section-table">
        <thead>
          <tr>
            <th>Face</th>
            <th>Side</th>
            <th>X%</th>
            <th>Y%</th>
            <th>Size (vh)</th>
            <th></th>
          </tr>
        </thead>
        <tbody>
          ${branchRows || '<tr><td colspan="6">No branch nodes.</td></tr>'}
        </tbody>
      </table>
      <button type="button" class="add-row" data-action="add-branch">Add Branch Node</button>
    </div>
    <div class="table-wrapper">
      <div class="table-title">Algae Nodes</div>
      <table class="section-table">
        <thead>
          <tr>
            <th>X%</th>
            <th>Y%</th>
            <th>Size (vh)</th>
            <th></th>
          </tr>
        </thead>
        <tbody>
          ${algaeRows || '<tr><td colspan="4">No algae nodes.</td></tr>'}
        </tbody>
      </table>
      <button type="button" class="add-row" data-action="add-algae">Add Algae Node</button>
    </div>
  `;
}

function renderControlsFields() {
  ensureArray(state, "controls");
  const rows = state.controls
    .map((control, index) => {
      const classes = Array.isArray(control.classes) ? control.classes.join(", ") : "";
      const action = control.action || { type: "topic-delta" };
      return `
        <tr>
          <td>${renderTableInput(`controls[${index}].label`, control.label || "")}</td>
          <td>${renderTableInput(`controls[${index}].icon`, control.icon || "")}</td>
          <td>${renderSelect(
            `controls[${index}].iconType`,
            control.iconType || "text",
            [
              { value: "text", label: "Text" },
              { value: "image", label: "Image" },
            ]
          )}</td>
          <td>${renderTableInput(`controls[${index}].classes`, classes, { valueType: "csv" })}</td>
          <td>${renderStyleFields(control, index)}</td>
          <td>${renderSelect(
            `controls[${index}].action.type`,
            action.type || "topic-delta",
            [
              { value: "topic-delta", label: "Topic Delta" },
              { value: "topic-toggle", label: "Topic Toggle" },
              { value: "topic-set", label: "Topic Set" },
              { value: "queue-command", label: "Queue Command" },
            ],
            { rerender: true }
          )}</td>
          <td>${renderActionFields(action, index)}</td>
          <td>
            <button type="button" data-action="remove-control" data-index="${index}">Remove</button>
          </td>
        </tr>
      `;
    })
    .join("");
  return `
    <table class="section-table">
      <thead>
        <tr>
          <th>Label</th>
          <th>Icon</th>
          <th>Icon Type</th>
          <th>Classes</th>
          <th>Styles</th>
          <th>Action</th>
          <th>Action Options</th>
          <th></th>
        </tr>
      </thead>
      <tbody>
        ${rows || '<tr><td colspan="7">No controls configured.</td></tr>'}
      </tbody>
    </table>
    <button type="button" class="add-row" data-action="add-control">Add Control</button>
  `;
}

function renderStyleFields(control, index) {
  const background = control.style?.background || "";
  const textColor = control.style?.textColor || "";
  const borderColor = control.style?.borderColor || "";
  return `
    <div class="action-field-grid">
      ${renderTableInput(`controls[${index}].style.background`, background, {
        placeholder: "Background CSS",
      })}
      ${renderTableInput(`controls[${index}].style.textColor`, textColor, {
        placeholder: "Text Color",
      })}
      ${renderTableInput(`controls[${index}].style.borderColor`, borderColor, {
        placeholder: "Border Color",
      })}
    </div>
  `;
}

function renderActionFields(action, index) {
  const baseTarget = renderSelect(
    `controls[${index}].action.target`,
    action.target || "Level1",
    ["Level1", "Level2", "Level3", "Level4", "Coop", "SelectedLevel"]
  );
  const topicOverride = renderTableInput(
    `controls[${index}].action.topic`,
    action.topic || "",
    { placeholder: "Override topic name" }
  );
  if (action.type === "topic-delta") {
    return `
      <div class="action-field-grid">
        ${baseTarget}
        ${topicOverride}
        ${renderTableInput(`controls[${index}].action.delta`, action.delta ?? 1, {
          type: "number",
          valueType: "number",
        })}
        ${renderTableInput(`controls[${index}].action.min`, action.min ?? "", {
          type: "number",
          valueType: "number",
        })}
        ${renderTableInput(`controls[${index}].action.max`, action.max ?? "", {
          type: "number",
          valueType: "number",
        })}
      </div>
    `;
  }
  if (action.type === "topic-toggle") {
    return `
      <div class="action-field-grid">
        ${baseTarget}
        ${topicOverride}
      </div>
    `;
  }
  if (action.type === "topic-set") {
    return `
      <div class="action-field-grid">
        ${baseTarget}
        ${topicOverride}
        ${renderTableInput(`controls[${index}].action.value`, formatLiteral(action.value), {
          placeholder: "Value (auto-detected)",
          valueType: "literal",
        })}
      </div>
    `;
  }
  if (action.type === "queue-command") {
    return renderTableInput(
      `controls[${index}].action.command`,
      action.command || "start",
      { placeholder: "start" }
    );
  }
  return baseTarget;
}

function renderQueueFields() {
  if (!state.queue) {
    state.queue = {};
  }
  const queue = state.queue;
  const general = `
    <div class="form-grid">
      ${renderInput("Title", "queue.title", queue.title || "Tablet Queue")}
      ${renderInput("Subtitle", "queue.subtitle", queue.subtitle || "Plan the robot's next moves")}
      ${renderTextarea("Hint", "queue.hint", queue.hint || "")}
      ${renderInput(
        "Queue Enabled Label",
        "queue.manualToggle.queueEnabledLabel",
        queue.manualToggle?.queueEnabledLabel || "Disable Queue"
      )}
      ${renderInput(
        "Manual Enabled Label",
        "queue.manualToggle.manualEnabledLabel",
        queue.manualToggle?.manualEnabledLabel || "Enable Queue"
      )}
    </div>
  `;
  ensureArray(queue, "commands");
  const rows = queue.commands
    .map((command, index) => `
      <tr>
        <td>${renderTableInput(`queue.commands[${index}].label`, command.label || "")}</td>
        <td>${renderTableInput(`queue.commands[${index}].command`, command.command || "")}</td>
        <td>
          <button type="button" data-action="remove-queue-command" data-index="${index}">Remove</button>
        </td>
      </tr>
    `)
    .join("");
  return `
    ${general}
    <table class="section-table">
      <thead>
        <tr>
          <th>Label</th>
          <th>Command</th>
          <th></th>
        </tr>
      </thead>
      <tbody>
        ${rows || '<tr><td colspan="3">No queue commands configured.</td></tr>'}
      </tbody>
    </table>
    <button type="button" class="add-row" data-action="add-queue-command">Add Queue Command</button>
  `;
}

function renderInput(label, path, value, options = {}) {
  return `
    <label class="form-field">
      <span>${label}</span>
      <input ${inputAttributes(path, value, options)} />
    </label>
  `;
}

function renderTextarea(label, path, value, options = {}) {
  return `
    <label class="form-field">
      <span>${label}</span>
      <textarea ${inputAttributes(path, value, { ...options, textarea: true })}>${escapeAttr(
        value || ""
      )}</textarea>
    </label>
  `;
}

function renderTableInput(path, value, options = {}) {
  return `<input ${inputAttributes(path, value, options)} />`;
}

function renderSelect(path, value, choices, options = {}) {
  const normalized = choices.map((choice) =>
    typeof choice === "string" ? { value: choice, label: choice } : choice
  );
  const selected = value != null ? String(value) : "";
  const attrs = inputAttributes(path, selected, { ...options, isSelect: true });
  const optionsHtml = normalized
    .map((choice) => {
      const isSelected = choice.value != null && String(choice.value) === selected;
      return `<option value="${escapeAttr(choice.value)}"${isSelected ? " selected" : ""}>${escapeAttr(
        choice.label
      )}</option>`;
    })
    .join("");
  return `<select ${attrs}>${optionsHtml}</select>`;
}

function inputAttributes(path, value, options = {}) {
  const attrs = [
    `data-path="${path}"`,
    options.placeholder ? `placeholder="${escapeAttr(options.placeholder)}"` : "",
    options.type ? `type="${options.type}"` : options.textarea || options.isSelect ? "" : "type=\"text\"",
    options.valueType ? `data-value-type="${options.valueType}"` : "",
    options.rerender ? `data-rerender="true"` : "",
  ].filter(Boolean);
  if (options.type === "checkbox") {
    if (value) {
      attrs.push("checked");
    }
  } else if (options.type === "color") {
    attrs.push(`value="${escapeAttr(value || "#000000")}"`);
  } else if (!options.textarea && !options.isSelect) {
    attrs.push(`value="${escapeAttr(value ?? "")}"`);
  }
  return attrs.join(" ");
}

function readInputValue(target) {
  const type = target.dataset.valueType;
  if (type === "boolean") {
    return target.checked;
  }
  if (type === "csv") {
    const parts = target.value
      .split(",")
      .map((part) => part.trim())
      .filter((part) => part.length > 0);
    return parts;
  }
  if (type === "optional-int") {
    const parsed = parseInt(target.value, 10);
    return isNaN(parsed) || parsed < 0 ? undefined : parsed;
  }
  if (type === "int") {
    if (target.value === "") {
      return undefined;
    }
    const parsed = parseInt(target.value, 10);
    return isNaN(parsed) ? 0 : parsed;
  }
  if (type === "number") {
    if (target.value === "") {
      return undefined;
    }
    const parsed = parseFloat(target.value);
    return isNaN(parsed) ? 0 : parsed;
  }
  if (type === "literal") {
    return parseLiteral(target.value);
  }
  if (target.type === "number") {
    if (target.value === "") {
      return undefined;
    }
    const parsed = parseFloat(target.value);
    return isNaN(parsed) ? 0 : parsed;
  }
  return target.value;
}

function parseLiteral(value) {
  const trimmed = String(value || "").trim();
  if (trimmed === "") {
    return "";
  }
  if (trimmed === "true" || trimmed === "false") {
    return trimmed === "true";
  }
  const numeric = Number(trimmed);
  if (!Number.isNaN(numeric)) {
    return numeric;
  }
  try {
    return JSON.parse(trimmed);
  } catch {
    return trimmed;
  }
}

function setByPath(obj, path, value) {
  const segments = pathToSegments(path);
  if (segments.length === 0) {
    return;
  }
  let current = obj;
  for (let i = 0; i < segments.length - 1; i++) {
    const key = segments[i];
    if (current[key] == null) {
      current[key] = typeof segments[i + 1] === "number" ? [] : {};
    }
    current = current[key];
  }
  const last = segments[segments.length - 1];
  if (value === undefined) {
    if (Array.isArray(current)) {
      current.splice(last, 1);
    } else {
      delete current[last];
    }
  } else {
    current[last] = value;
  }
}

function pathToSegments(path) {
  return path
    .replace(/\[(\d+)\]/g, ".$1")
    .split(".")
    .filter((segment) => segment.length > 0)
    .map((segment) => {
      const numeric = Number(segment);
      return Number.isInteger(numeric) && String(numeric) === segment ? numeric : segment;
    });
}

function escapeAttr(value) {
  return String(value ?? "")
    .replace(/&/g, "&amp;")
    .replace(/"/g, "&quot;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;");
}

function removeByIndex(list, index) {
  const position = Number(index);
  if (!Array.isArray(list) || isNaN(position)) {
    return;
  }
  list.splice(position, 1);
}

function createCounter() {
  return { label: "New Counter", source: "Level1" };
}

function createBranchNode() {
  return { face: 1, side: "LEFT", x: 50, y: 50 };
}

function createAlgaeNode() {
  return { x: 50, y: 50 };
}

function createControl() {
  return {
    label: "Button",
    icon: "+",
    action: { type: "topic-delta", target: "Level1", delta: 1 },
  };
}

function createQueueCommand() {
  return { label: "Command", command: "start" };
}

function clone(value) {
  if (typeof structuredClone === "function") {
    return structuredClone(value);
  }
  return JSON.parse(JSON.stringify(value));
}

function formatLiteral(value) {
  if (value == null) {
    return "";
  }
  if (typeof value === "object") {
    return JSON.stringify(value);
  }
  return String(value);
}
