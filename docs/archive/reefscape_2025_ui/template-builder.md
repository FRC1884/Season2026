# Reef Controls Template Builder

Use the builder to author Reef Controls templates without touching `index.js`. The JSON it produces maps directly to what the tablet UI reads from `src/main/deploy/reefcontrols/templates`.

## Quick start
- Open `docs/template-builder/index.html` in a browser (no server required).
- Tweak the sections, then click **Copy JSON** or **Download JSON**. **Reset** loads the defaults from `templates/2025.json`.
- Save the file as `src/main/deploy/reefcontrols/templates/<id>.json`.
- Load the UI with `?template=<id>` (and `?ntHost=<rio-ip>` when running the page off-robot), then redeploy or copy the JSON onto the RIO.

## Template fields
- **metadata**: `title`, `subtitle`, optional `favicon` URL (used for document title/icon).
- **theme**: `levelBackgrounds` sets the three state colors (L2/L3/L4), `disconnectedBackground` paints the body when NT4 is disconnected.
- **layout**: `controlsLayout` of `grid` or `absolute`; `snapToGrid` and `gridSize` snap node/control coordinates (percent units) when desired.
- **overview.counters**: Each counter shows `Level1`/`Level2`/`Level3`/`Level4`. `selectsLevel` picks the active reef level (0=L2, 1=L3, 2=L4) when tapped.
- **reef**: `branchImage`/`algaeImage` with default sizes `branchSizeVh` (5) and `algaeSizeVh` (7); `flagEmoji` for the RP marker. `branchNodes` and `algaeNodes` take `x`/`y` percentages plus optional `face`, `side`, and `sizeVh` overrides.
- **controls**: Label/icon/class styling plus placement (`position.x`/`position.y` when `controlsLayout=absolute`). Actions:
  - `topic-delta`: Increment/decrement a topic with optional `min`/`max`.
  - `topic-toggle`: Flip a boolean.
  - `topic-set`: Publish a literal value (numbers/booleans/strings/JSON).
  - `align-source`: Send an ALIGN_SOURCE queue command with `source` of `NEAREST`/`LEFT`/`RIGHT`.
  - `queue-command`: Send a queue command string.
  - Topic binding: leave blank to use `/ReefControls/ToRobot/*` (e.g., `Level2`, `QueueCommand`), set `topic` for a full path, or set `ntTable` + `ntKey` to publish to `/table/key`. Target `SelectedLevel` to change the active reef level.
- **queue**: Customize the queue panel title/subtitle/hint, manual toggle text, and command buttons (`label` + `command` string). Set `queue` to `false` in JSON if the queue should be hidden entirely.

## Assets and testing
- Images such as `coral.png` and `algae.png` live beside `src/main/deploy/reefcontrols/index.html`; point `branchImage`/`algaeImage` (or per-node `image`) at files in that folder or serve them from a URL.
- To preview a template locally, open `src/main/deploy/reefcontrols/index.html?template=<id>&ntHost=localhost` in a browser with a running NT4 server or deploy to the robot and browse to `http://<rio-ip>:5805/?template=<id>`.
