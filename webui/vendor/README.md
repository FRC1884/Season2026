Vendored NetworkTables (NT4) Browser Library

Why
- The UI connects to the roboRIO via NetworkTables v4 (NT4) over WebSocket.
- At events you may be offline, so relying on a CDN is risky.

Option A — Official WPILib NetworkTablesClients repo (native clients)
- That repository provides native (C/C++) clients, not a browser JS bundle. Use it for desktop apps or bridges.
- If you build/obtain a browser JS from WPILib in the future, place it at:
  webui/vendor/nt4-wpilib.js and the page will load it first.

Option B — NPM UMD (recommended for browsers)
1) Download the UMD browser build of an NT4 JS client:
   - Common package name: ntcore-ts
   - Example (open in browser and save):
     https://unpkg.com/ntcore-ts/dist/ntcore.umd.js
   - If needed, pin a version (example):
     https://unpkg.com/ntcore-ts@0.2.5/dist/ntcore.umd.js
2) Place the file at:
   webui/vendor/ntcore.umd.js
3) The web UI will first try the CDN, then fall back to this local file.

Notes
- If you prefer to pin a different version, update the script tag in index.html accordingly.
- If you later adopt another NT4 client library, app.js detects common client shapes automatically.
