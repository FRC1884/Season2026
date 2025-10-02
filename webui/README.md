# WebUI Hosting (Windows)

This is a static site. Host it from any Windows laptop on the robot network; iPads and other devices can connect over Wi‑Fi.

## Quick start (PowerShell)

1) Open PowerShell in this folder (`webui/`).
2) Run:
   - `./serve.ps1` (binds on `0.0.0.0:8000`)
3) On your iPad, open `http://<laptop-ip>:8000` and click Connect.

If PowerShell script can’t find Python/Node, install one of:
- Python: https://python.org (then rerun `./serve.ps1`)
- Node.js: https://nodejs.org (then rerun `./serve.ps1`)

## Quick start (Command Prompt)

- `serve.bat` (same behavior as PowerShell script)

## Manual commands

- Python (recommended):
  - `python -m http.server 8000 --bind 0.0.0.0`
- Node (alternative):
  - `npx http-server -a 0.0.0.0 -p 8000`

Then browse to `http://<laptop-ip>:8000` from the iPad.

## Firewall note

Allow inbound TCP on the chosen port (default 8000):
- Run PowerShell/Command Prompt as Administrator and execute:
  - `netsh advfirewall firewall add rule name=webui dir=in action=allow protocol=TCP localport=8000`

## Finding your laptop’s IP

- PowerShell: `Get-NetIPAddress -AddressFamily IPv4 | Select IPAddress,InterfaceAlias`
- Or run `ipconfig` and use the IPv4 address for the Wi‑Fi/Ethernet you’re on with the robot.

## NT4 client library (offline)

If there’s no internet at the event, download the NT4 UMD once and place it at:
- `webui/vendor/ntcore.umd.js`

The page loads the CDN first, then falls back to this local copy.

## Connecting to the roboRIO

- In the page, set the roboRIO to: `roborio-<TEAM>-frc.local` or its IP.
- The UI connects to `ws://ROBORIO:5810/nt/` and publishes/subscribes the topics in `docs/external-webui.md`.

