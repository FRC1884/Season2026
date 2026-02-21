# Logging Guardrails

This project enforces logging policy with the Gradle task `checkLoggingPolicy` (wired into `check`).

## What is enforced

- `System.out.println` and `System.err.println` are disallowed in `src/main/java`.
- Direct `Logger.recordOutput(...)` is only allowed in files listed in:
  - `config/logging/recordoutput-allowlist.txt`

## Workflow

1. Prefer using shared logging helpers and mode gates (`GlobalConstants.isDebugMode()` / `RobotLogging`).
2. If you add `Logger.recordOutput(...)` to a new file, update:
   - `config/logging/recordoutput-allowlist.txt`
3. Run:
   - `./gradlew check`

## Why allowlist exists

- It keeps telemetry growth intentional.
- It forces review when logging surfaces expand.
- It protects competition mode from accidental log spam.
