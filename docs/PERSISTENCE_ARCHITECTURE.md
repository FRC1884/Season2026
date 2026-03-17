# Operator Board Persistence Architecture

## Scope

This phase treats dashboard-created artifacts as first-class robot assets, not transient browser state.

Managed assets:

- `joystick-mappings.json`
- `subsystem-descriptions.json`
- `diagnostics/bundles/*`
- `src/main/deploy/pathplana/autos/*`

## Storage Locations

Local development copy:

- `<repo>/operatorboard-data`
- `<repo>/src/main/deploy/pathplana/autos`

roboRIO runtime copy:

- `/home/lvuser/operatorboard-data`
- `/home/lvuser/deploy/pathplana/autos`

Deployment-bundle seed copy:

- `src/main/deploy/operatorboard/default-data/joystick-mappings.json`
- `src/main/deploy/operatorboard/default-data/subsystem-descriptions.json`

## Authority Rules

- Joystick mappings: local canonical copy with roboRIO mirror.
- Subsystem descriptions: local canonical copy with roboRIO mirror.
- PathPlanA autos: local canonical copy with roboRIO mirror, but robot-side edits must be pulled before deploy.
- Diagnostic bundles: roboRIO-authoritative append-only history with local mirror.

## Sync Workflow

`./gradlew deployPreserve` performs:

1. Pull saved roboRIO data into `build/deploy-preserve/robot`.
2. Back up local saved data into `build/deploy-preserve/backups`.
3. Merge local and robot copies via `tools/deploy_preserve/merge_saved_data.py`.
4. Run the normal GradleRIO `deploy`.
5. Push the merged saved data back to the roboRIO.
6. Verify required files still exist remotely.

## Conflict Resolution

- If only one side has a file, copy it to the other side.
- If both sides changed and `schemaVersion` matches, the newer `metadata.updatedAtEpochMs` wins.
- If both sides changed and `schemaVersion` differs, keep the local file in place and preserve both copies under `build/deploy-preserve/conflicts` for manual review.
- PathPlanA autos merge by relative file path and file modification time.
- Diagnostic bundles are append-only and never destructively merged.

## Version Tags

Saved JSON documents carry:

- `schemaVersion`
- `metadata.updatedAtIso`
- `metadata.updatedAtEpochMs`
- `metadata.buildVersion`
- `metadata.gitSha`
- `metadata.gitBranch`

## Operator Board API

The roboRIO-hosted operator board now exposes:

- `GET/PUT /api/joystick-mappings`
- `GET/PUT /api/subsystem-descriptions`
- `GET /api/storage/inventory`
- `GET /api/diagnostics/latest`
- `GET /api/diagnostics/bundles`
- `POST /api/diagnostics/export`

## Notes

- The joystick tab persists profiles now, but the active robot driver binding path still originates from the hard-coded `DriverMap` implementations. The saved profile is authoritative for dashboard UX, deploy-sync, and future command-binding integration.
- System checks now emit a durable diagnostic bundle instead of remaining UI-only state.
