# PathPlanA Integration

`PathPlanA` is the planner-side authoring tool for REBUILT autos.

## Data flow

1. Build autos in the Flutter planner app.
2. Export the deploy library into `src/main/deploy/pathplana/autos/`.
3. Each auto lives in its own folder with an `auto.json`, and `index.json` maps auto ids to those files.
4. The operator board fetches that deploy library from `/planner-autos/index.json`.
5. Preview/select an auto on the dashboard.
6. The dashboard publishes only `SelectedAutoId`.
7. The robot loads that deploy auto through `RebuiltAutoQueue` and executes it with `AutoAlignToPoseCommand`.

## Robot execution model

The robot does not run a separate spline follower here. It keeps the current objective-based flow:

- route waypoints are treated as intermediate poses
- each pose is driven with `AutoAlignToPoseCommand`
- live robot pose is used every execute cycle
- vision correction remains whatever the swerve/vision stack is already feeding into the robot pose estimate

This means planner autos stay compatible with the existing auto-align and vision correction model instead of bypassing it.

## Deploy library layout

The robot and dashboard both read the same deploy-backed library:

- `src/main/deploy/pathplana/autos/index.json`
- `src/main/deploy/pathplana/autos/<auto-folder>/auto.json`

`index.json` is the manifest. Each entry needs:

- `id`
- `name`
- `folder`
- `relativePath`
- `updatedAt`

The per-auto `auto.json` carries the runnable step data:

- `startPose`
- `customZones`
- `steps`

## Supported per-step planner tuning

The queue parser now accepts these optional step-level fields:

- `constraintFactor`
- `toleranceMeters`
- `timeoutSeconds`
- `endVelocityMps`
- `stopOnEnd`

Those map into the existing `AutoAlignToPoseCommand` call path.

## Vendordep note

`vendordeps/PathPlanA.json` is currently a metadata vendordep placeholder for the planner ecosystem. It does not pull a Java/JNI artifact yet, so it does not change robot build resolution. The real planner app/release repo should publish the same vendordep file under `vendor/PathPlanA.json`.
