# PathPlanA Contract

PathPlanA exports planner packages as JSON.

The operator board accepts either:

- a top-level array of autos
- or an object with an `autos` array

Each auto uses the same queue schema the robot already consumes through `AutoQueueSpec`.

## Package shape

```json
{
  "version": "2026.1",
  "generator": "PathPlanA",
  "commandProfiles": [
    {
      "id": "cmd-intake",
      "name": "Rear Intake",
      "requestedState": "INTAKING",
      "colorHex": "#39D98A"
    }
  ],
  "autos": [
    {
      "id": "hub-cycle-upper",
      "name": "Hub Cycle Upper",
      "folder": "Cycle Autos",
      "updatedAt": 1773705600000,
      "startPose": {
        "xMeters": 1.55,
        "yMeters": 5.75,
        "headingDeg": 180.0
      },
      "customZones": [],
      "eventMarkers": [
        {
          "id": "marker-1",
          "name": "Spin Intake",
          "progress": 0.18,
          "commandId": "cmd-intake"
        }
      ],
      "eventZones": [
        {
          "id": "zone-1",
          "name": "Collect Window",
          "xMinMeters": 1.12,
          "yMinMeters": 5.18,
          "xMaxMeters": 2.38,
          "yMaxMeters": 6.30,
          "enterCommandId": "cmd-intake",
          "activeCommandId": "cmd-intake",
          "colorHex": "#39D98A"
        }
      ],
      "constraintZones": [
        {
          "id": "constraint-1",
          "name": "Approach Slowdown",
          "startProgress": 0.62,
          "endProgress": 0.96,
          "maxVelocityMps": 2.3,
          "maxAccelerationMpsSq": 1.9,
          "constraintFactor": 0.55
        }
      ],
      "steps": [
        {
          "spotId": "blue-depot",
          "commandId": "cmd-intake",
          "commandName": "Rear Intake",
          "label": "Depot Intake",
          "group": "DEPOT",
          "requestedState": "INTAKING",
          "waitSeconds": 0.25,
          "xMeters": 1.82,
          "yMeters": 5.92,
          "headingDeg": 180.0,
          "routeWaypoints": [
            {
              "xMeters": 1.44,
              "yMeters": 5.38,
              "headingDeg": 180.0
            }
          ]
        }
      ],
      "plannerSettings": {
        "visionCorrection": {
          "enabled": true,
          "poseBlendWeight": 0.35
        },
        "autoAlign": {
          "constraintFactor": 0.8,
          "toleranceMeters": 0.06,
          "timeoutSec": 1.6,
          "flowThroughEndVelocityMps": 1.2
        }
      }
    }
  ]
}
```

## Runtime behavior

- The robot still executes autos through `RebuiltAutoQueue`.
- Route waypoints are fed into the existing `AutoAlignToPoseCommand` sequence.
- Robot pose and vision stay live on the robot side; PathPlanA only authors and exports the intended queue.
- Unknown JSON fields are ignored by the robot parser, so planner-side settings can grow without breaking runtime parsing.
- Event markers, event zones, and constraint zones are planner-side metadata today. Event zones are spatial field boxes, not time slices. They are preserved in the export/package format even when the robot is only consuming the queue-driving subset.
- `folder` is planner-side project metadata used for the left browser/library grouping.

## Dashboard behavior

- The operator board is preview/select only.
- Imported autos are persisted locally in browser storage.
- Selecting an auto stages its queue spec to the robot when NetworkTables is connected.
- Planner `commandName` values are preserved for dashboard preview text, while the robot still executes the mapped `requestedState`.
