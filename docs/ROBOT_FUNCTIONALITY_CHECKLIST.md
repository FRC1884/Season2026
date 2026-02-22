# Robot Functionality Checklist (KISS / YAGNI)

Use this before practice, scrimmage, or comp match blocks.

## 0) Quick Preflight (5 min)

- [ ] Code branch clean, expected commit deployed
- [ ] Robot boots cleanly (no fatal errors)
- [ ] DS connected, battery healthy, CAN stable
- [ ] All cameras connected and streaming
- [ ] Gyro zeros correctly

## 1) Vision (LL3/LL4)

- [ ] Confirm `AprilTagVision/<camera>/Profile` in logs
- [ ] Confirm `AprilTagVision/<camera>/ProfileSource` in logs
- [ ] Cover one camera, verify other still publishes tag IDs
- [ ] Single-tag test at close range: pose accepted
- [ ] Single-tag test at long range: bad frames rejected
- [ ] Multi-tag test: stable acceptance while driving slowly
- [ ] Fast spin test: yaw gate/rejection behaves as expected

Pass criteria
- [ ] No repeated pose jumps > ~0.5 m from one frame to next
- [ ] Reject reasons/logs are explainable (not random)

## 2) Pose Estimation

- [ ] Start from known field pose and reset odometry
- [ ] Drive a rectangle and return to start
- [ ] Compare estimated pose vs tape-measured pose

Pass criteria
- [ ] End error <= 0.25 m translation, <= 5 deg heading
- [ ] Vision fusion improves drift versus odometry-only run

## 3) Drivetrain

- [ ] Robot-relative and field-relative both work
- [ ] Rotation hold/heading control stable
- [ ] Max speed and accel feel predictable
- [ ] No oscillation after snap turns

Pass criteria
- [ ] Driver can place robot on target repeatedly without over-correction

## 3A) Gyro Failover + Odometry Reset Hardening

- [ ] Start enabled with both gyros healthy (`primaryConnected=true`, `secondaryConnected=true`, `usingSecondary=false`)
- [ ] Induce primary gyro failure (disconnect/disable Pigeon2) and verify fallback engages within a few cycles
- [ ] Confirm heading stays continuous during failover (no large yaw step)
- [ ] Restore primary gyro and verify smooth hand-back from NavX to Pigeon2
- [ ] Trigger driver reset (`zeroGyroAndOdometryToAllianceWall`) and verify vision is briefly suppressed after reset
- [ ] Trigger autonomous/resetSimulationField reset path and verify same vision suppression behavior

Pass criteria
- [ ] No yaw jump large enough to break driver control or auto tracking during failover/switch-back
- [ ] `Swerve/Gyro/usingSecondary` toggles correctly for failover and recovery
- [ ] No immediate vision snap for ~0.35s after any odometry reset path

## 3B) Shooting While Moving

- [ ] Enable moving-shot mode and confirm `Superstructure/ShootingWhileMoving/Enabled=true`
- [ ] At low speed, confirm `Superstructure/ShootingWhileMoving/MotionSampleValid=true` before shots are released
- [ ] Run constant-speed strafe shots left and right and compare hit rate vs stationary baseline
- [ ] Run forward/backward shots and confirm no obvious over-lead or under-lead pattern
- [ ] Verify `Turret/ShootingWhileMoving/LatencySec`, `ShotTimeSec`, and `LeadOffsetMeters` are populated in logs
- [ ] Force a stale/invalid motion sample (disable/enable or intentional pause) and verify fallback to normal (non-moving) targeting

Pass criteria
- [ ] Hit rate while moving is materially better than non-compensated baseline
- [ ] No NaN/invalid telemetry in moving-shot channels
- [ ] No shot release when moving-shot is enabled but motion sample is invalid

## 4) Autos

- [ ] Run each priority auto at least 3 times
- [ ] Verify initial pose is correct for alliance side
- [ ] Confirm event markers/actions trigger at correct times
- [ ] Check auto end pose is within tolerance

Pass criteria
- [ ] 3/3 clean runs for each priority auto
- [ ] End pose <= 0.35 m and <= 7 deg from expected

## 5) Teleop

- [ ] Driver controls mapped correctly
- [ ] Operator controls mapped correctly
- [ ] No command conflicts/stuck states
- [ ] All critical actions recover after disable/enable

Pass criteria
- [ ] Full 2:30 simulated match without software reset or dead controls

## 6) Regression Gate Before Merging

- [ ] `./gradlew test`
- [ ] Deploy and execute Sections 1-5 on robot
- [ ] Save one clean `.wpilog` and note commit SHA
- [ ] Record any failures as GitHub issues with logs attached
