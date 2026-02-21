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
