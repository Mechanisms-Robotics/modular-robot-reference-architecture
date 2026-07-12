# Project Review — Team 8736 "Rebuilt" 2026 Robot Code

**Reviewed:** 2026-07-12, branch `joel-complete-review` (main + PR #20, then cleaned).
**Reviewer:** Complete pass over every Java source file, the Gradle build, vendordeps,
and repo configuration. Every claim below cites a file so a human or an AI assistant
can act on it without re-deriving context.

This document has three jobs:
1. Record **what is good** so future seasons keep doing it on purpose.
2. Record **what is weak or broken** with enough precision to fix it.
3. Provide a **prioritized roadmap** where each item states the problem, the change,
   the files involved, and an acceptance test — implementable by a student or a
   smaller AI model without further research.

---

## 1. Executive summary

The architecture is genuinely good: a clean AdvantageKit IO-layer swerve codebase
with high-frequency odometry, pose estimation with vision fusion, and working
simulation. It is a strong foundation that most FRC teams would be happy to have.

The weaknesses are concentrated in three places:
- **The autonomous story is incomplete.** No trajectories ship in the repo, only a
  "None" auto is registered, and the path follower feeds Choreo samples through a
  controller that assumes the robot travels nose-first (`FollowPath.java`).
- **Verification is aspirational.** The README promises testability; there are zero
  tests (`src/test/` does not exist) and no CI, so nothing prevents a broken build
  from being merged.
- **Configuration hygiene decayed.** The per-robot constants machinery is commented
  out (`build.gradle`), its template had drifted into a byte-for-byte copy of the
  competition constants, and several tuned-looking values have no recorded provenance.

A set of real bugs found during this review has already been fixed on this branch
(§4). The highest-value next steps are: a correct Choreo follower, real registered
autos, distance-scaled vision trust, a handful of unit tests around the geometry
math, and CI (§5).

---

## 2. What's good — keep doing this

| # | Strength | Evidence |
|---|----------|----------|
| G1 | **IO-layer architecture everywhere.** Every device (modules, gyro, cameras) sits behind an interface with an `@AutoLog` inputs class and per-vendor implementations. Subsystem logic never touches vendor APIs. This is what makes sim and log replay possible, and the team applied it consistently — including retrofitting it onto the cameras (`subsystems/vision/PoseCameraIO*.java`). | whole `subsystems/` tree |
| G2 | **High-frequency odometry done properly.** `PhoenixOdometryThread` (adapted from AdvantageKit) samples wheel/gyro positions at 100–250 Hz with latency-compensated timestamps, drained under a lock each loop. Few teams get this right. | `PhoenixOdometryThread.java`, `Drivetrain.periodic()` |
| G3 | **Centralized, grouped constants with units-typed values.** CAN IDs, gear ratios, and tunables live in one file using WPILib `Measure` types (`Amps.of(90)`, `Inches.of(2.23)`), grouped per subsystem. | `CONSTANTS.java` |
| G4 | **Simulation is first-class.** Physics sim for modules (`ModuleIOSim`), a full PhotonVision camera sim with noise/latency/wireframe (`PoseCameraIOSim`), and a deliberate odometry-only ground-truth estimator so sim vision can't feed itself (`PoseEstimator8736`). |
| G5 | **Vendor-swap flexibility proven.** Two gyro backends (Redux, CTRE) and two module encoder backends exist behind the same interfaces — the team actually exercised the architecture's main selling point. | `GyroIORedux/CTRE`, `ModuleIOTalonFX(Redux)` |
| G6 | **Sane hardware config practices.** Config-with-retry (`PhoenixUtil.tryUntilOk`), explicit status-frame frequencies + `optimizeBusUtilization`, brake mode, steer `ContinuousWrap`, absolute-encoder seeding at boot. | `ModuleIOTalonFX*.java` |
| G7 | **A real branching/tagging strategy for competition**, documented with a diagram and pit commands. | `README.md`, `res/img/branch-strategy.png` |
| G8 | **Driver-experience details**: field-oriented drive with alliance-aware heading, squared inputs with magnitude deadband, heading re-zero button, boot-time auto-selection reset so a stale dashboard can't start the wrong auto. | `RobotContainer.java`, `Robot.java` |

---

## 3. What's not good

### 3.1 Autonomous (the biggest gap)

- **N1. No trajectories exist.** `src/main/deploy/` contains only `example.txt`; the
  Choreo project file and `Test Path.traj` were deleted in PR #20. `FollowPath` is
  currently unreachable code.
- **N2. Only "None" is registered as an auto.** `RobotContainer.publishAutoNames()`
  has a single map entry. There is auto-selection plumbing but nothing to select.
- **N3. The follower mis-drives strafe paths.** `FollowPath.execute()` collapses the
  sample's `(vx, vy)` into a scalar speed and hands it to WPILib's
  `HolonomicDriveController`, which assumes travel along the reference pose's
  heading. Swerve paths where heading ≠ direction of travel (most of them) get their
  feedforward pointed the wrong way, leaving the P controllers to drag the robot
  around. Choreo's docs recommend direct `vx/vy/omega` feedforward + per-axis PID.
- **N4. No end-state tolerance.** `isFinished()` is purely time-based; the command
  declares victory wherever it happens to be when the clock runs out.

### 3.2 Verification

- **N5. Zero tests.** JUnit 5 is fully configured in `build.gradle` (lines 86–96) but
  `src/test/` does not exist. Pure-math code (pose flipping, sample mirroring,
  odometry deltas) is ideal test material and has already had real bugs (§4: F1, F9).
- **N6. No CI.** No `.github/workflows/`. PRs can merge without compiling. The team's
  own README rule ("PR after a successful simulation") is enforced by nothing.
- **N7. Simulation-only physics quirks go unnoticed** because nothing asserts on sim
  behavior (the 2x-speed sim bug fixed in §4/F6 shipped for months).

### 3.3 Configuration & repo hygiene

- **N8. Per-robot constants machinery is dead.** The `prepareConstants` /
  `validateConstantsNotModified` Gradle tasks are fully commented out
  (`build.gradle` ~lines 150–230) and referenced a `CONSTANTS_Rebuilt.java.template`
  that no longer exists. The surviving Mechiatto template had drifted into an exact
  copy of the competition constants (fixed cosmetically on this branch; the real
  Mechiatto values still need measuring).
- **N9. Tuning provenance is unrecorded.** `DRIVE_GAINS` (kP 1.6, kS 0.129, kV 0.753)
  look sysid-derived; `PATH_FOLLOWER_P_X = 10.0` looks hand-tuned; `SPEED_AT_12_VOLTS
  = 4.0 m/s` is marked "needs tuning" upstream. Nobody can tell which numbers are
  measured vs. guessed. Dead tunables linger (`ANGLE_KP/KD`, `FF_START_DELAY`,
  `WHEEL_RADIUS_*`, `PATH_PLANNER_*` are unused since the sysid/PathPlanner code was
  stripped).
- **N10. README is a placeholder** ("TODO: [Students] Update this README") with the
  old framework text commented out below it.
- **N11. Line endings had churned** (main was CRLF, PR #20 flipped everything to LF,
  producing whole-file diffs). `.gitattributes` added on this branch prevents a
  repeat, but history remains noisy.

### 3.4 Vision quality

- **N12. Fixed measurement trust.** Every accepted estimate gets std devs
  `(0.9, 0.9, 0.9)` regardless of tag count, distance, or ambiguity
  (`Vision.periodic()`). One far, glancing tag is trusted exactly as much as a
  close multi-tag solve — and 0.9 rad heading trust lets vision fight the gyro.
- **N13. Weak gating.** Only a Z sanity check exists. No field-boundary check, no
  single-tag ambiguity threshold, no max-distance cut.
- **N14. Logging namespaces are inconsistent.** Camera IOs log under a top-level
  `<cameraName>/pose` key while the subsystem logs under `Vision/<index>`; module
  logs use `"Drive/Module Front Left"` and `"Module Front Left/..."` split roots
  (`SwerveModule.java`). Finding related data in AdvantageScope requires knowing
  three naming schemes.
- **N15. Each `PoseCameraIOSim` builds its own `VisionSystemSim("visionSim")`** —
  two sim cameras would collide on the same NetworkTables name and duplicate the
  tag layout; the sim system should be shared.

### 3.5 Smaller code-level issues (all still open)

- **N16.** `Drivetrain.periodic()` indexes `gyroInputs.odometryYawPositions[i]`
  parallel to module sample arrays with no length guard — a device hiccup that
  desyncs queue counts would throw `ArrayIndexOutOfBoundsException` in the main loop.
- **N17.** `GyroIOCTRE` registers yaw with the odometry thread as a *generic*
  supplier, so Pigeon yaw samples skip the latency-compensated Phoenix path that
  module signals use (`registerSignal(StatusSignal)` variant with `.clone()`).
- **N18.** `PhoenixUtil.tryUntilOk` gives up silently — a module can enter a match
  unconfigured with zero log evidence.
- **N19.** `GyroIO.zeroGyro()` is implemented but never called; heading resets go
  through pose resets only. Dead API — either use it in `resetHeading()` or delete it.
- **N20.** `DrivetrainController` is a one-method wrapper that exists only for
  teleop; either grow it into the real teleop-command home or fold it into
  `RobotContainer`.
- **N21.** The `Robot Drives Untuned` state (commit c33d8bb on main) was never
  revisited: steer kP 60 / drive kP 1.6 on the branch vs 25 / 0.1 on main, with no
  record of which chassis either set was tuned on.
- **N22.** `CONSTANTS.CURRENT_MODE` can never be `REPLAY` without editing source
  (`SIM_MODE` is hardcoded to `SIM`) — log replay, a headline AdvantageKit feature,
  has no switch.
- **N23.** Robot code selects PS4 controller mappings (`CommandPS4Controller`) with
  a rotation-axis workaround comment history in git — worth a documented controller
  standard.

---

## 4. Fixed during this review (already on this branch)

| ID | Fix | Commit |
|----|-----|--------|
| F1 | FollowPath left vision fusion disabled after the first path — robot ignored AprilTags for the rest of the match | 9d6e545 |
| F2 | FollowPath theta profile passed max-acceleration into the max-velocity slot (20 rad/s vs intended 8) | 9d6e545 |
| F3 | FollowPath `isMirrored` reflection didn't negate `alpha` and didn't swap left/right module force pairs | 9d6e545 |
| F4 | `publishAutoNames()` was never called — auto chooser never appeared on the dashboard; the resulting null produced the `getName()` NPE that had been hotfixed around | f581a2d |
| F5 | `GyroIOCTRE` set the angular-velocity frame to 0.01 **Hz** (a period was passed where Phoenix expects a frequency): one update per 100 s | d7c698d |
| F6 | Module temperature signals were read without registration after `optimizeBusUtilization` disabled their frames — logs showed frozen boot-time values; also `turnEncoderConnected` was hardcoded `true`, masking real encoder disconnects | d7c698d |
| F7 | `ChassisSpeeds.discretize()` was computed and logged but the *raw* speeds were sent to the kinematics — rotation-while-driving skew compensation never reached the wheels | ce3d69c |
| F8 | `simulationPeriodic()` re-ran `periodic()`, double-stepping sim physics (2x-speed robot) and double-feeding the estimator | ce3d69c |
| F9 | `PoseEstimator8736.resetPose` didn't reset the sim ground-truth estimator, so sim vision rendered tags from a stale pose after any reset | c81d20c |
| F10 | The dashboard "Reset Pose" chooser fired every loop while selected, continuously clearing the estimator's latency buffer (silently disabling vision); now edge-triggered | c81d20c |
| F11 | Vision Z gate only rejected above-floor solutions; `Math.abs` now also rejects below-floor ones | c81d20c |
| F12 | Deprecated-for-removal APIs (`TalonFX(int,String)`, `Command.schedule()`) replaced; build is warning-free so new warnings are visible | 02beac7 |

Cleanup also landed: `CODING_STANDARDS.md` (910c769), typo/naming/dead-code pass
including the `cameraToRobot`→`robotToCamera` frame-direction correction (7f78688,
ee17a77), and `.gitattributes` line-ending enforcement (7f78688).

---

## 5. Roadmap for future seasons

Ordered by priority. Each item is self-contained; an implementer needs only this
section plus the named files.

### P0-1. Replace the FollowPath controller with a proper Choreo follower
- **Problem:** N3 — feedforward direction is wrong for strafe paths.
- **Change:** In `commands/FollowPath.java`, delete `HolonomicDriveController` and
  compute field-relative speeds directly from the sample:
  `vx = sample.vx + kPx*(sample.x - pose.x)`, same for y;
  `omega = sample.omega + kPtheta*(wrap(sample.heading - pose.heading))` using a
  `PIDController` with continuous input for heading. Convert field→robot speeds with
  `ChassisSpeeds.fromFieldRelativeSpeeds(speeds, pose.getRotation())` (NOT the
  driver-perspective version in `DrivetrainController`).
- **Acceptance:** a sim run of a pure-strafe path tracks within ~5 cm; unit test on
  the speed calculation with a synthetic sample.

### P0-2. Ship at least one real auto
- **Problem:** N1, N2 — the auto plumbing has nothing to select.
- **Change:** (a) add a hardware-independent `DriveForward` command (robot-relative
  1.5 m/s for 2 s, then stop) and register it in `publishAutoNames()`; (b) commit a
  Choreo project + one trajectory to `src/main/deploy/choreo/` and register a
  `FollowPath` auto for it, guarded so a missing file logs an error instead of
  crashing (`Choreo.loadTrajectory` returns `Optional`).
- **Acceptance:** both autos appear in the dashboard chooser and run in sim.

### P0-3. Add CI
- **Problem:** N6 — nothing enforces "it compiles" on PRs.
- **Change:** `.github/workflows/build.yml`: on push/PR — checkout, setup-java
  (Temurin 17), `./gradlew build --no-daemon` (build also runs tests).
- **Acceptance:** a PR with a syntax error shows a red X.

### P0-4. Start the test suite with the geometry math
- **Problem:** N5 — the mirror/flip math already had two real bugs (F2, F3).
- **Change:** create `src/test/java/frc/robot/` with tests for:
  `FieldUtil.flipPose` (double-flip = identity; heading negation),
  FollowPath's mirror transform (extract it to a static package-visible
  `mirrorSample(SwerveSample)` first), and `PoseEstimator8736.updateOdometry`
  gyro-less heading integration (drive all 4 wheels in an arc, expect rotation).
  Note: WPILib math classes work headless; no HAL needed for these.
- **Acceptance:** `./gradlew test` runs ≥3 meaningful tests green in CI.

### P1-5. Scale vision trust by tag count and distance
- **Problem:** N12, N13.
- **Change:** in `PoseCameraIO`, add per-estimate `tagCount` and `avgTagDistance`
  input arrays (both IOs can compute them from `EstimatedRobotPose.targetsUsed`).
  In `Vision.periodic()`: reject estimates outside the field
  (`FieldConstants.LENGTH/WIDTH` + margin), reject single-tag estimates beyond ~4 m,
  then `stdDev = base * distance² / tagCount` with `base` in `VisionConstants`
  (linear ~0.08, angular ~0.16 — AdvantageKit template values; tune on-field), and
  use a large angular std dev when the gyro is healthy.
- **Acceptance:** in sim, a far single-tag view visibly stops yanking the pose;
  logged std devs vary with distance.

### P1-6. Decide the constants machinery's fate (re-enable or delete)
- **Problem:** N8 — half-dead build machinery confuses everyone.
- **Change (preferred):** re-enable `prepareConstants` keyed on `-PtargetRobot`,
  restore a real `CONSTANTS_Rebuilt.java.template`, measure and fill actual
  Mechiatto values, and make `deploy` depend on it. Otherwise delete the commented
  block and the template directory and declare this repo single-robot.
- **Acceptance:** `./gradlew build -PtargetRobot=mechiatto` produces a CONSTANTS.java
  that differs from the rebuilt one (or the machinery is gone).

### P1-7. Adopt an auto-formatter
- **Problem:** mixed 2/4-space, wrap styles, trailing whitespace (see
  `CODING_STANDARDS.md` §5 for the target style).
- **Change:** add Spotless to `build.gradle` (`palantir-java-format` or
  `prettier-java`), one whole-repo `spotlessApply` commit, `spotlessCheck` in CI.
- **Acceptance:** CI fails on unformatted code.

### P1-8. Make REPLAY reachable
- **Problem:** N22.
- **Change:** in `CONSTANTS`, read an env var / system property
  (e.g. `AKIT_REPLAY=1`) to select `Mode.REPLAY` when not on a real robot; document
  the AdvantageKit replay workflow in the README.
- **Acceptance:** a log file can be replayed without editing source.

### P2-9. Robustness hardening (small, independent items)
- Length-guard the gyro/module odometry arrays in `Drivetrain.periodic()` (N16).
- Register Pigeon yaw as a Phoenix signal (`registerSignal(gyro.getYaw().clone())`)
  in `GyroIOCTRE` (N17).
- Make `tryUntilOk` log device + status code on final failure and surface an
  AdvantageKit `Alert` (N18).
- Wire `gyroIO.zeroGyro()` into `resetHeading()` or delete the API (N19).
- Share one `VisionSystemSim` across sim cameras (N15).
- Delete unused tunables or mark them `// UNUSED — kept for <reason>` (N9).
- Unify log namespaces: everything drivetrain under `Drive/`, everything vision
  under `Vision/<cameraName>/` (N14).

### P2-10. Operational tooling for competition
- AdvantageKit `Alert`s for disconnected devices (modules, gyro, cameras) surfaced
  on the dashboard — the `connected` inputs already exist, nothing reads them.
- Re-enable `WPILOGWriter` (USB logging) for REAL mode at events (it's commented out
  in `Robot()`); log to `/U/logs` with a free-space check.
- A pre-match dashboard tab: alliance, selected auto, gyro health, camera FPS.
- Document the sysid workflow and keep the characterization hooks
  (`runCharacterization`, `getFFCharacterizationVelocity`) exercised each season —
  they currently have no callers (the sysid routines from PR #14 were stripped).

### P3-11. Structural niceties
- Promote `DrivetrainController` into the real home of teleop drive logic (move the
  default-command lambda out of `RobotContainer`) or delete it (N20).
- Record tuning provenance inline per `CODING_STANDARDS.md` §6 (N9, N21).
- Rewrite the README for this season: subsystem map, how to add an auto, how to run
  sim, controller bindings table (N10, N23).
- Consider AdvantageKit's `LoggedTunableNumber` pattern (PR #11 attempted this) for
  on-the-fly PID tuning without redeploys.

---

## 6. How to verify changes (for humans and AIs)

```bash
# Requires JDK 17. From the repo root:
./gradlew build          # compile + (future) tests
./gradlew simulateJava   # desktop simulation with the sim GUI
```

- The build must stay **warning-free** (F12); treat new deprecation warnings as work.
- Drivetrain/control changes: run the sim, drive with the keyboard/joystick, watch
  `Odometry/Robot` and `SwerveStates/*` in AdvantageScope.
- The coding conventions this repo follows are in `CODING_STANDARDS.md`.

## 7. File map (orientation for new contributors)

```
Robot.java                 mode lifecycle, logger setup, dashboard choosers
RobotContainer.java        subsystem wiring (real vs sim IO), bindings, autos
CONSTANTS.java             all tunables/IDs; per-robot templates in src/config/
PoseEstimator8736.java     odometry+vision fusion; sim ground-truth twin
commands/FollowPath.java   Choreo trajectory follower (see P0-1)
subsystems/drivetrain/     Drivetrain, SwerveModule, module/gyro IO layers,
                           PhoenixOdometryThread (vendored, 100-250 Hz sampling)
subsystems/vision/         Vision subsystem + PhotonVision real/sim camera IOs
util/                      FieldUtil (alliance/flip), PhoenixUtil (config retry)
```
