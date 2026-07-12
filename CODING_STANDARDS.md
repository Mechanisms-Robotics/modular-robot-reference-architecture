# Team 8736 Coding Standards

These standards are derived from the conventions already established on the `main`
branch of this repository. They exist so that code written by any student, mentor, or
AI assistant looks like it was written by one team. When in doubt, imitate the best
existing example in the codebase (`PoseEstimator8736.java` for documentation,
`ModuleIOTalonFX.java` for hardware IO, `CONSTANTS.java` for configuration).

Reasonable exceptions are allowed (see [Exceptions](#exceptions)), but they should be
deliberate, not accidental.

---

## 1. Toolchain

* **Java 17** (`sourceCompatibility = JavaVersion.VERSION_17` in `build.gradle`).
* **WPILib + GradleRIO** for build/deploy, **AdvantageKit** for logging/replay.
* Vendor libraries live in `vendordeps/` and are updated deliberately, one at a time,
  with a commit message naming the old and new versions.
* Line endings are **LF** everywhere. Do not commit CRLF files (enforced by
  `.gitattributes`).

## 2. Project layout

```
src/main/java/frc/robot/
├── CONSTANTS.java              ← generated/copied per-robot configuration (see §6)
├── Main.java                   ← WPILib entry point; never add logic here
├── Robot.java                  ← mode lifecycle + logger setup only
├── RobotContainer.java         ← subsystem construction, button bindings, autos
├── PoseEstimator8736.java      ← team utility classes ("8736" suffix marks ours)
├── commands/                   ← Command classes (one class per file)
├── subsystems/<mechanism>/     ← one package per mechanism
│   ├── <Mechanism>.java        ← the SubsystemBase class
│   ├── <Device>IO.java         ← hardware-abstraction interface (+ @AutoLog inputs)
│   └── <Device>IO<Vendor>.java ← one implementation per vendor/sim backend
└── util/                       ← small stateless helpers
```

* **One mechanism = one package** under `subsystems/` (e.g. `drivetrain`, `vision`).
* Team-authored general-purpose classes that could collide with WPILib names carry
  the `8736` suffix (`PoseEstimator8736`).

## 3. The IO-layer pattern (required for all hardware)

Every piece of hardware is accessed through an AdvantageKit-style IO layer. This is
the load-bearing convention of the whole architecture — it is what makes simulation
and log replay possible.

* `XxxIO` is an interface containing:
  * a nested `@AutoLog public static class XxxIOInputs` holding **all** sensor data
    the robot code reads, with sensible defaults (`connected = false`, positions zero);
  * `public default void updateInputs(XxxIOInputs inputs) {}` plus default no-op
    setters (`setDriveVelocity`, `setTurnPosition`, ...). Defaults are no-ops so a
    disconnected/replay robot can be built with `new XxxIO() {}`.
* Implementations are named for their backend: `ModuleIOTalonFX`, `ModuleIOTalonFXRedux`,
  `GyroIORedux`, `GyroIOCTRE`, `ModuleIOSim`, `PoseCameraIOPhoton`, `PoseCameraIOSim`.
* The subsystem owns the IO plus one `XxxIOInputsAutoLogged` per device and calls, in
  `periodic()`:
  ```java
  io.updateInputs(inputs);
  Logger.processInputs("Drive/ModuleFrontLeft", inputs);
  ```
* **Rule: subsystem logic never touches vendor classes.** If you find yourself
  importing `com.ctre.*`, `com.reduxrobotics.*`, `com.revrobotics.*` or
  `org.photonvision.*` outside an `*IO*` implementation or `CONSTANTS.java`, you are
  breaking the architecture.
* Odometry-rate signals go through `PhoenixOdometryThread` queues; everything read by
  both the odometry thread and the main loop is guarded by `Drivetrain.odometryLock`
  (lock before reading inputs in `periodic()`, unlock immediately after — never hold
  it across control logic).

## 4. Naming

| Thing | Convention | Example |
|---|---|---|
| Classes / interfaces | `PascalCase` | `SwerveModule`, `PoseCameraIO` |
| Methods / fields / locals | `camelCase` | `desiredChassisSpeeds` |
| Constants (`static final`) | `SCREAMING_SNAKE_CASE` | `WHEEL_RADIUS`, `GYRO_CAN_ID` |
| Packages | lowercase, one word | `frc.robot.subsystems.drivetrain` |
| Log keys | `Subsystem/Item` hierarchy | `"Drive/Gyro"`, `"Odometry/Robot"` |

* **Units belong in names** whenever the type doesn't carry them:
  `driveVelocityRadPerSec`, `timestampSeconds`, `LENGTH_METERS`, `driveTempFahrenheit`.
  Prefer WPILib `Measure` types (`Distance`, `LinearVelocity`, ...) in `CONSTANTS`,
  and document the unit in a comment when a bare `double` crosses an API boundary.
* **Directional transforms name both frames in order**: a `Transform3d` that goes
  from the robot frame to the camera frame is `robotToCamera`, never `cameraTransform`
  or (worse) the reverse of what it actually is.
* No Hungarian prefixes: we do not use `m_member` or `kConstant` even though WPILib
  templates do.
* The `CONSTANTS` class name is intentionally all-caps — a deliberate, grandfathered
  exception that makes configuration references (`CONSTANTS.DriveConstants.…`) stand
  out. Do not create new all-caps class names.

## 5. Formatting

* **4-space indentation** for team-authored code. No tabs.
* Target ~80–100 characters per line. When a call or declaration doesn't fit, break
  it with **one argument per line** and the closing `)` on its own line — this is the
  dominant style on `main`:
  ```java
  this.frontLeftModule = new SwerveModule(
      frontLeftModuleIO,
      "Front Left"
  );
  ```
* Use `this.` when assigning instance fields and in code that mixes fields with
  similarly named locals/parameters. (Main is not 100% consistent here; new code
  should lean toward `this.` for field access.)
* Import order: `static` imports first, then third-party (`com.…`), then WPILib
  (`edu.…`), then team (`frc.…`), then `java.…`, alphabetical within each group.
  Remove unused imports before committing.
* No trailing whitespace; files end with a single newline.
* Braces on the same line as the declaration (`public void periodic() {`).

## 6. Constants and configuration

* Every tunable number, CAN ID, and hardware dimension lives in `CONSTANTS.java`,
  grouped in a nested static class per area (`DriveConstants`, `VisionConstants`,
  `FieldConstants`, `Timeouts`). No magic numbers in subsystem code.
* Per-robot configurations live in `src/config/constants/*.java.template`; the header
  banner of `CONSTANTS.java` must state which robot it configures. Never let the
  active `CONSTANTS.java` and its template drift silently apart.
* Constants that only feed other constants are `private`; only what other classes
  read is `public`.
* Comment the *provenance* of every tuned value: measured, calculated from CAD, sysid,
  or vendor default (`// tuned 2026-03-01 on comp bot`, `// theoretical, needs sysid`).
* CAN IDs for one corner/mechanism are grouped together with a `// Front Left`-style
  header comment. Oddities get called out (`// drive/steer IDs swapped on Mechiatto`).

## 7. Logging

* Inputs: `Logger.processInputs("<Subsystem>/<Device>", inputs)` — every IO, every loop.
* Outputs/setpoints: `Logger.recordOutput("<Subsystem>/<Thing>", value)`.
* Derived state getters use `@AutoLogOutput(key = "<Subsystem>/<Thing>")`.
* Keys are stable, hierarchical, and never contain per-instance dynamic text other
  than the device name given at construction.
* All logging goes through AdvantageKit `Logger` so it appears in replay;
  `SmartDashboard` is reserved for operator-facing controls (choosers, buttons).

## 8. Comments and documentation

* Every class gets a Javadoc block saying what it is **and why it exists** (see
  `PoseEstimator8736`, `PhoenixOdometryThread` for the expected tone).
* Every non-obvious public method gets Javadoc with `@param`/`@return` including units.
* Inline comments explain **why**, plus physical/units context the code can't express:
  coordinate conventions (`// +X forward, +Y left, CCW positive`), latency handling,
  vendor quirks, field-relative vs robot-relative frames.
* Open questions are `// TODO:` with enough context that someone else can pick them
  up. Delete TODOs when resolved. No profanity/slang in comments — logs and code get
  shown on projectors at competitions and to sponsors.

## 9. Hardware & control conventions

* Coordinates follow WPILib: **+X forward, +Y left, CCW-positive**, blue-alliance
  origin. Any alliance-dependent flip happens in exactly one documented place.
* Motor configs are applied with retry (`PhoenixUtil.tryUntilOk`) and timeouts from
  `CONSTANTS.Timeouts` — never bare `.apply()` on the CAN bus at startup.
* Steer motors always use `ContinuousWrap`; drive motors default to brake mode.
* Status-signal update rates are set explicitly, then
  `ParentDevice.optimizeBusUtilizationForAll(...)` disables everything else. Any
  signal you read **must** be registered with an update frequency **and refreshed**
  before reading — reading an unregistered signal returns frozen data.
* Degrees/rotations/radians conversions happen at the IO boundary; subsystem logic is
  radians/meters (WPILib native).

## 10. Git & workflow

* Never push directly to `main`. Branch from `develop` (`feature/...`, `fix/...`),
  PR with review after a successful simulation.
* Commits are small and messages explain **what and why**, present tense
  ("Fix steer offset sign on front-left module — was fighting the encoder").
* Code must build (`./gradlew build`) before every PR; simulate before merging
  drivetrain/control changes.

## Exceptions

* **WPILib template files** (`Main.java`, `Robot.java`) keep the upstream 2-space
  template formatting and license header; don't reformat them wholesale.
* **Vendored/derived files** (`PhoenixOdometryThread.java`, from Littleton Robotics)
  keep their upstream license header and style; fixes there should stay minimal and
  clearly marked.
* Generated files (`*AutoLogged`, build outputs) are never edited by hand.
* Deviations beyond these require a comment at the deviation site explaining why.
