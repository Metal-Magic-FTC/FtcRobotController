# AarushImprovements — CHANGELOG

All changes are against the code used at the **Chesapeake Championship (March 2026)**  
where Metal Magic #23362 placed **9th in the Blue Crab division** with the 12-ball sorted autonomous + leave points.

---

## Structure

```
AarushImprovements/
├── autonomous/
│   ├── RedClose12BallAutonomous.java    ★ primary autonomous
│   └── BlueClose12BallAutonomous.java   ★ Blue-side mirror
├── limelight/
│   └── FusedPose.java                   fused pose estimation for turret teleop
├── paths/
│   ├── RedClose12Paths.java             generated path definitions (Red)
│   └── BlueClose12Paths.java            generated path definitions (Blue, mirrored)
├── subsystems/
│   ├── Spindexer.java                   rotating 3-slot ball storage + color sensors
│   ├── Launcher.java                    flywheel + flick motor + hood servo
│   ├── Intake.java                      intake motor
│   └── MecanumDrive.java                four-wheel mecanum (replaces CustomMecanumDrive)
├── teleop/
│   ├── TurretRedTeleOp.java             turret tracking + auto-launch teleop
│   └── TeleV4.java                      simpler teleop fallback
├── util/
│   ├── MotifDecoder.java                AprilTag → motif mapping + best-start-index
│   └── MagicConstants.java              hardware constants
└── CHANGELOG.md                         ← this file
```

---

## Improvements by file

### Subsystems

#### `Spindexer.java`
- **Constructor chaining** — `Spindexer(HardwareMap)` calls `Spindexer(HardwareMap, String, String, String)` with defaults
- **Encapsulated state** — all fields private; slot manipulation via `setSlot/getSlot/resetSlots`
- **Color detection** — extracted `detectSingleSensor()` helper with named thresholds instead of opaque literals
- **Modular rotation** — `closestModularPosition()` extracted to its own method
- **Sweep shoot** — `startShootAllSweep()`/`isSweepComplete()`/`cancelSweep()` from the original preserved
- **Queries** — added `contains()`, `isEmpty()`, `isFull()`, `findClosestLoaded()`, `getSlots()`
- **Configurable delay** — `setColorDelayMs()` for tuning the sensor → rotation delay
- **No more `Spindexer.Ball` enum duplicated** — exists once in Spindexer, used by MotifDecoder

#### `Launcher.java`
- **Constructor chaining** — `Launcher(HardwareMap)` delegates to full constructor
- **PIDF constants** — extracted from opaque `new PIDFCoefficients(200, 0, 0, 17.4)` to named constants
- **All velocity constants named** — `LAUNCH_VELOCITY_HIGH`, `LAUNCH_VELOCITY_DEFAULT`, `LAUNCH_VELOCITY_IDLE`
- **Accessors** — `getFlywheelVelocity()`, `isFlywheelRunning()` for telemetry
- **Timed shoot** — `startTimedShoot(ms)` / `updateTimedShoot()`

#### `Intake.java`
- **Constructor chaining** with defaults
- **Named power constants** — `INTAKE_POWER_DEFAULT`, `INTAKE_POWER_SLOW`, `OUTTAKE_POWER_DEFAULT`, `OUTTAKE_POWER_FAST`
- **Convenience methods** — `intakeSlow()`, `outtakeFast()`, `reverseDirection()`
- **Accessors** — `isIntaking()`, `isOuttaking()`, `getTelemetryString()`

#### `MecanumDrive.java` *(new — replaces `CustomMecanumDrive`)*
- Same motor map and direction convention as the original
- `driveMecanum(strafe, drive, turn)` with auto-normalisation (no wheels exceed 1.0)
- `powerOff()`, `setZeroPowerBehavior()`, `setDirection()`

---

### Utilities

#### `MotifDecoder.java`
- **Green-position logic** — `Motif.greenPosition()` identifies which slot has the green ball in each motif
- **Better `findBestStartIndex`** — aligns spindexer green with motif green, returns -1 if no green loaded (vs. silent 0)
- **Varargs overload** — `fromTagIds(int... tagIds)` picks the first valid tag from a list
- **Javadoc** on every public method
- **Final class** + private constructor — pure utility

#### `MagicConstants.java` *(new — consolidates magic numbers)*
- `SPINDEXER_TICKS_PER_REV` — 750 (was hardcoded in both autonomous files and Spindexer)
- `TURRET_TICKS_PER_REV` — 555.0
- Red start pose constants (X, Y, heading)

---

### Paths

#### `RedClose12Paths.java`
- `START_POSE` exposed as a public constant
- Every path is a named method (shoot, toIntake1, intake1, gate, intake1ToShoot2, shoot2, …)
- Same coordinates as the original V2 set — **no behavioral change**

#### `BlueClose12Paths.java`
- `START_POSE` exposed; uses a `blue(Pose)` helper to mirror Red coordinates across X
- Same approach as Red — named methods for every path

---

### Autonomous

#### `RedClose12BallAutonomous.java`
- **Uses `Spindexer.Ball` for slots** — correctly typed (not conflated with `MotifDecoder.Motif`)
- **Proper `applyMotifToSpindexer`** — calls `MotifDecoder.findBestStartIndex()` with the actual loaded slots, not an empty array
- **Named constants** — every magic number extracted to a `static final` field at the top
- **All unused imports removed** — was importing TurretRedTeleOp, RedClose12Ball, etc.
- **Intake state machine** — `tickIntake()` extracted from inline loop; uses `COLOR_DELAY_MS` instead of `FLICK_PAUSE_MS` noise
- **Shoot sequence** — `fireOneBall()`, `moveSpindexerTo()`, `shootAll()` extracted
- **Color detection** — `detectSingleSensor()` helper; both sensors OR'd correctly
- **No `@Disabled`** — opmode is active with name "Aarush Red Close 12 Ball"
- **Approach** — conservative: on-robot behavior preserved, only cleanup applied

#### `BlueClose12BallAutonomous.java`
- Same improvements as Red: named constants, correct types, MotifDecoder integration
- `detectColor()` available (annotated `@SuppressWarnings("unused")`) for future sensor-driven intake
- Blue-specific turret sign (`TURRET_SHOOT_TICKS = -76`)
- **No `@Disabled`** — opmode is active

---

### Limelight

#### `FusedPose.java`
- **Package** updated to `AarushImprovements.limelight`
- **All unused imports removed** — was importing `@Disabled`, `@TeleOp`, `PanelsTelemetry`, `TelemetryManager`, `BezierCurve`, `BezierLine`, `BezierPoint`, `HeadingInterpolator`, `Path`, `PathChain`, `OpMode`, `Supplier`
- **Wildcard replaced** — `import com.pedropathing.control.*` → specific `KalmanFilter` and `KalmanFilterParameters` only
- **Duplicate imports removed** — LLResult, LLResultTypes, Limelight3A, List all appeared twice

---

### Teleop

#### `TurretRedTeleOp.java`
- **Package** updated; uses `MecanumDrive` from `AarushImprovements.subsystems`
- **FusedPose** imported from `AarushImprovements.limelight.FusedPose`
- **OpMode name** changed from "!!!!!TS TURRET RED SIDE" to "Aarush Turret TeleOp"
- **Group** set to "AarushImprovements" for UI grouping

#### `TeleV4.java`
- **Package** updated; `@Disabled` removed, opmode is active
- **Uses `MecanumDrive`** instead of `CustomMecanumDrive`

---

## What was NOT changed (intentionally)

| File | Reason |
|------|--------|
| `decode/pedroPathing/Constants.java` | Third-party Pedro Pathing setup; referenced, not owned |
| `decode/framework/*` | Not part of the competition code; separate July 2026 refactor |
| Original files under `decode/` | Left untouched — improvements are only in `AarushImprovements/` |
| `ColorSensorDecode.java` | Functionality subsumed by `Spindexer.detectBall()` |
| `CustomMecanumDrive.java` | Replaced by `MecanumDrive.java` in subsystems |

---

## Known limitations
- **Blue-side paths** use a `blue(Pose)` mirroring helper that flips the Red coordinates. On-robot verification is recommended since the Blue `intake1` path in the original V2 had different (simpler) logic than Red.
- **FusedPose** still depends on `mmintothedeep.odometry.pinpoint.GoBildaPinpointDriver` (third party). No refactoring was applied to the Kalman filter or GoBilda driver — only imports were cleaned.
- **Build verification** is pending (run `gradle assembleDebug` to check).
- The **Blue autonomous `shootAll()`** in the original V2 only fires once (vs. 3 times in Red). This is a known limitation of the Blue-side code from states.

---
## New improvements (Pedro Pathing 2.1.2 + pose correction)

### `build.dependencies.gradle`
- **Pedro Pathing upgraded** from `2.0.0` → `2.1.2` (6 releases ahead)
  - Includes **predictive braking** (auto-tuned braking at path endpoints — no manual PIDF)
  - `pose.mirror()` now uses correct 141.5" field length (was 144")
  - `Follower.setX()/setY()/setHeading()` available for mid-auto pose correction
  - Fixes: heading snap, `isRobotStuck()`, predictive parametric end vector
- **Pedro Telemetry** upgraded `0.0.6` → `1.0.0`
- **FullPanels** upgraded `1.0.2` → `1.0.12`
- **FTC SDK** upgraded `11.0.0` → `11.1.0`

### `RedClose12BallAutonomous.java` / `BlueClose12BallAutonomous.java`
- **Pose correction from Limelight AprilTag** — at the start of autonomous, reads corner tags (IDs 20/24) and corrects the Follower's pose if drift exceeds 3 inches (capped at 6 inches per update)
- Uses same coordinate conversion as `FusedPose.limelightToPedroPathing()`
- Constants: `POSE_CORRECTION_TOLERANCE_IN = 3.0`, `MAX_POSE_CORRECTION_IN = 6.0`
