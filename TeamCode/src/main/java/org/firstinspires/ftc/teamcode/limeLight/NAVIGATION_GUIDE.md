# Autonomous Navigation Guide

This guide explains how to use the Limelight localization system for autonomous navigation.

## NavigateToFieldCenter OpMode

The `NavigateToFieldCenter` OpMode demonstrates complete autonomous navigation using:
- **Limelight vision** for absolute position correction
- **GoBilda Pinpoint odometry** for continuous position tracking
- **PID control** for smooth, accurate movement
- **Mecanum drive** for omnidirectional field-centric control

### Target Position

The robot navigates to **(0, 0, 0°)** which represents:
- **X = 0 meters**: Field center along length
- **Y = 0 meters**: Field center along width
- **Heading = 0°**: Facing forward

## How It Works

### 1. Sensor Fusion
```
┌─────────────┐
│  Odometry   │ ──> Continuous high-frequency position tracking
└─────────────┘
       ↓
   [Fusion]  <── Vision corrections every 500ms
       ↓
┌─────────────┐
│  Limelight  │ ──> Absolute field position (when tags visible)
└─────────────┘
```

**Benefits:**
- Odometry eliminates vision gaps (no AprilTags visible)
- Vision eliminates odometry drift
- Result: Accurate, robust localization

### 2. Navigation Loop

```
1. Update odometry position
2. Update Limelight with current heading (CRITICAL!)
3. Get vision pose
4. Correct odometry if vision confidence is high
5. Calculate errors (current vs target)
6. PID controllers calculate velocities
7. Drive field-centric to target
8. Repeat until target reached
```

### 3. Field-Centric Control

The robot moves relative to the **field**, not itself:
- Push joystick forward → robot moves toward positive X (regardless of heading)
- Push joystick left → robot moves toward positive Y (regardless of heading)

This is implemented by:
1. PID outputs velocities in field coordinates
2. Transform to robot coordinates using current heading
3. Apply mecanum drive calculations

## Configuration

### Robot Hardware Configuration

Ensure your robot configuration file has:
```java
// In robot configuration (via Driver Station app):
- limelight (Limelight3A)
- odo (GoBildaPinpointDriver)
- frontLeft (DcMotor)
- frontRight (DcMotor)
- backLeft (DcMotor)
- backRight (DcMotor)
```

### Odometry Setup

The OpMode is configured for:
- **Pinpoint location**: Center of robot
- **X offset**: 0.0 mm
- **Y offset**: 0.0 mm
- **Pod type**: Swingarm pods

If your configuration is different, modify `initializeHardware()`:
```java
odometry.setOffsets(YOUR_X, YOUR_Y, DistanceUnit.MM);
odometry.setEncoderResolution(YOUR_POD_TYPE);
```

### Motor Direction Setup

Default configuration:
```java
frontLeft.setDirection(DcMotor.Direction.REVERSE);
backLeft.setDirection(DcMotor.Direction.REVERSE);
frontRight.setDirection(DcMotor.Direction.FORWARD);
backRight.setDirection(DcMotor.Direction.FORWARD);
```

**Test and adjust for your robot!** If the robot drives in wrong directions:
1. Run the OpMode
2. Observe which direction it tries to move
3. Reverse motor directions as needed

### Limelight Configuration

**CRITICAL:** Configure in Limelight web interface (http://limelight.local:5801):

1. **AprilTag Pipeline** (usually pipeline 3):
   - Enable AprilTag detection
   - Load FTC field layout for your game year
   - Verify tag positions match your field

2. **Robot Orientation**:
   - Set Limelight mounting position on robot
   - Set mounting angle (tilt from horizontal)
   - Set height above ground
   - This is NOT configured in code - must be done in web interface!

3. **Validation**:
   - Run "Limelight Coordinate Diagnostic" OpMode
   - Verify field-centric coordinates look correct
   - Check that MegaTag2 works when multiple tags visible

## PID Tuning

The OpMode uses three PID controllers:

### Position Controllers (X and Y)

**Default values:**
```java
xController = new PIDController(1.5, 0.0, 0.1);
yController = new PIDController(1.5, 0.0, 0.1);
```

**Tuning guide:**

1. **Start with P only** (kI=0, kD=0):
   - Increase kP until robot oscillates around target
   - Reduce kP by 30-50%

2. **Add D term** to reduce oscillation:
   - Start with kD = kP / 10
   - Increase until oscillation stops

3. **Add I term** if needed (usually not required):
   - Only if robot stops before reaching target
   - Start very small (0.01)

**Symptoms:**
- **Oscillates back and forth**: kP too high or kD too low
- **Slow to reach target**: kP too low
- **Overshoots target**: kD too low
- **Stops before target**: Add small kI term

### Heading Controller

**Default values:**
```java
headingController = new PIDController(0.02, 0.0, 0.005);
```

**Note:** Heading uses different scale because:
- Position error is in meters (0.0 to ~7.0)
- Heading error is in degrees (-180 to 180)

**Tuning:**
- If robot spins too aggressively: Reduce kP
- If robot turns too slowly: Increase kP
- If heading oscillates: Reduce kP or increase kD

## Speed Limits

```java
MAX_DRIVE_SPEED = 0.6;    // Maximum translation speed (60% power)
MAX_TURN_SPEED = 0.5;     // Maximum rotation speed (50% power)
MIN_DRIVE_SPEED = 0.1;    // Minimum speed to overcome friction
```

**Adjustments:**
- **Robot too fast/jerky**: Reduce MAX speeds
- **Robot too slow**: Increase MAX speeds
- **Robot stalls near target**: Reduce MIN_DRIVE_SPEED

## Tolerances

```java
POSITION_TOLERANCE = 0.05;  // 5cm
HEADING_TOLERANCE = 2.0;    // 2 degrees
SETTLE_TIME_SEC = 0.5;      // Must stay in tolerance for 0.5 sec
```

**Purpose:**
- Robot must be within tolerance for SETTLE_TIME to finish
- Prevents false "arrived" detection from oscillation

**Adjustments:**
- **For more precise positioning**: Reduce tolerances (warning: may take longer)
- **For faster autonomous**: Increase tolerances
- **If robot bounces in/out of tolerance**: Increase SETTLE_TIME

## Vision Correction Settings

```java
VISION_CONFIDENCE_THRESHOLD = 0.5;  // Minimum confidence to use vision
VISION_UPDATE_INTERVAL_MS = 500;    // Update odometry every 500ms max
```

**How it works:**
- Odometry tracks position continuously
- Every 500ms, if vision confidence ≥ 0.5, odometry is corrected
- This eliminates odometry drift while maintaining smooth tracking

**Adjustments:**
- **Vision corrections too frequent/jumpy**: Increase INTERVAL or THRESHOLD
- **Odometry drifting**: Decrease INTERVAL or THRESHOLD
- **Poor vision data**: Increase THRESHOLD

## Troubleshooting

### Robot Doesn't Move

**Check:**
1. Motors connected and configured correctly
2. Motor directions set correctly
3. No gamepad safety blocking autonomous (make sure gamepad is disconnected or start button was pressed)
4. Check telemetry - are errors being calculated?

**Test:** Manually set motor powers in code to verify motors work

### Robot Moves Wrong Direction

**Fix:** Reverse motor directions in `initializeHardware()`

Example:
```java
// If robot drives backward when should go forward:
frontLeft.setDirection(DcMotor.Direction.FORWARD);  // Was REVERSE
frontRight.setDirection(DcMotor.Direction.REVERSE); // Was FORWARD
```

### Robot Oscillates Around Target

**Cause:** PID kP too high or kD too low

**Fix:**
1. Reduce kP by 30%
2. If still oscillating, increase kD
3. Re-tune in small increments

### Robot Stops Before Reaching Target

**Cause:** Friction or MIN_DRIVE_SPEED too high

**Fix:**
1. Reduce MIN_DRIVE_SPEED
2. Add small kI term (0.01) to overcome steady-state error

### Vision Corrections Cause Jerky Movement

**Cause:** Vision updates too frequent or odometry jumping too much

**Fix:**
1. Increase VISION_UPDATE_INTERVAL_MS (try 1000ms)
2. Increase VISION_CONFIDENCE_THRESHOLD (try 0.7)
3. Check Limelight is stable (not loose mounting)

### Robot Spins But Doesn't Translate

**Cause:** Heading controller overpowering position controllers

**Fix:**
1. Reduce heading controller kP
2. Reduce MAX_TURN_SPEED

### Timeout Reached Before Target

**Causes:**
- PID gains too low (robot moving too slowly)
- Target unreachable (blocked path)
- Tolerances too tight

**Fix:**
1. Check telemetry for error values - are they decreasing?
2. Increase NAVIGATION_TIMEOUT_SEC for testing
3. Tune PID for faster response
4. Widen tolerances if needed

### No Vision Correction

**Check:**
1. AprilTags visible to Limelight
2. Run "Limelight Coordinate Diagnostic" to verify vision works
3. Check vision confidence in telemetry
4. Verify field layout loaded in Limelight web interface

## Customizing Target Position

To navigate to a different position, change the constants:

```java
private static final double TARGET_X = 1.5;      // 1.5 meters along X
private static final double TARGET_Y = -1.0;     // -1.0 meters along Y
private static final double TARGET_HEADING = 90.0; // Face left (90°)
```

## Advanced: Creating Your Own Autonomous

Use `NavigateToFieldCenter` as a template:

```java
@Autonomous(name = "My Custom Auto")
public class MyCustomAuto extends LinearOpMode {

    // Copy initialization code from NavigateToFieldCenter

    @Override
    public void runOpMode() {
        initializeHardware();
        initializePIDControllers();

        waitForStart();

        // Navigate to multiple waypoints
        navigateToPosition(1.0, 0.0, 0.0);   // First position
        navigateToPosition(1.0, 1.0, 90.0);  // Second position
        navigateToPosition(0.0, 0.0, 0.0);   // Return to center
    }

    private void navigateToPosition(double x, double y, double heading) {
        // Copy navigation loop from NavigateToFieldCenter
        // Modify TARGET_X, TARGET_Y, TARGET_HEADING to use parameters
    }
}
```

## Performance Tips

### For Best Accuracy
1. ✅ Tune PID controllers for your robot
2. ✅ Ensure good AprilTag visibility throughout path
3. ✅ Use vision correction (don't disable it)
4. ✅ Test on actual field with actual field layout
5. ✅ Configure Limelight orientation accurately in web interface

### For Fastest Navigation
1. ✅ Increase MAX speeds (but keep controllable)
2. ✅ Widen tolerances slightly
3. ✅ Reduce SETTLE_TIME (but prevent false arrivals)
4. ✅ Optimize path to keep AprilTags in view

### For Smoothest Movement
1. ✅ Use D term in PID controllers
2. ✅ Reduce MAX speeds
3. ✅ Gradual acceleration/deceleration (add ramping)
4. ✅ Increase vision update interval

## Example Telemetry Output

```
=== NAVIGATION STATUS ===
Time: 3.2 / 15.0 sec

Current X: 0.523 m
Current Y: -0.341 m
Current Heading: 12.3°

Target X: 0.000 m
Target Y: 0.000 m
Target Heading: 0.0°

Error X: -0.523 m
Error Y: 0.341 m
Error Heading: -12.3°
Distance to Target: 0.623 m

Vision Available: YES
Vision Confidence: 0.87
Vision Mode: MEGATAG2
Visible Tags: 2
```

## Key Takeaways

1. **Always call `updateRobotOrientation()` before getting vision pose** - This is critical for field-centric coordinates!

2. **Vision + Odometry fusion is powerful** - Combines best of both sensors

3. **PID tuning is essential** - Default values are a starting point, tune for your robot

4. **Test incrementally** - Start with slow speeds, increase as you tune

5. **Monitor telemetry** - Watch errors, vision confidence, and time

6. **Configure Limelight properly** - Field layout and robot orientation MUST be set in web interface

## Further Reading

- [Limelight FTC Documentation](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming)
- [MegaTag2 Documentation](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2)
- FTC Field Coordinate System (see game manual)
- PID Control Theory

Good luck with your autonomous navigation! 🤖
