# PedroPathing + Limelight Integration Guide

This guide explains how to use Limelight vision localization with PedroPathing's pre-tuned controllers.

## Why PedroPathing?

**Advantages:**
- ✅ **Pre-tuned PID controllers** - No tuning required!
- ✅ **Optimized for FTC** - Designed specifically for mecanum drives
- ✅ **Smooth path following** - Handles acceleration limits and curves
- ✅ **Robust to disturbances** - Better handling of collisions and obstacles
- ✅ **Active community** - Well-documented and widely used

**vs Custom PID:**
- Custom PID: You tune kP, kI, kD for X, Y, and heading
- PedroPathing: Already tuned in `Constants.java`, works out of the box!

## Components

### 1. NavigateToFieldCenterPedro.java
Complete autonomous OpMode that navigates to (0, 0, 0°) using PedroPathing.

**Features:**
- Uses PedroPathing for all movement (pre-tuned!)
- Limelight provides vision corrections
- Creates straight-line path to target
- Automatic arrival detection

### 2. PedroLimelightLocalizer.java
Utility class for easy integration.

**Features:**
- Simple API for vision corrections
- Automatic coordinate conversion (meters ↔ inches)
- Configurable confidence threshold and update interval
- Statistics tracking

## Quick Start

### Option 1: Using NavigateToFieldCenterPedro (Recommended for Learning)

```java
// Just run the OpMode!
// 1. Select "Navigate to Center (PedroPathing)" from Autonomous menu
// 2. Place robot anywhere on field
// 3. Press INIT → START
// 4. PedroPathing handles everything!
```

**That's it!** No PID tuning needed - PedroPathing is already tuned in `Constants.java`.

### Option 2: Using PedroLimelightLocalizer (Recommended for Custom Autonomous)

```java
@Autonomous(name = "My Custom Auto")
public class MyCustomAuto extends LinearOpMode {

    private Follower follower;
    private PedroLimelightLocalizer localizer;

    @Override
    public void runOpMode() {
        // Initialize PedroPathing
        follower = Constants.createFollower(hardwareMap);

        // Initialize Limelight localizer
        localizer = new PedroLimelightLocalizer(hardwareMap, follower);
        localizer.start();

        // Get initial position from vision
        Pose initialPose = localizer.getVisionPose();
        if (initialPose != null) {
            follower.setPose(initialPose);
        }

        waitForStart();

        // Create path to target
        Pose target = new Pose(0, 0, 0); // Field center
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), target))
                .setConstantHeadingInterpolation(0) // Face forward
                .build();

        follower.followPath(path);

        // Follow path with vision corrections
        while (opModeIsActive() && !follower.atParametricEnd()) {
            follower.update();
            localizer.updateFollowerWithVision(); // Automatic vision correction!

            telemetry.addData("Pose", follower.getPose());
            telemetry.addData("Vision Corrections", localizer.getCorrectionCount());
            telemetry.update();
        }

        follower.breakFollowing();
    }
}
```

## How It Works

### Sensor Fusion Architecture

```
┌──────────────────┐
│  Pinpoint Odo    │ ──> Continuous tracking (built into PedroPathing)
└──────────────────┘
         ↓
    [PedroPathing]
         ↓
┌──────────────────┐
│  Limelight       │ ──> Vision corrections every 500ms
└──────────────────┘
```

1. **PedroPathing uses Pinpoint odometry** (configured in `Constants.java`)
2. **Limelight provides periodic corrections** via `updateFollowerWithVision()`
3. **PedroPathing handles all movement** with pre-tuned controllers
4. **Result:** Accurate autonomous with no PID tuning required!

### Vision Correction Process

```java
// In your autonomous loop:
follower.update();                           // 1. Update PedroPathing
localizer.updateFollowerWithVision();        // 2. Apply vision correction

// What happens inside updateFollowerWithVision():
// 1. Gets current pose from PedroPathing
// 2. Updates Limelight with current heading (CRITICAL!)
// 3. Gets vision pose from AprilTags
// 4. If confidence ≥ 0.5, updates PedroPathing pose
// 5. Correction happens automatically!
```

## Configuration

### PedroPathing Configuration (Already Done!)

Your `Constants.java` is already configured:
```java
public static PinpointConstants localizerConstants = new PinpointConstants()
    .forwardPodY(0)              // ✓ Center of robot
    .strafePodX(0)               // ✓ Center of robot
    .encoderResolution(goBILDA_SWINGARM_POD)  // ✓ Swingarm pods
    // ... etc
```

**You don't need to change this!** It matches your robot configuration.

### Limelight Configuration (Web Interface)

**REQUIRED:** Configure in web interface (http://limelight.local:5801):
1. Load FTC field layout for your game year
2. Set robot orientation (mounting angle, height)
3. Pipeline 3 = AprilTag detection

### Vision Correction Settings (Optional)

```java
// Default settings (work for most cases):
localizer.setMinConfidence(0.5);      // 50% confidence minimum
localizer.setUpdateInterval(500);      // Correct every 500ms

// For more aggressive corrections:
localizer.setMinConfidence(0.3);      // Lower threshold
localizer.setUpdateInterval(250);      // More frequent

// For conservative corrections:
localizer.setMinConfidence(0.7);      // Higher threshold
localizer.setUpdateInterval(1000);     // Less frequent
```

## Coordinate Systems

### PedroPathing Uses Inches and Radians

```java
// Limelight: meters and degrees
RobotPose limelightPose = new RobotPose(1.0, 0.5, 45.0);

// PedroPathing: inches and radians
Pose pedroPose = new Pose(39.37, 19.69, Math.toRadians(45.0));
```

**Good news:** `PedroLimelightLocalizer` handles all conversions automatically!

### Field Coordinates

FTC field in PedroPathing (inches):
- **X-axis**: Along field length (0 to ~141 inches)
- **Y-axis**: Along field width (0 to ~141 inches)
- **Origin**: Depends on your starting position
- **Heading**: 0 = forward, counterclockwise positive (radians)

**For field center:**
```java
Pose fieldCenter = new Pose(0, 0, 0);  // If you start at center
```

## Creating Paths

### Straight Line Path

```java
Pose start = follower.getPose();
Pose end = new Pose(48, 48, Math.toRadians(90));

PathChain path = follower.pathBuilder()
        .addPath(new BezierLine(start, end))
        .setConstantHeadingInterpolation(Math.toRadians(90))
        .build();
```

### Curved Path (Bezier)

```java
Pose start = new Pose(0, 0, 0);
Pose control = new Pose(24, 24, Math.toRadians(45));
Pose end = new Pose(48, 0, Math.toRadians(90));

PathChain path = follower.pathBuilder()
        .addPath(new BezierCurve(start, control, end))
        .setLinearHeadingInterpolation(0, Math.toRadians(90))
        .build();
```

### Multiple Waypoints

```java
PathChain path = follower.pathBuilder()
        .addPath(new BezierLine(pose1, pose2))
        .addPath(new BezierLine(pose2, pose3))
        .addPath(new BezierLine(pose3, pose4))
        .setConstantHeadingInterpolation(0)
        .build();
```

## Complete Example: Multi-Waypoint Autonomous

```java
@Autonomous(name = "Multi-Waypoint Auto")
public class MultiWaypointAuto extends LinearOpMode {

    private Follower follower;
    private PedroLimelightLocalizer localizer;

    @Override
    public void runOpMode() {
        // Initialize
        follower = Constants.createFollower(hardwareMap);
        localizer = new PedroLimelightLocalizer(hardwareMap, follower);
        localizer.start();

        // Set initial pose from vision
        Pose initialPose = localizer.getVisionPose();
        if (initialPose != null) {
            follower.setPose(initialPose);
        } else {
            follower.setPose(new Pose(0, 0, 0)); // Default
        }

        waitForStart();

        // Navigate to multiple waypoints
        navigateToPoint(24, 24, 0);      // Point 1
        sleep(1000);                      // Pause

        navigateToPoint(48, 24, Math.toRadians(90));  // Point 2
        sleep(1000);

        navigateToPoint(0, 0, 0);         // Return to start
    }

    private void navigateToPoint(double x, double y, double heading) {
        Pose target = new Pose(x, y, heading);

        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), target))
                .setConstantHeadingInterpolation(heading)
                .build();

        follower.followPath(path);

        while (opModeIsActive() && !follower.atParametricEnd()) {
            follower.update();
            localizer.updateFollowerWithVision();

            telemetry.addData("Target", "X: %.1f, Y: %.1f", x, y);
            telemetry.addData("Current", follower.getPose());
            telemetry.addData("Corrections", localizer.getCorrectionCount());
            telemetry.update();
        }

        follower.breakFollowing();
    }
}
```

## Troubleshooting

### Robot Doesn't Move

**Check:**
1. `follower.update()` is being called in loop
2. `follower.followPath(path)` was called
3. Path is not zero length
4. Motors configured correctly in `Constants.java`

### Jerky Movement from Vision Corrections

**Solutions:**
1. Increase update interval:
   ```java
   localizer.setUpdateInterval(1000); // 1 second
   ```
2. Increase confidence threshold:
   ```java
   localizer.setMinConfidence(0.7);
   ```
3. Check Limelight mounting is stable

### Path Not Reaching Target

**Causes:**
- Path timeout (default: 15 seconds)
- Target unreachable
- Obstacles blocking path

**Fix:**
```java
// Check if path completed
if (follower.atParametricEnd()) {
    telemetry.addData("Status", "Path complete");
} else {
    telemetry.addData("Status", "Still following or timed out");
}
```

### No Vision Corrections

**Check:**
1. AprilTags visible to Limelight
2. Field layout loaded in Limelight web interface
3. Call `updateFollowerWithVision()` in loop
4. Check telemetry:
   ```java
   telemetry.addData("Corrections", localizer.getCorrectionCount());
   telemetry.addData("Visible Tags", localizer.getVisibleTagCount());
   telemetry.addData("Confidence", localizer.getVisionConfidence());
   ```

### Wrong Coordinate System

**Problem:** Robot moves in wrong direction

**Solution:** Verify you're using PedroPathing coordinates (inches, radians):
```java
// WRONG (Limelight coordinates)
Pose target = new Pose(1.0, 0.5, 45.0);

// CORRECT (PedroPathing coordinates)
Pose target = new Pose(39.37, 19.69, Math.toRadians(45.0));
```

## Advantages vs Custom PID Navigation

| Feature | Custom PID | PedroPathing + Limelight |
|---------|-----------|-------------------------|
| **Setup Time** | 1-2 hours tuning | 5 minutes (already tuned!) |
| **PID Tuning** | Required (kP, kI, kD for X, Y, heading) | None (pre-tuned) |
| **Path Following** | Basic point-to-point | Smooth curves and lines |
| **Acceleration Limits** | Manual implementation | Built-in |
| **Code Complexity** | ~400 lines | ~100 lines |
| **Obstacle Handling** | Basic | Better (robust controllers) |
| **Learning Curve** | High (PID theory required) | Low (works out of the box) |

## Performance Tips

### For Best Accuracy
1. ✅ Keep AprilTags in view during entire path
2. ✅ Use vision correction (don't disable it!)
3. ✅ Configure Limelight orientation in web interface
4. ✅ Start autonomous with vision pose, not odometry

### For Fastest Navigation
1. ✅ Use `BezierLine` for straight paths (faster than curves)
2. ✅ Minimize waypoints (fewer stops = faster)
3. ✅ Increase path constraints in `Constants.java` (if stable)

### For Smoothest Movement
1. ✅ Reduce vision update frequency (500-1000ms)
2. ✅ Use higher confidence threshold (0.6-0.8)
3. ✅ Use curved paths instead of sharp corners

## Customization

### Changing Target Position

```java
// Navigate to different field position
private static final double TARGET_X = 48.0;      // 48 inches
private static final double TARGET_Y = 36.0;      // 36 inches
private static final double TARGET_HEADING = Math.toRadians(90.0); // 90°
```

### Using Different Path Types

```java
import com.pedropathing.geometry.*;

// Straight line
new BezierLine(start, end)

// Smooth curve (one control point)
new BezierCurve(start, control, end)

// Complex curve (two control points)
new BezierCurve(start, control1, control2, end)
```

### Adjusting Speed

Modify in `Constants.java`:
```java
public static PathConstraints pathConstraints = new PathConstraints(
    0.99,  // baseVel (max velocity)
    100,   // baseTurnVel
    1,     // trackWidth
    1      // other params
);
```

## Key Takeaways

1. **PedroPathing is already tuned** - Use it instead of custom PID!

2. **Vision corrections are automatic** - Just call `updateFollowerWithVision()`

3. **Coordinate conversion is handled** - Use inches and radians for PedroPathing

4. **Pre-configured for your robot** - Constants.java matches your setup

5. **Start with NavigateToFieldCenterPedro** - See how it works, then customize

## Further Reading

- [PedroPathing Documentation](https://pedro-path-generator.vercel.app/docs)
- [Limelight FTC Documentation](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming)
- Your `Constants.java` - Already configured for your robot!

Happy autonomous programming! 🤖🎯
