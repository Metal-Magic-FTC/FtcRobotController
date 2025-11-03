# Limelight Robot Localization Pipeline for FTC

This package provides a complete robot localization system using Limelight 3A camera and AprilTags for FIRST Tech Challenge (FTC).

## ⚠️ CRITICAL: Field-Centric Coordinates Requirement

**If your Limelight is returning incorrect coordinates (e.g., x decreasing as you approach an AprilTag), you MUST:**

### 1. Call `updateRobotOrientation()` BEFORE Getting Pose

```java
// WRONG - Will give wrong coordinates!
RobotPose pose = localizer.update();

// CORRECT - Update orientation first!
localizer.updateRobotOrientation(imu.getYaw());  // or odometry heading
RobotPose pose = localizer.update();
```

**Why?** The Limelight NEEDS your robot's current heading to calculate field-centric coordinates. Without it, you'll get tag-relative coordinates instead, which will appear backwards or incorrect.

### 2. Configure AprilTag Field Layout in Limelight

1. Connect to your Limelight's web interface (http://limelight.local:5801)
2. Go to the AprilTag pipeline settings
3. Load the FTC field layout for your game year
4. Verify tags are at correct positions

**Why?** The Limelight must know where AprilTags are on the field to calculate your robot's field position.

### 3. Use the Diagnostic Tool

Run the **"Limelight Coordinate Diagnostic"** OpMode to verify your setup:
- It shows ALL coordinate frames (MegaTag2, Botpose, individual tags)
- Helps identify if you're getting tag-relative vs field-centric data
- Tests with/without IMU orientation updates

## Overview

The pipeline determines the robot's position (x, y) and orientation (heading) on the FTC field by detecting AprilTags with the Limelight camera. It uses advanced algorithms to provide accurate, real-time localization.

## Components

### 1. **RobotPose.java**
A data class representing the robot's pose on the field.
- **Position**: X, Y coordinates in meters (convertible to inches)
- **Orientation**: Heading in degrees (0° = forward, counterclockwise positive)
- **Confidence**: Quality score from 0.0 to 1.0
- **Timestamp**: When the measurement was taken

### 2. **LimelightLocalizer.java**
The main localization engine.

#### Features:
- **MegaTag2 Multi-Tag Localization**: Uses multiple AprilTags simultaneously for highest accuracy
- **Single-Tag Fallback**: Works with just one visible tag
- **Confidence Scoring**: Rates pose quality based on:
  - Number of visible tags
  - Distance to tags
  - Viewing angles
  - Target size in camera view
- **Pose Filtering**: Rejects unreliable measurements
- **IMU/Odometry Integration**: Improves accuracy with robot heading
- **Statistics Tracking**: Success rates, pose counts, diagnostics

#### Localization Modes:
1. **MEGATAG2**: Multi-tag localization (most accurate, confidence ≥ 0.5)
2. **SINGLE_TAG**: Single AprilTag localization (good, confidence ≥ 0.4)
3. **STALE**: Returns last valid pose if recent enough
4. **NONE**: No valid pose available

### 3. **LimelightLocalizationDemo.java**
Basic example showing how to use the localizer.

Features:
- Simple setup with optional IMU integration
- Real-time position display
- Visual confidence indicators
- Statistics and diagnostics
- Interactive controls

### 4. **LimelightOdometryFusion.java**
Advanced example that fuses Limelight vision with GoBilda Pinpoint Odometry.

#### How Sensor Fusion Works:
1. **Odometry**: Provides continuous, high-frequency position tracking
2. **Vision**: Provides periodic absolute position corrections
3. **Fusion**: Vision corrects odometry drift, odometry provides smooth tracking

This is the **recommended approach** for competitive robots - combines the best of both sensors!

### 5. **LimelightCoordinateDiagnostic.java** 🔍
**Essential diagnostic tool** for debugging coordinate system issues.

Features:
- Displays ALL coordinate frames (MegaTag2, Botpose, tag-relative)
- Toggle between IMU and manual heading
- See real-time which data is field-centric vs tag-relative
- Verify updateRobotOrientation() is working
- Check if field layout is configured

**Use this FIRST if you're having coordinate issues!**

### 6. **NavigateToFieldCenter.java** 🤖
**Complete autonomous navigation example** using vision + odometry fusion.

Features:
- Navigates from any position to field center (0, 0, 0°)
- PID control for smooth, accurate movement
- Mecanum drive field-centric control
- Vision corrections eliminate odometry drift
- Comprehensive telemetry and diagnostics

Configuration:
- Pinpoint odometry at robot center (X=0, Y=0)
- Swingarm odometry pods
- Limelight orientation set via web interface

**See NAVIGATION_GUIDE.md for complete tuning and usage instructions**

### 7. **NavigateToFieldCenterPedro.java** 🎯 (RECOMMENDED)
**Autonomous navigation using PedroPathing's pre-tuned controllers.**

Features:
- No PID tuning required - works out of the box!
- Uses PedroPathing from `Constants.java` (already configured)
- Limelight provides vision corrections
- Smooth path following with acceleration limits
- Simpler code (~200 lines vs 450 lines)

**This is the easiest way to get started with autonomous!**

### 8. **PedroLimelightLocalizer.java** 🔧
**Utility class for easy PedroPathing + Limelight integration.**

Features:
- Simple API: `updateFollowerWithVision()` - that's it!
- Automatic coordinate conversion (meters ↔ inches)
- Configurable confidence threshold and update interval
- Statistics tracking

**See PEDROPATHING_GUIDE.md for complete integration guide**

## Quick Start

### Basic Usage (With IMU - REQUIRED!)

```java
// In your OpMode

// 1. Initialize
LimelightLocalizer localizer = new LimelightLocalizer(hardwareMap, "limelight");
IMU imu = hardwareMap.get(IMU.class, "imu");
// ... configure IMU ...

// 2. Start in your start() method
@Override
public void start() {
    localizer.start();
}

// 3. Update in loop - MUST call updateRobotOrientation() FIRST!
@Override
public void loop() {
    // CRITICAL: Update robot heading BEFORE getting pose!
    double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    localizer.updateRobotOrientation(heading);

    // Now get the pose
    RobotPose pose = localizer.update();

    if (pose != null) {
        double x = pose.getX();           // meters
        double y = pose.getY();           // meters
        double heading = pose.getHeading(); // degrees
        double confidence = pose.getConfidence();

        telemetry.addData("Position", "X: %.2f, Y: %.2f", x, y);
        telemetry.addData("Heading", "%.1f°", heading);
        telemetry.addData("Confidence", "%.2f", confidence);
    }
}
```

### Testing Without IMU (For Debugging Only)

```java
// You can test with a fixed heading for debugging
// WARNING: This only works if robot never rotates!
localizer.updateRobotOrientation(0.0);  // Assume robot faces 0°
RobotPose pose = localizer.update();
```

### With Odometry Fusion

```java
// See LimelightOdometryFusion.java for complete example

// The key steps:
// 1. Update odometry
odometry.update();

// 2. Update Limelight with current heading
localizer.updateRobotOrientation(odometry.getHeading(AngleUnit.DEGREES));

// 3. Get vision pose
RobotPose visionPose = localizer.update();

// 4. Periodically correct odometry with high-confidence vision poses
if (visionPose.getConfidence() >= 0.5) {
    odometry.setPosition(convertToOdometryPose(visionPose));
}
```

### Autonomous Navigation to Field Center

```java
// See NavigateToFieldCenter.java for complete autonomous example

@Autonomous(name = "Navigate to Center")
public class MyAuto extends LinearOpMode {
    private LimelightLocalizer localizer;
    private GoBildaPinpointDriver odometry;

    @Override
    public void runOpMode() {
        // Initialize hardware
        localizer = new LimelightLocalizer(hardwareMap, "limelight");
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Configure odometry
        odometry.setOffsets(0.0, 0.0, DistanceUnit.MM); // Center of robot
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometry.resetPosAndIMU();

        localizer.start();

        waitForStart();

        // Navigate to field center (0, 0, 0°)
        // Uses PID control with vision + odometry fusion
        // See NavigateToFieldCenter.java for full implementation
    }
}
```

**For complete autonomous navigation guide, see [NAVIGATION_GUIDE.md](NAVIGATION_GUIDE.md)**

### With PedroPathing (RECOMMENDED - No Tuning Required!)

```java
// See NavigateToFieldCenterPedro.java for complete example

@Autonomous(name = "Navigate with PedroPathing")
public class MyPedroAuto extends LinearOpMode {
    private Follower follower;
    private PedroLimelightLocalizer localizer;

    @Override
    public void runOpMode() {
        // Initialize PedroPathing (pre-tuned from Constants.java)
        follower = Constants.createFollower(hardwareMap);

        // Initialize Limelight localizer
        localizer = new PedroLimelightLocalizer(hardwareMap, follower);
        localizer.start();

        // Set initial pose from vision
        Pose initialPose = localizer.getVisionPose();
        if (initialPose != null) {
            follower.setPose(initialPose);
        }

        waitForStart();

        // Create path to field center
        Pose target = new Pose(0, 0, 0); // (x, y, heading) in inches/radians
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), target))
                .setConstantHeadingInterpolation(0)
                .build();

        follower.followPath(path);

        // Follow path with automatic vision corrections
        while (opModeIsActive() && !follower.atParametricEnd()) {
            follower.update();
            localizer.updateFollowerWithVision(); // Automatic vision correction!

            telemetry.addData("Pose", follower.getPose());
            telemetry.update();
        }
    }
}
```

**Advantages:**
- ✅ No PID tuning required (already tuned in Constants.java)
- ✅ Smooth path following with acceleration limits
- ✅ Vision corrections with one line of code
- ✅ Simpler than custom PID (~100 lines vs 450 lines)

**For complete PedroPathing integration guide, see [PEDROPATHING_GUIDE.md](PEDROPATHING_GUIDE.md)**

## Configuration

### Limelight Setup

1. **Pipeline Configuration**: Set pipeline 3 for AprilTag detection
   - In Limelight web interface, configure an AprilTag pipeline
   - Default pipeline index is 3 (configurable in code)

2. **Physical Mounting**:
   - Mount Limelight with clear view of field
   - Higher mounting position = better field view
   - Angle slightly downward if needed

3. **AprilTag Layout**:
   - Ensure AprilTags are placed per FTC field setup
   - Tags should be visible from multiple robot positions
   - See FTC game manual for official tag positions

### Localizer Configuration

```java
// Default configuration (recommended for most cases)
LimelightLocalizer localizer = new LimelightLocalizer(
    hardwareMap,
    "limelight"  // Device name in configuration
);

// Custom configuration
LimelightLocalizer localizer = new LimelightLocalizer(
    hardwareMap,
    "limelight",
    3,      // Pipeline index for AprilTags
    100,    // Poll rate in Hz
    0.3,    // Minimum confidence threshold
    500     // Max pose age in ms
);
```

### Odometry Configuration (GoBilda Pinpoint)

```java
// Example configuration - adjust for your robot
// This example: Pinpoint at center of robot with swingarm pods
odometry.setOffsets(0.0, 0.0, DistanceUnit.MM); // X and Y offsets from robot center
odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
odometry.setEncoderDirections(
    GoBildaPinpointDriver.EncoderDirection.FORWARD,
    GoBildaPinpointDriver.EncoderDirection.FORWARD
);
```

### Limelight Orientation Configuration

**Important:** Limelight orientation (mounting angle, position on robot) is configured through the Limelight web interface, NOT in code:
1. Connect to http://limelight.local:5801
2. Go to Settings
3. Set "Robot Orientation" parameters (mounting angle, height, etc.)
4. This allows the Limelight to correctly transform coordinates

## Coordinate System

### Field Coordinates
- **X-axis**: Along field length (0 at one end, positive forward)
- **Y-axis**: Along field width (0 at one side, positive left)
- **Origin**: Depends on AprilTag configuration (typically field corner or center)
- **Units**: Meters (use `.getXInches()` for inches)

### Heading
- **0°**: Robot facing forward (along positive X)
- **90°**: Robot facing left (along positive Y)
- **-90° or 270°**: Robot facing right
- **180° or -180°**: Robot facing backward

## Troubleshooting

### ⚠️ WRONG COORDINATES - X Decreases When Approaching Tag

**This is the most common issue!** Your coordinates are in **tag-relative** space instead of **field-centric** space.

**Solutions:**
1. **Call `updateRobotOrientation()` before `update()`**:
   ```java
   localizer.updateRobotOrientation(imu.getYaw());  // MUST DO THIS!
   RobotPose pose = localizer.update();
   ```

2. **Configure field layout in Limelight web interface**:
   - Go to http://limelight.local:5801
   - Load FTC field layout for your game year
   - Verify AprilTag positions match field setup

3. **Run the Diagnostic OpMode**: "Limelight Coordinate Diagnostic"
   - Shows ALL coordinate frames side-by-side
   - Verify which data is field-centric
   - Test with/without orientation updates

**How to identify:** If robot moves forward toward tag and X value decreases (instead of staying constant or changing based on field position), you're getting tag-relative coordinates.

### No Position Available
- **Check AprilTag visibility**: At least one tag must be visible
- **Check lighting**: Ensure adequate field lighting
- **Check Limelight pipeline**: Verify pipeline 3 is configured for AprilTags
- **Check hardware config**: Verify "limelight" device is in robot configuration

### Low Confidence / Jumpy Positions
- **Too far from tags**: Move closer to AprilTags (< 3m recommended)
- **Poor viewing angle**: Face tags more directly
- **Add IMU integration**: Improves accuracy significantly
- **Use odometry fusion**: Best solution for smooth, accurate tracking

### Position Drift
- **Use odometry fusion**: Vision-only can have gaps
- **Increase poll rate**: Higher frequency = more updates
- **Check tag calibration**: Ensure tags are at correct field positions

### Vision Updates Not Correcting Odometry
- **Check confidence threshold**: May need to lower `VISION_UPDATE_THRESHOLD`
- **Check update interval**: Adjust `VISION_UPDATE_INTERVAL_MS`
- **Verify heading input**: Ensure `updateRobotOrientation()` is called

## Performance Tips

### For Best Accuracy
1. ✅ Use odometry fusion (LimelightOdometryFusion)
2. ✅ Update robot heading from IMU/odometry
3. ✅ Keep AprilTags in view during autonomous
4. ✅ Use MegaTag2 mode (multiple tags visible)
5. ✅ Mount Limelight high with good field view

### For Best Performance
1. ✅ Use appropriate poll rate (100 Hz is good default)
2. ✅ Set reasonable confidence thresholds
3. ✅ Don't process vision data faster than it updates
4. ✅ Use stale pose fallback for brief occlusions

## Example Use Cases

### Autonomous Navigation
```java
RobotPose pose = localizer.update();
if (pose != null) {
    // Navigate to target position
    double targetX = 1.5; // meters
    double targetY = 1.5;

    double dx = targetX - pose.getX();
    double dy = targetY - pose.getY();
    double distance = Math.sqrt(dx * dx + dy * dy);
    double angle = Math.toDegrees(Math.atan2(dy, dx));

    // Drive toward target...
}
```

### Field-Relative TeleOp
```java
RobotPose pose = localizer.update();
if (pose != null) {
    // Transform gamepad input to field coordinates
    double fieldX = gamepad1.left_stick_x;
    double fieldY = -gamepad1.left_stick_y;

    // Rotate by robot heading to get robot-relative commands
    double robotHeadingRad = Math.toRadians(pose.getHeading());
    // ... apply rotation matrix ...
}
```

### Auto-Alignment
```java
RobotPose pose = localizer.update();
if (pose != null && limelightLocalizer.getVisibleTagCount() > 0) {
    // Align to specific tag or field position
    double targetHeading = 0.0; // face forward
    double headingError = targetHeading - pose.getHeading();

    // Apply correction...
}
```

## API Reference

### RobotPose
- `getX()` - X position in meters
- `getY()` - Y position in meters
- `getHeading()` - Heading in degrees
- `getConfidence()` - Quality score [0.0, 1.0]
- `getXInches()` / `getYInches()` - Position in inches
- `distanceTo(RobotPose other)` - Distance to another pose
- `headingDifference(RobotPose other)` - Angular difference

### LimelightLocalizer
- `start()` - Start the Limelight
- `stop()` - Stop the Limelight
- `update()` - Get latest robot pose (call in loop)
- `updateRobotOrientation(double heading)` - Provide current heading
- `getCurrentMode()` - Get current localization mode
- `getVisibleTagCount()` - Number of visible AprilTags
- `getVisibleTagIds()` - List of visible tag IDs
- `getSuccessRate()` - Percentage of successful pose estimates

## Requirements

- Limelight 3A camera
- FTC Robot Controller (v9.0+)
- AprilTags placed on field per FTC specifications
- Optional: IMU (for improved accuracy)
- Optional: GoBilda Pinpoint Odometry (for sensor fusion)

## Credits

Developed for FTC teams to improve autonomous navigation and field awareness.

Based on:
- Limelight Vision hardware and SDK
- FTC AprilTag localization concepts
- MegaTag2 multi-tag localization algorithm

## License

Use freely for FTC competition and educational purposes.
