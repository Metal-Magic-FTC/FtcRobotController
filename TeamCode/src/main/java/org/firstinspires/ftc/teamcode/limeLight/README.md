# Limelight Robot Localization Pipeline for FTC

This package provides a complete robot localization system using Limelight 3A camera and AprilTags for FIRST Tech Challenge (FTC).

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

## Quick Start

### Basic Usage (No Odometry)

```java
// In your OpMode

// 1. Initialize
LimelightLocalizer localizer = new LimelightLocalizer(hardwareMap, "limelight");

// 2. Start in your start() method
@Override
public void start() {
    localizer.start();
}

// 3. Update in loop
@Override
public void loop() {
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

### With IMU Integration

```java
// Add IMU to initialization
IMU imu = hardwareMap.get(IMU.class, "imu");
// ... configure IMU ...

// In loop, update heading before getting pose
double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
localizer.updateRobotOrientation(heading);
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
// Configure for your specific robot setup
odometry.setOffsets(-84.0, -168.0, DistanceUnit.MM);
odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
odometry.setEncoderDirections(
    GoBildaPinpointDriver.EncoderDirection.FORWARD,
    GoBildaPinpointDriver.EncoderDirection.FORWARD
);
```

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
