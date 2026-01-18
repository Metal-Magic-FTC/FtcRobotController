package org.firstinspires.ftc.teamcode.decode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.limeLight.LimelightLocalizer;
import org.firstinspires.ftc.teamcode.limeLight.RobotPose;

/**
 * Utility class for integrating Limelight vision localization with PedroPathing.
 *
 * This class provides a simple interface to:
 * - Get vision-corrected poses in PedroPathing format (inches, radians)
 * - Update PedroPathing Follower with vision corrections
 * - Handle coordinate conversions (meters ↔ inches)
 * - Manage vision confidence and update intervals
 *
 * Usage:
 * ```java
 * PedroLimelightLocalizer localizer = new PedroLimelightLocalizer(hardwareMap, follower);
 * localizer.start();
 *
 * // In your loop:
 * localizer.updateFollowerWithVision(); // Automatically corrects pose if vision is good
 * ```
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class PedroLimelightLocalizer {

    private final LimelightLocalizer limelightLocalizer;
    private final Follower follower;

    // Vision correction settings
    private double minConfidence = 0.5;
    private long updateIntervalMs = 500;
    private long lastUpdateTime = 0;

    // Statistics
    private int correctionCount = 0;
    private double lastCorrectionDistance = 0.0;

    /**
     * Creates a PedroLimelightLocalizer with default settings.
     *
     * @param hardwareMap The hardware map from your OpMode
     * @param follower The PedroPathing Follower to update with vision corrections
     */
    public PedroLimelightLocalizer(HardwareMap hardwareMap, Follower follower) {
        this(hardwareMap, follower, "limelight");
    }

    /**
     * Creates a PedroLimelightLocalizer with custom Limelight name.
     *
     * @param hardwareMap The hardware map from your OpMode
     * @param follower The PedroPathing Follower to update with vision corrections
     * @param limelightName The name of the Limelight device in configuration
     */
    public PedroLimelightLocalizer(HardwareMap hardwareMap, Follower follower, String limelightName) {
        this.limelightLocalizer = new LimelightLocalizer(hardwareMap, limelightName);
        this.follower = follower;
    }

    /**
     * Starts the Limelight. Call this in your OpMode's start() method.
     */
    public void start() {
        limelightLocalizer.start();
    }

    /**
     * Stops the Limelight. Call this in your OpMode's stop() method if needed.
     */
    public void stop() {
        limelightLocalizer.stop();
    }

    /**
     * Updates the PedroPathing Follower's pose with vision data if available and reliable.
     * Call this in your autonomous loop for continuous vision corrections.
     *
     * This method:
     * 1. Gets current pose from Follower
     * 2. Updates Limelight with current heading (REQUIRED for field-centric!)
     * 3. Gets vision pose
     * 4. Applies correction if confidence is high enough
     *
     * @return true if correction was applied, false otherwise
     */
    public boolean updateFollowerWithVision() {
        long currentTime = System.currentTimeMillis();
        long timeSinceLastUpdate = currentTime - lastUpdateTime;

        // Check update interval
        if (timeSinceLastUpdate < updateIntervalMs) {
            return false;
        }

        // Get current pose from Follower
        Pose currentPose = follower.getPose();
        if (currentPose == null) {
            return false;
        }

        // Update Limelight with current heading (CRITICAL!)
        limelightLocalizer.updateRobotOrientation(
                Math.toDegrees(currentPose.getHeading())
        );

        // Get vision pose
        RobotPose robotPose = limelightLocalizer.update();
        if (robotPose == null || robotPose.getConfidence() < minConfidence) {
            return false;
        }

        // Convert to PedroPathing format
        Pose visionPose = convertToPedroPose(robotPose);

        // Calculate correction distance
        lastCorrectionDistance = Math.hypot(
                visionPose.getX() - currentPose.getX(),
                visionPose.getY() - currentPose.getY()
        );

        // Apply correction
        follower.setPose(visionPose);

        lastUpdateTime = currentTime;
        correctionCount++;

        return true;
    }

    /**
     * Gets the current pose from vision in PedroPathing format.
     * Returns null if no vision data is available.
     *
     * @return Pose in PedroPathing format (inches, radians), or null
     */
    public Pose getVisionPose() {
        // Update Limelight with current heading from Follower
        Pose currentPose = follower.getPose();
        if (currentPose != null) {
            limelightLocalizer.updateRobotOrientation(
                    Math.toDegrees(currentPose.getHeading())
            );
        }

        RobotPose robotPose = limelightLocalizer.update();
        if (robotPose == null) {
            return null;
        }

        return convertToPedroPose(robotPose);
    }

    /**
     * Converts RobotPose (meters, degrees) to PedroPathing Pose (inches, radians).
     */
    private Pose convertToPedroPose(RobotPose robotPose) {
        double xInches = robotPose.getX() * 39.3701; // meters to inches
        double yInches = robotPose.getY() * 39.3701;
        double headingRad = Math.toRadians(robotPose.getHeading());

        return new Pose(xInches, yInches, headingRad);
    }

    /**
     * Converts PedroPathing Pose (inches, radians) to field coordinates (meters, degrees).
     */
    public static RobotPose convertToRobotPose(Pose pedroPose) {
        double xMeters = pedroPose.getX() / 39.3701; // inches to meters
        double yMeters = pedroPose.getY() / 39.3701;
        double headingDeg = Math.toDegrees(pedroPose.getHeading());

        return new RobotPose(xMeters, yMeters, headingDeg, 1.0);
    }

    // Configuration methods

    /**
     * Sets the minimum confidence threshold for vision corrections.
     * Default: 0.5 (50%)
     */
    public void setMinConfidence(double minConfidence) {
        this.minConfidence = minConfidence;
    }

    /**
     * Sets the update interval for vision corrections.
     * Default: 500ms
     */
    public void setUpdateInterval(long intervalMs) {
        this.updateIntervalMs = intervalMs;
    }

    // Getters for diagnostics

    /**
     * Gets the underlying LimelightLocalizer for advanced usage.
     */
    public LimelightLocalizer getLimelightLocalizer() {
        return limelightLocalizer;
    }

    /**
     * Gets the number of vision corrections applied.
     */
    public int getCorrectionCount() {
        return correctionCount;
    }

    /**
     * Gets the distance of the last vision correction (in inches).
     */
    public double getLastCorrectionDistance() {
        return lastCorrectionDistance;
    }

    /**
     * Gets the number of visible AprilTags.
     */
    public int getVisibleTagCount() {
        return limelightLocalizer.getVisibleTagCount();
    }

    /**
     * Gets the current localization mode.
     */
    public LimelightLocalizer.LocalizationMode getLocalizationMode() {
        return limelightLocalizer.getCurrentMode();
    }

    /**
     * Gets the vision confidence of the last pose (0.0 to 1.0).
     * Returns 0.0 if no pose available.
     */
    public double getVisionConfidence() {
        RobotPose pose = limelightLocalizer.getLastValidPose();
        return (pose != null) ? pose.getConfidence() : 0.0;
    }

    /**
     * Resets the correction counter.
     */
    public void resetStatistics() {
        correctionCount = 0;
        lastCorrectionDistance = 0.0;
        limelightLocalizer.resetStatistics();
    }
}