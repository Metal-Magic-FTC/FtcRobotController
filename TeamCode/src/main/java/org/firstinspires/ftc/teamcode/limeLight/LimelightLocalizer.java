package org.firstinspires.ftc.teamcode.limeLight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.List;

/**
 * LimelightLocalizer provides robot localization using Limelight camera and AprilTags.
 *
 * Features:
 * - MegaTag2 multi-tag localization (most accurate)
 * - Single-tag fallback localization
 * - Pose filtering and validation
 * - Confidence scoring
 * - IMU/Odometry integration for improved accuracy
 *
 * Usage:
 * 1. Initialize with hardwareMap
 * 2. Call start() in your OpMode's start() method
 * 3. Call update() in your loop() to get latest pose
 * 4. Optionally call updateRobotOrientation() with IMU/odometry heading for better accuracy
 */
public class LimelightLocalizer {

    // Hardware
    private final Limelight3A limelight;

    // Configuration
    private final int aprilTagPipeline;
    private final int pollRateHz;
    private final double minConfidence;
    private final double maxPoseAgeMs;

    // State
    private RobotPose lastValidPose;
    private LLResult lastResult;
    private LocalizationMode currentMode;

    // Statistics
    private int totalUpdates;
    private int validPoseCount;
    private int mt2PoseCount;
    private int singleTagPoseCount;

    /**
     * Localization mode used for the current pose estimate.
     */
    public enum LocalizationMode {
        NONE,           // No pose available
        MEGATAG2,       // Multi-tag localization (best)
        SINGLE_TAG,     // Single AprilTag localization
        STALE           // Using old pose data
    }

    /**
     * Creates a LimelightLocalizer with default configuration.
     *
     * @param hardwareMap The hardware map from your OpMode
     * @param limelightName The name of the Limelight device in your configuration
     */
    public LimelightLocalizer(HardwareMap hardwareMap, String limelightName) {
        this(hardwareMap, limelightName, 3, 100, 0.3, 500);
    }

    /**
     * Creates a LimelightLocalizer with custom configuration.
     *
     * @param hardwareMap The hardware map from your OpMode
     * @param limelightName The name of the Limelight device in your configuration
     * @param aprilTagPipeline Pipeline index for AprilTag detection (typically 3)
     * @param pollRateHz How often to poll the Limelight (Hz)
     * @param minConfidence Minimum confidence threshold (0.0 to 1.0)
     * @param maxPoseAgeMs Maximum age for poses to be considered valid (milliseconds)
     */
    public LimelightLocalizer(HardwareMap hardwareMap, String limelightName,
                              int aprilTagPipeline, int pollRateHz,
                              double minConfidence, double maxPoseAgeMs) {
        this.limelight = hardwareMap.get(Limelight3A.class, limelightName);
        this.aprilTagPipeline = aprilTagPipeline;
        this.pollRateHz = pollRateHz;
        this.minConfidence = minConfidence;
        this.maxPoseAgeMs = maxPoseAgeMs;

        // Initialize state
        this.lastValidPose = null;
        this.lastResult = null;
        this.currentMode = LocalizationMode.NONE;

        // Initialize statistics
        this.totalUpdates = 0;
        this.validPoseCount = 0;
        this.mt2PoseCount = 0;
        this.singleTagPoseCount = 0;

        // Configure Limelight
        limelight.setPollRateHz(pollRateHz);
        limelight.pipelineSwitch(aprilTagPipeline);
    }

    /**
     * Starts the Limelight. Call this in your OpMode's start() method.
     */
    public void start() {
        limelight.start();
    }

    /**
     * Stops the Limelight. Call this in your OpMode's stop() method if needed.
     */
    public void stop() {
        limelight.stop();
    }

    /**
     * Updates the robot's orientation in the Limelight.
     * This improves localization accuracy by providing the current heading from IMU/odometry.
     *
     * @param headingDegrees Current robot heading in degrees
     */
    public void updateRobotOrientation(double headingDegrees) {
        limelight.updateRobotOrientation(headingDegrees);
    }

    /**
     * Updates and returns the latest robot pose.
     * Call this in your loop() method to get the current position.
     *
     * @return Latest valid robot pose, or null if no valid pose is available
     */
    public RobotPose update() {
        totalUpdates++;
        lastResult = limelight.getLatestResult();

        if (lastResult == null || !lastResult.isValid()) {
            currentMode = LocalizationMode.NONE;
            return getStaleOrNullPose();
        }

        // Try MegaTag2 first (multi-tag localization - most accurate)
        RobotPose mt2Pose = getMegaTag2Pose();
        if (mt2Pose != null && mt2Pose.getConfidence() >= minConfidence) {
            lastValidPose = mt2Pose;
            currentMode = LocalizationMode.MEGATAG2;
            validPoseCount++;
            mt2PoseCount++;
            return mt2Pose;
        }

        // Fallback to single-tag localization
        RobotPose singleTagPose = getBestSingleTagPose();
        if (singleTagPose != null && singleTagPose.getConfidence() >= minConfidence) {
            lastValidPose = singleTagPose;
            currentMode = LocalizationMode.SINGLE_TAG;
            validPoseCount++;
            singleTagPoseCount++;
            return singleTagPose;
        }

        // No valid new pose available
        currentMode = LocalizationMode.NONE;
        return getStaleOrNullPose();
    }

    /**
     * Gets the robot pose using MegaTag2 algorithm (multi-tag localization).
     * This is the most accurate method when multiple tags are visible.
     */
    private RobotPose getMegaTag2Pose() {
        if (lastResult == null) return null;

        Pose3D botpose = lastResult.getBotpose_MT2();
        if (botpose == null) return null;

        // Extract position (in meters)
        double x = botpose.getPosition().x;
        double y = botpose.getPosition().y;

        // Extract orientation (yaw in degrees)
        double heading = botpose.getOrientation().getYaw();

        // Calculate confidence based on multiple factors
        double confidence = calculateMT2Confidence(lastResult);

        return new RobotPose(x, y, heading, confidence);
    }

    /**
     * Gets the best single-tag pose from all visible AprilTags.
     * Uses the tag with the highest confidence.
     */
    private RobotPose getBestSingleTagPose() {
        if (lastResult == null) return null;

        List<LLResultTypes.FiducialResult> fiducials = lastResult.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;

        RobotPose bestPose = null;
        double bestConfidence = 0.0;

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            Pose3D robotPoseFieldSpace = fiducial.getRobotPoseFieldSpace();
            if (robotPoseFieldSpace == null) continue;

            // Extract position and orientation
            double x = robotPoseFieldSpace.getPosition().x;
            double y = robotPoseFieldSpace.getPosition().y;
            double heading = robotPoseFieldSpace.getOrientation().getYaw();

            // Calculate confidence for this tag
            double confidence = calculateSingleTagConfidence(fiducial);

            // Keep the pose with highest confidence
            if (confidence > bestConfidence) {
                bestConfidence = confidence;
                bestPose = new RobotPose(x, y, heading, confidence);
            }
        }

        return bestPose;
    }

    /**
     * Calculates confidence for MegaTag2 localization based on:
     * - Number of tags detected
     * - Target area (closer tags are more reliable)
     * - Data freshness
     */
    private double calculateMT2Confidence(LLResult result) {
        double confidence = 0.5; // Base confidence

        // More tags = higher confidence
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        int tagCount = (fiducials != null) ? fiducials.size() : 0;
        if (tagCount >= 2) confidence += 0.3;
        else if (tagCount == 1) confidence += 0.1;

        // Larger target area = closer distance = higher confidence
        double ta = result.getTa();
        if (ta > 1.0) confidence += 0.2;
        else if (ta > 0.5) confidence += 0.1;

        return Math.min(1.0, confidence);
    }

    /**
     * Calculates confidence for single-tag localization based on:
     * - Tag ID (some tags may be more reliable based on placement)
     * - Distance to tag
     * - Viewing angle
     */
    private double calculateSingleTagConfidence(LLResultTypes.FiducialResult fiducial) {
        double confidence = 0.4; // Base confidence (lower than MT2)

        // Get distance from robot to tag
        Pose3D robotToTag = fiducial.getRobotPoseTargetSpace();
        if (robotToTag != null) {
            double distance = robotToTag.getPosition().z; // Z is forward distance

            // Closer tags are more reliable
            if (distance < 1.0) confidence += 0.3;      // Very close (< 1m)
            else if (distance < 2.0) confidence += 0.2; // Medium (1-2m)
            else if (distance < 3.0) confidence += 0.1; // Far (2-3m)
            // Beyond 3m, don't add confidence
        }

        // Viewing angle affects confidence
        double tx = fiducial.getTargetXDegrees();
        double ty = fiducial.getTargetYDegrees();
        double angleFromCenter = Math.sqrt(tx * tx + ty * ty);

        // Tags near center of view are more reliable
        if (angleFromCenter < 10.0) confidence += 0.1;
        else if (angleFromCenter < 20.0) confidence += 0.05;

        return Math.min(1.0, confidence);
    }

    /**
     * Returns the last valid pose if it's not too old, otherwise returns null.
     */
    private RobotPose getStaleOrNullPose() {
        if (lastValidPose != null && lastValidPose.getAgeMs() < maxPoseAgeMs) {
            currentMode = LocalizationMode.STALE;
            return lastValidPose;
        }
        return null;
    }

    // Getters for state and diagnostics

    public RobotPose getLastValidPose() {
        return lastValidPose;
    }

    public LocalizationMode getCurrentMode() {
        return currentMode;
    }

    public LLResult getLastResult() {
        return lastResult;
    }

    public int getVisibleTagCount() {
        if (lastResult == null || !lastResult.isValid()) return 0;
        List<LLResultTypes.FiducialResult> fiducials = lastResult.getFiducialResults();
        return (fiducials != null) ? fiducials.size() : 0;
    }

    public List<Integer> getVisibleTagIds() {
        List<Integer> ids = new ArrayList<>();
        if (lastResult != null && lastResult.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = lastResult.getFiducialResults();
            if (fiducials != null) {
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    ids.add(fiducial.getFiducialId());
                }
            }
        }
        return ids;
    }

    public int getTotalUpdates() {
        return totalUpdates;
    }

    public int getValidPoseCount() {
        return validPoseCount;
    }

    public int getMT2PoseCount() {
        return mt2PoseCount;
    }

    public int getSingleTagPoseCount() {
        return singleTagPoseCount;
    }

    public double getSuccessRate() {
        return (totalUpdates > 0) ? (double) validPoseCount / totalUpdates : 0.0;
    }

    /**
     * Resets all statistics counters.
     */
    public void resetStatistics() {
        totalUpdates = 0;
        validPoseCount = 0;
        mt2PoseCount = 0;
        singleTagPoseCount = 0;
    }
}
