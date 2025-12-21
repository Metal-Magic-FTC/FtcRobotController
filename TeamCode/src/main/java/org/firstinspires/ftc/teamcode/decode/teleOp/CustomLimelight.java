package org.firstinspires.ftc.teamcode.decode.teleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

/**
 * CustomLimelight - A robust wrapper for the Limelight 3A camera for FTC robot localization.
 *
 * Features:
 * - Pose filtering to reduce noise and outliers
 * - Configurable coordinate system conversion for FTC field
 * - Pose validation to reject invalid readings
 * - Confidence scoring based on target area and validity
 * - Support for MT1 (MegaTag) and MT2 (MegaTag2) localization
 */
public class CustomLimelight {
    private Limelight3A limelight;

    // Current filtered position
    private double xPos = 0;
    private double yPos = 0;
    private double headingPos = 0;

    // Last valid pose for fallback
    private Pose lastValidPose = null;
    private ElapsedTime timeSinceLastValidPose = new ElapsedTime();

    // Filter settings
    private static final int FILTER_SIZE = 5;
    private List<Double> xHistory = new ArrayList<>();
    private List<Double> yHistory = new ArrayList<>();
    private List<Double> headingHistory = new ArrayList<>();

    // Validation thresholds
    private static final double MAX_POSITION_JUMP = 24.0; // inches - max allowed position change per update
    private static final double MIN_TARGET_AREA = 0.1;    // minimum target area percentage for valid reading
    private static final double MAX_POSE_AGE_SECONDS = 0.5; // max time to use old pose data

    // FTC Field dimensions (in inches, standard FTC field is 144" x 144")
    private static final double FIELD_WIDTH = 144.0;
    private static final double FIELD_LENGTH = 144.0;

    // Camera mount offset from robot center (configure these for your robot)
    private double cameraOffsetX = 0; // inches forward from robot center
    private double cameraOffsetY = 0; // inches left from robot center
    private double cameraOffsetHeading = 0; // radians, camera rotation relative to robot

    // Pipeline selection
    private int currentPipeline = 0;

    // Status tracking
    private boolean isConnected = false;
    private int validReadingCount = 0;
    private int totalReadingCount = 0;

    /**
     * Creates a CustomLimelight instance with default settings.
     * Uses pipeline 0 (AprilTag detection) by default.
     */
    public CustomLimelight(HardwareMap hardwareMap) {
        this(hardwareMap, 0);
    }

    /**
     * Creates a CustomLimelight instance with a specified pipeline.
     *
     * @param hardwareMap The FTC hardware map
     * @param pipeline The pipeline index to use (0-9)
     */
    public CustomLimelight(HardwareMap hardwareMap, int pipeline) {
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");

            limelight.pipelineSwitch(pipeline);
            currentPipeline = pipeline;
            limelight.setPollRateHz(100);
            limelight.start();

            isConnected = true;
            timeSinceLastValidPose.reset();
        } catch (Exception e) {
            isConnected = false;
        }
    }

    /**
     * Sets the camera mounting offset from robot center.
     * Call this to account for camera position on your robot.
     *
     * @param forwardOffset Inches forward from robot center (positive = forward)
     * @param leftOffset Inches left from robot center (positive = left)
     * @param headingOffset Radians of camera rotation (positive = counter-clockwise)
     */
    public void setCameraOffset(double forwardOffset, double leftOffset, double headingOffset) {
        this.cameraOffsetX = forwardOffset;
        this.cameraOffsetY = leftOffset;
        this.cameraOffsetHeading = headingOffset;
    }

    /**
     * Switches the Limelight pipeline.
     *
     * @param pipeline Pipeline index (0-9)
     */
    public void setPipeline(int pipeline) {
        if (limelight != null && pipeline >= 0 && pipeline <= 9) {
            limelight.pipelineSwitch(pipeline);
            currentPipeline = pipeline;
        }
    }

    /**
     * Updates the robot's IMU heading for MegaTag2 localization.
     * Call this every loop with your IMU's heading for best results.
     *
     * @param headingDegrees Robot heading in degrees (0 = forward, positive = counter-clockwise)
     */
    public void updateRobotOrientation(double headingDegrees) {
        if (limelight != null) {
            // Limelight expects heading where 0 is forward, positive is counter-clockwise
            limelight.updateRobotOrientation(headingDegrees);
        }
    }

    /**
     * Gets the current robot pose from Limelight with filtering and validation.
     * Returns null if no valid pose is available.
     *
     * @return Pose object with x (inches), y (inches), heading (radians), or null if invalid
     */
    public Pose getLimelightPose() {
        if (!isConnected || limelight == null) {
            return getLastValidPoseIfRecent();
        }

        LLResult result = limelight.getLatestResult();
        totalReadingCount++;

        if (result == null || !result.isValid()) {
            return getLastValidPoseIfRecent();
        }

        // Get target area for confidence check
        double ta = result.getTa();
        if (ta < MIN_TARGET_AREA) {
            return getLastValidPoseIfRecent();
        }

        // Get the bot pose from MegaTag/MegaTag2 localization
        Pose3D botpose = result.getBotpose();
        if (botpose == null) {
            return getLastValidPoseIfRecent();
        }

        // Extract position (Limelight returns meters, convert to inches)
        double rawX = botpose.getPosition().x * 39.3701; // meters to inches
        double rawY = botpose.getPosition().y * 39.3701;
        double rawHeading = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

        // Apply camera offset correction
        double correctedX = rawX - (cameraOffsetX * Math.cos(rawHeading) - cameraOffsetY * Math.sin(rawHeading));
        double correctedY = rawY - (cameraOffsetX * Math.sin(rawHeading) + cameraOffsetY * Math.cos(rawHeading));
        double correctedHeading = normalizeAngle(rawHeading - cameraOffsetHeading);

        // Validate the pose
        if (!isValidPose(correctedX, correctedY, correctedHeading)) {
            return getLastValidPoseIfRecent();
        }

        // Apply filtering
        double filteredX = applyFilter(xHistory, correctedX);
        double filteredY = applyFilter(yHistory, correctedY);
        double filteredHeading = applyAngleFilter(headingHistory, correctedHeading);

        // Update stored values
        xPos = filteredX;
        yPos = filteredY;
        headingPos = filteredHeading;

        // Store as last valid pose
        lastValidPose = new Pose(filteredX, filteredY, filteredHeading);
        timeSinceLastValidPose.reset();
        validReadingCount++;

        return new Pose(filteredX, filteredY, filteredHeading);
    }

    /**
     * Gets the raw, unfiltered pose from Limelight.
     * Use this for debugging or when you need immediate response.
     *
     * @return Raw Pose or null if invalid
     */
    public Pose getRawLimelightPose() {
        if (!isConnected || limelight == null) {
            return null;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return null;
        }

        Pose3D botpose = result.getBotpose();
        if (botpose == null) {
            return null;
        }

        double x = botpose.getPosition().x * 39.3701;
        double y = botpose.getPosition().y * 39.3701;
        double heading = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

        return new Pose(x, y, heading);
    }

    /**
     * Gets pose data along with confidence metrics.
     *
     * @return PoseWithConfidence object containing pose and confidence data
     */
    public PoseWithConfidence getLimelightPoseWithConfidence() {
        if (!isConnected || limelight == null) {
            return new PoseWithConfidence(null, 0, 0, false);
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return new PoseWithConfidence(getLastValidPoseIfRecent(), 0, 0, false);
        }

        double ta = result.getTa();
        int numTargets = result.getFiducialResults().size();

        Pose pose = getLimelightPose();

        // Calculate confidence (0-1) based on target area and number of visible targets
        double confidence = Math.min(1.0, (ta / 5.0) * (numTargets / 2.0));

        return new PoseWithConfidence(pose, confidence, numTargets, true);
    }

    /**
     * Returns the last valid pose if it's recent enough.
     */
    private Pose getLastValidPoseIfRecent() {
        if (lastValidPose != null && timeSinceLastValidPose.seconds() < MAX_POSE_AGE_SECONDS) {
            return lastValidPose;
        }
        return null;
    }

    /**
     * Validates a pose reading to reject outliers.
     */
    private boolean isValidPose(double x, double y, double heading) {
        // Check if position is within field bounds (with some margin)
        double margin = 12.0; // inches
        if (Math.abs(x) > (FIELD_WIDTH / 2 + margin) || Math.abs(y) > (FIELD_LENGTH / 2 + margin)) {
            return false;
        }

        // Check for unreasonable position jumps
        if (lastValidPose != null && timeSinceLastValidPose.seconds() < MAX_POSE_AGE_SECONDS) {
            double dx = x - lastValidPose.getX();
            double dy = y - lastValidPose.getY();
            double distance = Math.sqrt(dx * dx + dy * dy);

            // Scale max jump by time elapsed (faster updates = smaller max jumps)
            double maxJump = MAX_POSITION_JUMP * Math.max(0.1, timeSinceLastValidPose.seconds() * 10);
            if (distance > maxJump) {
                return false;
            }
        }

        return true;
    }

    /**
     * Applies a median filter to reduce noise.
     */
    private double applyFilter(List<Double> history, double newValue) {
        history.add(newValue);
        if (history.size() > FILTER_SIZE) {
            history.remove(0);
        }

        // Return median of history
        List<Double> sorted = new ArrayList<>(history);
        sorted.sort(Double::compareTo);

        int mid = sorted.size() / 2;
        if (sorted.size() % 2 == 0) {
            return (sorted.get(mid - 1) + sorted.get(mid)) / 2.0;
        }
        return sorted.get(mid);
    }

    /**
     * Applies angle filtering with proper wraparound handling.
     */
    private double applyAngleFilter(List<Double> history, double newAngle) {
        // Convert to unit vectors for averaging
        double sumSin = 0;
        double sumCos = 0;

        history.add(newAngle);
        if (history.size() > FILTER_SIZE) {
            history.remove(0);
        }

        for (double angle : history) {
            sumSin += Math.sin(angle);
            sumCos += Math.cos(angle);
        }

        return Math.atan2(sumSin / history.size(), sumCos / history.size());
    }

    /**
     * Normalizes an angle to the range [-π, π].
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Clears the filter history. Call this after teleporting the robot
     * or when position tracking should reset.
     */
    public void clearFilterHistory() {
        xHistory.clear();
        yHistory.clear();
        headingHistory.clear();
        lastValidPose = null;
    }

    /**
     * Gets the current X position in inches.
     */
    public double getX() {
        return xPos;
    }

    /**
     * Gets the current Y position in inches.
     */
    public double getY() {
        return yPos;
    }

    /**
     * Gets the current heading in radians.
     */
    public double getHeading() {
        return headingPos;
    }

    /**
     * Gets the current heading in degrees.
     */
    public double getHeadingDegrees() {
        return Math.toDegrees(headingPos);
    }

    /**
     * Checks if the Limelight is connected and operational.
     */
    public boolean isConnected() {
        return isConnected;
    }

    /**
     * Gets the percentage of valid readings.
     */
    public double getValidReadingPercentage() {
        if (totalReadingCount == 0) return 0;
        return (double) validReadingCount / totalReadingCount * 100;
    }

    /**
     * Gets the current pipeline index.
     */
    public int getCurrentPipeline() {
        return currentPipeline;
    }

    /**
     * Stops the Limelight. Call this in stop() of your OpMode.
     */
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    @Override
    public String toString() {
        return String.format("Pose{x=%.2f, y=%.2f, heading=%.2f°}", xPos, yPos, Math.toDegrees(headingPos));
    }

    /**
     * Helper class to return pose with confidence metrics.
     */
    public static class PoseWithConfidence {
        public final Pose pose;
        public final double confidence; // 0-1
        public final int numTargetsVisible;
        public final boolean isCurrentReading;

        public PoseWithConfidence(Pose pose, double confidence, int numTargets, boolean isCurrent) {
            this.pose = pose;
            this.confidence = confidence;
            this.numTargetsVisible = numTargets;
            this.isCurrentReading = isCurrent;
        }
    }
}