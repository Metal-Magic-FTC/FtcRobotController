package org.firstinspires.ftc.teamcode.limeLight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.GoBildaPinpointDriver;

/**
 * Advanced localization example that fuses Limelight vision with GoBilda Pinpoint Odometry.
 *
 * This approach combines:
 * - Limelight AprilTag vision for absolute field position
 * - Pinpoint Odometry for continuous position tracking
 *
 * Benefits:
 * - Odometry provides smooth, high-frequency position updates
 * - Limelight provides periodic absolute position corrections
 * - Together they eliminate odometry drift and provide accurate real-time pose
 *
 * This is the recommended approach for competitive FTC robots.
 */
@TeleOp(name = "Limelight + Odometry Fusion", group = "Limelight")
@Disabled
public class LimelightOdometryFusion extends OpMode {

    // Localization
    private LimelightLocalizer limelightLocalizer;
    private GoBildaPinpointDriver odometry;

    // Configuration
    private static final String LIMELIGHT_NAME = "limelight";
    private static final String ODOMETRY_NAME = "odo";

    // Fusion settings
    private static final double VISION_UPDATE_THRESHOLD = 0.5; // Minimum confidence to update odometry
    private static final long VISION_UPDATE_INTERVAL_MS = 500; // Update odometry every 500ms max
    private long lastVisionUpdateTime = 0;

    // Statistics
    private int odometryUpdateCount = 0;
    private RobotPose lastVisionPose = null;

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(50);

        // Initialize Limelight Localizer
        limelightLocalizer = new LimelightLocalizer(hardwareMap, LIMELIGHT_NAME);

        // Initialize GoBilda Pinpoint Odometry
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, ODOMETRY_NAME);

        // Configure odometry - ADJUST THESE VALUES FOR YOUR ROBOT
        odometry.setOffsets(0, 0);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometry.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        odometry.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Limelight", "Ready");
        telemetry.addData("Odometry", "Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        limelightLocalizer.start();
        lastVisionUpdateTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        // Update odometry
        odometry.update();
        Pose2D odoPose = odometry.getPosition();

        // Update Limelight with current heading for better accuracy
        double currentHeading = Math.toDegrees(odometry.getHeading());
        limelightLocalizer.updateRobotOrientation(currentHeading);

        // Get latest vision pose
        RobotPose visionPose = limelightLocalizer.update();

        // Fuse vision and odometry
        fuseLocalization(visionPose, odoPose);

        // Display telemetry
        displayFusionTelemetry(visionPose, odoPose);
    }

    /**
     * Fuses Limelight vision data with odometry.
     *
     * Strategy:
     * - Use odometry for continuous high-frequency updates
     * - Periodically correct odometry with high-confidence vision poses
     * - This eliminates odometry drift while maintaining smooth tracking
     */
    private void fuseLocalization(RobotPose visionPose, Pose2D odoPose) {
        if (visionPose == null) {
            return; // No vision data, continue with odometry only
        }

        long currentTime = System.currentTimeMillis();
        long timeSinceLastUpdate = currentTime - lastVisionUpdateTime;

        // Check if we should update odometry with vision data
        boolean shouldUpdate =
                visionPose.getConfidence() >= VISION_UPDATE_THRESHOLD &&
                        timeSinceLastUpdate >= VISION_UPDATE_INTERVAL_MS;

        if (shouldUpdate) {
            // Convert vision pose to odometry format
            // Vision uses meters, odometry uses mm
            double xMm = visionPose.getX() * 1000.0;
            double yMm = visionPose.getY() * 1000.0;
            double headingRad = Math.toRadians(visionPose.getHeading());

            // Update odometry position with vision correction
            Pose2D correctedPose = new Pose2D(DistanceUnit.MM, xMm, yMm, AngleUnit.RADIANS, headingRad);
            odometry.setPosition(correctedPose);

            // Record update
            lastVisionUpdateTime = currentTime;
            odometryUpdateCount++;
            lastVisionPose = visionPose;

            telemetry.addLine("⚠ ODOMETRY CORRECTED BY VISION");
        }
    }

    /**
     * Displays comprehensive telemetry for sensor fusion.
     */
    private void displayFusionTelemetry(RobotPose visionPose, Pose2D odoPose) {
        telemetry.addLine("=== FUSED POSITION (Odometry) ===");

        // Current position (from odometry, corrected by vision)
        double xInches = odoPose.getX(DistanceUnit.INCH);
        double yInches = odoPose.getY(DistanceUnit.INCH);
        double headingDeg = odoPose.getHeading(AngleUnit.DEGREES);

        telemetry.addData("X Position", "%.1f in", xInches);
        telemetry.addData("Y Position", "%.1f in", yInches);
        telemetry.addData("Heading", "%.1f°", headingDeg);

        telemetry.addLine();
        telemetry.addLine("=== VISION (Limelight) ===");

        if (visionPose != null) {
            telemetry.addData("X Position", "%.1f in", visionPose.getXInches());
            telemetry.addData("Y Position", "%.1f in", visionPose.getYInches());
            telemetry.addData("Heading", "%.1f°", visionPose.getHeading());
            telemetry.addData("Confidence", "%.2f %s",
                    visionPose.getConfidence(),
                    getConfidenceEmoji(visionPose.getConfidence()));
            telemetry.addData("Mode", getModeString(limelightLocalizer.getCurrentMode()));
            telemetry.addData("Visible Tags", limelightLocalizer.getVisibleTagCount());

            // Show position difference if we have a previous vision pose
            if (lastVisionPose != null && visionPose != lastVisionPose) {
                double positionDiff = visionPose.distanceTo(lastVisionPose) * 39.37; // m to inches
                double headingDiff = Math.abs(visionPose.headingDifference(lastVisionPose));
                telemetry.addData("Correction", "%.1f in, %.1f°", positionDiff, headingDiff);
            }
        } else {
            telemetry.addData("Status", "NO VISION DATA");
            telemetry.addData("", "Relying on odometry only");
        }

        telemetry.addLine();
        telemetry.addLine("=== ODOMETRY ===");
        telemetry.addData("Velocity X", "%.1f in/s",
                odometry.getVelocity().getX(DistanceUnit.INCH));
        telemetry.addData("Velocity Y", "%.1f in/s",
                odometry.getVelocity().getY(DistanceUnit.INCH));
        telemetry.addData("Angular Velocity", "%.1f°/s",
                Math.toDegrees(odometry.getVelocity().getHeading(AngleUnit.RADIANS)));

        telemetry.addLine();
        telemetry.addLine("=== FUSION STATS ===");
        telemetry.addData("Vision Corrections", odometryUpdateCount);
        telemetry.addData("Time Since Last", "%d ms",
                System.currentTimeMillis() - lastVisionUpdateTime);
        telemetry.addData("Vision Success Rate", "%.1f%%",
                limelightLocalizer.getSuccessRate() * 100);

        telemetry.update();
    }

    /**
     * Gets an emoji indicator for confidence level.
     */
    private String getConfidenceEmoji(double confidence) {
        if (confidence >= 0.8) return "✓✓";
        if (confidence >= 0.6) return "✓";
        if (confidence >= 0.4) return "~";
        return "✗";
    }

    /**
     * Gets a human-readable string for the localization mode.
     */
    private String getModeString(LimelightLocalizer.LocalizationMode mode) {
        switch (mode) {
            case MEGATAG2:
                return "MegaTag2";
            case SINGLE_TAG:
                return "Single Tag";
            case STALE:
                return "Stale";
            case NONE:
            default:
                return "None";
        }
    }

    @Override
    public void stop() {
        if (limelightLocalizer != null) {
            limelightLocalizer.stop();
        }
    }
}