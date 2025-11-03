package org.firstinspires.ftc.teamcode.limeLight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Demonstrates robot localization using LimelightLocalizer.
 *
 * This OpMode shows:
 * - How to initialize the localizer
 * - How to integrate with IMU for improved accuracy
 * - How to get robot position (x, y, heading)
 * - How to access diagnostic information
 *
 * Controls:
 * - Press X to reset statistics
 * - Press Y to toggle detailed diagnostics
 *
 * Display shows:
 * - Current robot position on field
 * - Localization mode (MegaTag2, Single Tag, etc.)
 * - Visible AprilTags
 * - Success rate and statistics
 */
@TeleOp(name = "Limelight Localization Demo", group = "Limelight")
public class LimelightLocalizationDemo extends OpMode {

    // Localization
    private LimelightLocalizer localizer;
    private IMU imu;

    // Configuration
    private static final String LIMELIGHT_NAME = "limelight";
    private static final boolean USE_IMU = true; // Set to false if no IMU available

    // Display options
    private boolean showDetailedDiagnostics = false;

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(50); // Update telemetry faster

        // Initialize Limelight Localizer
        // Default configuration: pipeline 3, 100Hz poll rate, 0.3 min confidence
        localizer = new LimelightLocalizer(hardwareMap, LIMELIGHT_NAME);

        // Optionally initialize IMU for improved accuracy
        if (USE_IMU) {
            try {
                imu = hardwareMap.get(IMU.class, "imu");
                IMU.Parameters parameters = new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                );
                imu.initialize(parameters);
                telemetry.addData("IMU", "Initialized");
            } catch (Exception e) {
                imu = null;
                telemetry.addData("IMU", "Not available");
            }
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Limelight", LIMELIGHT_NAME);
        telemetry.update();
    }

    @Override
    public void start() {
        // Start the Limelight
        localizer.start();
    }

    @Override
    public void loop() {
        // Update robot orientation from IMU if available
        if (imu != null) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            localizer.updateRobotOrientation(heading);
        }

        // Get latest robot pose
        RobotPose pose = localizer.update();

        // Handle gamepad inputs
        if (gamepad1.x) {
            localizer.resetStatistics();
        }
        if (gamepad1.y) {
            showDetailedDiagnostics = !showDetailedDiagnostics;
        }

        // Display telemetry
        displayTelemetry(pose);
    }

    /**
     * Displays comprehensive telemetry about robot localization.
     */
    private void displayTelemetry(RobotPose pose) {
        telemetry.addLine("=== ROBOT POSITION ===");

        if (pose != null) {
            // Display position in both meters and inches
            telemetry.addData("X Position", "%.3f m (%.1f in)", pose.getX(), pose.getXInches());
            telemetry.addData("Y Position", "%.3f m (%.1f in)", pose.getY(), pose.getYInches());
            telemetry.addData("Heading", "%.1f°", pose.getHeading());
            telemetry.addData("Confidence", "%.2f", pose.getConfidence());
            telemetry.addData("Age", "%d ms", pose.getAgeMs());

            // Visual confidence indicator
            String confidenceBar = getConfidenceBar(pose.getConfidence());
            telemetry.addData("Quality", confidenceBar);
        } else {
            telemetry.addData("Status", "NO POSITION AVAILABLE");
            telemetry.addData("", "Check AprilTag visibility");

            // Check if orientation is being updated
            if (!localizer.isOrientationUpdated()) {
                telemetry.addLine();
                telemetry.addData("⚠ WARNING", "Orientation not updated!");
                telemetry.addData("", "Call updateRobotOrientation()");
            }
        }

        telemetry.addLine();
        telemetry.addLine("=== LOCALIZATION INFO ===");

        // Show orientation update status
        if (localizer.isOrientationUpdated() && localizer.getLastOrientation() != null) {
            telemetry.addData("Orientation Updated", "✓ %.1f°", localizer.getLastOrientation());
        } else {
            telemetry.addData("Orientation Updated", "✗ NOT SET - REQUIRED!");
        }

        // Localization mode
        LimelightLocalizer.LocalizationMode mode = localizer.getCurrentMode();
        String modeStr = getModeString(mode);
        telemetry.addData("Mode", modeStr);

        // Visible AprilTags
        int tagCount = localizer.getVisibleTagCount();
        telemetry.addData("Visible Tags", tagCount);

        if (tagCount > 0) {
            StringBuilder tagIds = new StringBuilder();
            for (int id : localizer.getVisibleTagIds()) {
                if (tagIds.length() > 0) tagIds.append(", ");
                tagIds.append(id);
            }
            telemetry.addData("Tag IDs", tagIds.toString());
        }

        // Statistics
        telemetry.addLine();
        telemetry.addLine("=== STATISTICS ===");
        telemetry.addData("Success Rate", "%.1f%%", localizer.getSuccessRate() * 100);
        telemetry.addData("Valid Poses", "%d / %d",
                localizer.getValidPoseCount(), localizer.getTotalUpdates());
        telemetry.addData("MegaTag2 Poses", localizer.getMT2PoseCount());
        telemetry.addData("Single Tag Poses", localizer.getSingleTagPoseCount());

        // Detailed diagnostics (toggle with Y button)
        if (showDetailedDiagnostics) {
            displayDetailedDiagnostics();
        } else {
            telemetry.addLine();
            telemetry.addData("Controls", "X: Reset Stats | Y: Toggle Details");
        }

        telemetry.update();
    }

    /**
     * Displays additional diagnostic information.
     */
    private void displayDetailedDiagnostics() {
        telemetry.addLine();
        telemetry.addLine("=== DETAILED DIAGNOSTICS ===");

        LLResult result = localizer.getLastResult();
        if (result != null && result.isValid()) {
            telemetry.addData("TX", "%.2f°", result.getTx());
            telemetry.addData("TY", "%.2f°", result.getTy());
            telemetry.addData("TA", "%.2f%%", result.getTa());
            telemetry.addData("Pipeline", result.getPipelineIndex());
//            telemetry.addData("Latency", "%.1f ms", result.getLatency());
        }

        if (imu != null) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("IMU Heading", "%.1f°", heading);
        }

        telemetry.addLine();
        telemetry.addData("Controls", "X: Reset Stats | Y: Hide Details");
    }

    /**
     * Creates a visual confidence bar.
     */
    private String getConfidenceBar(double confidence) {
        int bars = (int) (confidence * 10);
        StringBuilder result = new StringBuilder("[");
        for (int i = 0; i < 10; i++) {
            result.append(i < bars ? "█" : "░");
        }
        result.append("]");
        return result.toString();
    }

    /**
     * Gets a human-readable string for the localization mode.
     */
    private String getModeString(LimelightLocalizer.LocalizationMode mode) {
        switch (mode) {
            case MEGATAG2:
                return "MegaTag2 (Multi-Tag) ✓✓";
            case SINGLE_TAG:
                return "Single Tag ✓";
            case STALE:
                return "Stale Data ⚠";
            case NONE:
            default:
                return "No Data ✗";
        }
    }

    @Override
    public void stop() {
        if (localizer != null) {
            localizer.stop();
        }
    }
}