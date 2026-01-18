package org.firstinspires.ftc.teamcode.limeLight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.GoBildaPinpointDriver;

import java.util.List;

/**
 * Diagnostic tool to understand Limelight coordinate systems and debug localization issues.
 *
 * This OpMode displays ALL available coordinate frames from the Limelight:
 * - MegaTag2 (MT2) - Multi-tag field-centric localization
 * - Botpose - Standard field-centric localization
 * - Individual tag poses in multiple reference frames
 *
 * Use this to:
 * 1. Verify your Limelight has field layout configured
 * 2. Check if updateRobotOrientation() is working correctly
 * 3. See which coordinate frames give correct field-centric data
 * 4. Debug why positions might be in wrong reference frame
 *
 * Configuration:
 * - Uses GoBilda Pinpoint odometry for IMU data
 * - Pinpoint at center of robot (X=0, Y=0)
 * - Swingarm odometry pods
 *
 * Controls:
 * - Press A to toggle Pinpoint IMU usage (for updateRobotOrientation)
 * - Press B to manually set robot heading to 0°
 * - Press X to manually set robot heading to 90°
 * - Press Y to manually set robot heading to 180°
 * - Press DPAD_UP to initialize Pinpoint with MegaTag2 absolute heading
 * - Press DPAD_DOWN to show MegaTag2 heading WITHOUT updateRobotOrientation
 *
 * IMPORTANT: This diagnostic helps verify the "odometry reset" fix where we
 * initialize Pinpoint heading using absolute heading from MegaTag2.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@TeleOp(name = "Limelight Coordinate Diagnostic", group = "Diagnostic")
//@Disabled
public class LimelightCoordinateDiagnostic extends OpMode {

    private Limelight3A limelight;
    private GoBildaPinpointDriver odometry;

    private boolean usePinpointIMU = false;
    private double manualHeading = 0.0;
    private boolean showAbsoluteHeading = false; // Show MT2 heading without updateRobotOrientation

    private boolean lastAPress = false;
    private boolean lastBPress = false;
    private boolean lastXPress = false;
    private boolean lastYPress = false;
    private boolean lastDpadUpPress = false;
    private boolean lastDpadDownPress = false;

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(50);

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3); // AprilTag pipeline
        limelight.setPollRateHz(100);

        // Try to initialize GoBilda Pinpoint Odometry
        try {
            odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

            // Configure odometry - Pinpoint at center of robot with swingarm pods
            odometry.setOffsets(0.0, 0.0); // Center of robot
            odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
            odometry.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );
            odometry.resetPosAndIMU();

            telemetry.addData("Pinpoint IMU", "Available (press A to enable)");
            telemetry.addData("Odometry Config", "Center mount, swingarm pods");
        } catch (Exception e) {
            odometry = null;
            telemetry.addData("Pinpoint IMU", "Not available");
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("", "Point camera at AprilTag");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        // Update odometry if available
        if (odometry != null) {
            odometry.update();
        }

        handleGamepadInput();

        // Get current heading
        double currentHeading = getCurrentHeading();

        // Get latest result
        LLResult result;

        if (showAbsoluteHeading) {
            // Special mode: Get MegaTag2 WITHOUT calling updateRobotOrientation
            // This shows the ABSOLUTE field heading that MegaTag2 detects
            result = limelight.getLatestResult();
        } else {
            // Normal mode: Update robot orientation BEFORE getting results
            limelight.updateRobotOrientation(currentHeading);
            result = limelight.getLatestResult();
        }

        if (result == null || !result.isValid()) {
            displayNoData(currentHeading);
            return;
        }

        displayDiagnostics(result, currentHeading);
    }

    private void handleGamepadInput() {
        // Toggle Pinpoint IMU usage
        if (gamepad1.a && !lastAPress && odometry != null) {
            usePinpointIMU = !usePinpointIMU;
        }
        lastAPress = gamepad1.a;

        // Set manual headings
        if (gamepad1.b && !lastBPress) {
            manualHeading = 0.0;
            usePinpointIMU = false;
        }
        lastBPress = gamepad1.b;

        if (gamepad1.x && !lastXPress) {
            manualHeading = 90.0;
            usePinpointIMU = false;
        }
        lastXPress = gamepad1.x;

        if (gamepad1.y && !lastYPress) {
            manualHeading = 180.0;
            usePinpointIMU = false;
        }
        lastYPress = gamepad1.y;

        // Initialize Pinpoint with MegaTag2 absolute heading
        if (gamepad1.dpad_up && !lastDpadUpPress && odometry != null) {
            initializePinpointFromMegaTag2();
        }
        lastDpadUpPress = gamepad1.dpad_up;

        // Toggle showing absolute MT2 heading (without updateRobotOrientation)
        if (gamepad1.dpad_down && !lastDpadDownPress) {
            showAbsoluteHeading = !showAbsoluteHeading;
        }
        lastDpadDownPress = gamepad1.dpad_down;
    }

    private double getCurrentHeading() {
        if (usePinpointIMU && odometry != null) {
            return Math.toDegrees(odometry.getHeading());
        }
        return manualHeading;
    }

    private void displayNoData(double currentHeading) {
        telemetry.addLine("═══ ROBOT HEADING ═══");
        telemetry.addData("Source", usePinpointIMU ? "Pinpoint IMU" : "Manual");
        telemetry.addData("Heading", "%.1f°", currentHeading);
        telemetry.addLine();
        telemetry.addData("STATUS", "❌ NO APRILTAG DETECTED");
        telemetry.addLine();
        telemetry.addLine("Point camera at an AprilTag to see data");
        telemetry.addLine();
        displayControls();
        telemetry.update();
    }

    private void displayDiagnostics(LLResult result, double currentHeading) {
        telemetry.addLine("═══ ROBOT HEADING ═══");
        telemetry.addData("Source", usePinpointIMU ? "Pinpoint IMU" : "Manual");
        telemetry.addData("Heading", "%.1f°", currentHeading);

        // Show absolute heading mode status
        if (showAbsoluteHeading) {
            telemetry.addData("⚠ MODE", "Absolute MT2 (no updateRobotOrientation)");
        } else {
            telemetry.addData("MODE", "Normal (with updateRobotOrientation)");
        }

        // Show odometry position if available
        if (odometry != null) {
            telemetry.addData("Odo X", "%.1f mm", odometry.getPosition().getX(DistanceUnit.MM));
            telemetry.addData("Odo Y", "%.1f mm", odometry.getPosition().getY(DistanceUnit.MM));
        }

        telemetry.addLine();
        telemetry.addLine("═══ MEGATAG2 (FIELD-CENTRIC) ═══");
        displayMegaTag2(result, currentHeading);

        telemetry.addLine();
        telemetry.addLine("═══ STANDARD BOTPOSE (FIELD-CENTRIC) ═══");
        displayBotpose(result);

        telemetry.addLine();
        telemetry.addLine("═══ INDIVIDUAL TAG DATA ═══");
        displayTagData(result);

        telemetry.addLine();
        displayControls();

        telemetry.update();
    }

    private void displayMegaTag2(LLResult result, double currentHeading) {
        Pose3D mt2 = result.getBotpose_MT2();
        if (mt2 != null) {
            double x = mt2.getPosition().x;
            double y = mt2.getPosition().y;
            double z = mt2.getPosition().z;
            double yaw = mt2.getOrientation().getYaw();

            telemetry.addData("X (meters)", "%.3f", x);
            telemetry.addData("Y (meters)", "%.3f", y);
            telemetry.addData("Z (meters)", "%.3f", z);
            telemetry.addData("MT2 Yaw", "%.1f°", yaw);

            // Show heading comparison
            if (showAbsoluteHeading) {
                telemetry.addData("⚠ NOTE", "This is ABSOLUTE heading (no updateRobotOrientation)");
            } else {
                double headingDiff = Math.abs(yaw - currentHeading);
                if (headingDiff > 180) headingDiff = 360 - headingDiff;

                telemetry.addData("Robot Heading", "%.1f°", currentHeading);
                telemetry.addData("Heading Diff", "%.1f°", headingDiff);

                if (headingDiff > 5.0) {
                    telemetry.addData("⚠ WARNING", "Large heading difference!");
                    telemetry.addData("", "Odometry may not be initialized correctly");
                }
            }

            telemetry.addData("Status", "✅ AVAILABLE");

            // Check if values look reasonable
            if (Math.abs(x) > 10 || Math.abs(y) > 10) {
                telemetry.addData("⚠ WARNING", "Values seem unreasonable!");
                telemetry.addData("", "Check field layout config");
            }
        } else {
            telemetry.addData("Status", "❌ NULL");
            telemetry.addData("Reason", "May need multiple tags visible");
        }
    }

    private void displayBotpose(LLResult result) {
        Pose3D botpose = result.getBotpose();
        if (botpose != null) {
            double x = botpose.getPosition().x;
            double y = botpose.getPosition().y;
            double yaw = botpose.getOrientation().getYaw();

            telemetry.addData("X (meters)", "%.3f", x);
            telemetry.addData("Y (meters)", "%.3f", y);
            telemetry.addData("Yaw (degrees)", "%.1f°", yaw);
            telemetry.addData("Status", "✅ AVAILABLE");
        } else {
            telemetry.addData("Status", "❌ NULL");
        }
    }

    private void displayTagData(LLResult result) {
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        if (fiducials == null || fiducials.isEmpty()) {
            telemetry.addData("Tags Visible", "0");
            return;
        }

        telemetry.addData("Tags Visible", fiducials.size());

        // Show first tag in detail
        LLResultTypes.FiducialResult tag = fiducials.get(0);
        telemetry.addData("Tag ID", tag.getFiducialId());

        // Robot pose in field space (SHOULD be field-centric)
        Pose3D robotField = tag.getRobotPoseFieldSpace();
        if (robotField != null) {
            telemetry.addLine("Robot→Field:");
            telemetry.addData("  X", "%.3f m", robotField.getPosition().x);
            telemetry.addData("  Y", "%.3f m", robotField.getPosition().y);
            telemetry.addData("  Yaw", "%.1f°", robotField.getOrientation().getYaw());
        } else {
            telemetry.addData("Robot→Field", "❌ NULL (field layout not configured?)");
        }

        // Robot pose relative to tag (robot-centric, NOT field-centric)
        Pose3D robotTarget = tag.getRobotPoseTargetSpace();
        if (robotTarget != null) {
            telemetry.addLine("Robot→Tag (relative):");
            telemetry.addData("  X", "%.3f m (forward)", robotTarget.getPosition().x);
            telemetry.addData("  Y", "%.3f m (left)", robotTarget.getPosition().y);
            telemetry.addData("  Z", "%.3f m (distance)", robotTarget.getPosition().z);
            telemetry.addData("Pitch: ", tag.getTargetYDegrees());
            telemetry.addData("Yaw: ", tag.getTargetXDegrees());

            // Check if this is being used incorrectly as field coordinates
            if (robotTarget.getPosition().z < 3.0) {
                telemetry.addData("⚠ NOTE", "Z decreases as you approach tag");
                telemetry.addData("", "This is TAG-RELATIVE, not field!");
            }
        }
    }

    /**
     * Initialize Pinpoint odometry with absolute heading from MegaTag2.
     * This demonstrates the fix for the "odometry reset" issue.
     */
    private void initializePinpointFromMegaTag2() {
        if (odometry == null) return;

        // Get MegaTag2 data WITHOUT calling updateRobotOrientation
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addData("Error", "No vision data available");
            telemetry.update();
            return;
        }

        org.firstinspires.ftc.robotcore.external.navigation.Pose2D visionPose2D;
        Pose3D mt2 = result.getBotpose_MT2();
        if (mt2 != null) {
            double xMM = mt2.getPosition().x * 1000.0; // meters to mm
            double yMM = mt2.getPosition().y * 1000.0;
            double headingDeg = mt2.getOrientation().getYaw();
            double headingRad = Math.toRadians(headingDeg);

            // Set Pinpoint position and heading
            odometry.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                    DistanceUnit.MM, xMM, yMM,
                    AngleUnit.RADIANS, headingRad
            ));
            odometry.update();

            telemetry.addData("✓ Pinpoint Initialized", "From MegaTag2");
            telemetry.addData("  Position", "X: %.0fmm, Y: %.0fmm", xMM, yMM);
            telemetry.addData("  Heading", "%.1f°", headingDeg);
            telemetry.update();

            // Enable Pinpoint IMU mode
            usePinpointIMU = true;
        } else {
            telemetry.addData("Error", "MegaTag2 not available");
            telemetry.addData("", "Need multiple AprilTags visible");
            telemetry.update();
        }
    }

    private void displayControls() {
        telemetry.addLine("─── CONTROLS ───");
        if (odometry != null) {
            telemetry.addData("A", usePinpointIMU ? "Using Pinpoint IMU ✓" : "Use Pinpoint IMU");
            telemetry.addData("DPAD ↑", "Init Pinpoint from MT2");
        }
        telemetry.addData("DPAD ↓", showAbsoluteHeading ? "Show Absolute MT2 ✓" : "Show Absolute MT2");
        telemetry.addData("B", "Set heading = 0°");
        telemetry.addData("X", "Set heading = 90°");
        telemetry.addData("Y", "Set heading = 180°");
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}