package org.firstinspires.ftc.teamcode.limeLight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

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
 * Controls:
 * - Press A to toggle IMU usage (for updateRobotOrientation)
 * - Press B to manually set robot heading to 0°
 * - Press X to manually set robot heading to 90°
 * - Press Y to manually set robot heading to 180°
 */
@TeleOp(name = "Limelight Coordinate Diagnostic", group = "Diagnostic")
public class LimelightCoordinateDiagnostic extends OpMode {

    private Limelight3A limelight;
    private IMU imu;

    private boolean useIMU = false;
    private double manualHeading = 0.0;

    private boolean lastAPress = false;
    private boolean lastBPress = false;
    private boolean lastXPress = false;
    private boolean lastYPress = false;

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(50);

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3); // AprilTag pipeline
        limelight.setPollRateHz(100);

        // Try to initialize IMU
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters parameters = new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
            );
            imu.initialize(parameters);
            telemetry.addData("IMU", "Available (press A to enable)");
        } catch (Exception e) {
            imu = null;
            telemetry.addData("IMU", "Not available");
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
        handleGamepadInput();

        // Get current heading
        double currentHeading = getCurrentHeading();

        // CRITICAL: Update robot orientation BEFORE getting results
        limelight.updateRobotOrientation(currentHeading);

        // Get latest result
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            displayNoData(currentHeading);
            return;
        }

        displayDiagnostics(result, currentHeading);
    }

    private void handleGamepadInput() {
        // Toggle IMU usage
        if (gamepad1.a && !lastAPress && imu != null) {
            useIMU = !useIMU;
        }
        lastAPress = gamepad1.a;

        // Set manual headings
        if (gamepad1.b && !lastBPress) {
            manualHeading = 0.0;
            useIMU = false;
        }
        lastBPress = gamepad1.b;

        if (gamepad1.x && !lastXPress) {
            manualHeading = 90.0;
            useIMU = false;
        }
        lastXPress = gamepad1.x;

        if (gamepad1.y && !lastYPress) {
            manualHeading = 180.0;
            useIMU = false;
        }
        lastYPress = gamepad1.y;
    }

    private double getCurrentHeading() {
        if (useIMU && imu != null) {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
        return manualHeading;
    }

    private void displayNoData(double currentHeading) {
        telemetry.addLine("═══ ROBOT HEADING ═══");
        telemetry.addData("Source", useIMU ? "IMU" : "Manual");
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
        telemetry.addData("Source", useIMU ? "IMU" : "Manual");
        telemetry.addData("Heading", "%.1f°", currentHeading);

        telemetry.addLine();
        telemetry.addLine("═══ MEGATAG2 (FIELD-CENTRIC) ═══");
        displayMegaTag2(result);

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

    private void displayMegaTag2(LLResult result) {
        Pose3D mt2 = result.getBotpose_MT2();
        if (mt2 != null) {
            double x = mt2.getPosition().x;
            double y = mt2.getPosition().y;
            double z = mt2.getPosition().z;
            double yaw = mt2.getOrientation().getYaw();

            telemetry.addData("X (meters)", "%.3f", x);
            telemetry.addData("Y (meters)", "%.3f", y);
            telemetry.addData("Z (meters)", "%.3f", z);
            telemetry.addData("Yaw (degrees)", "%.1f°", yaw);
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

            // Check if this is being used incorrectly as field coordinates
            if (robotTarget.getPosition().z < 3.0) {
                telemetry.addData("⚠ NOTE", "Z decreases as you approach tag");
                telemetry.addData("", "This is TAG-RELATIVE, not field!");
            }
        }
    }

    private void displayControls() {
        telemetry.addLine("─── CONTROLS ───");
        if (imu != null) {
            telemetry.addData("A", useIMU ? "Using IMU ✓" : "Use IMU");
        }
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