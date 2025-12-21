package org.firstinspires.ftc.teamcode.decode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * LimelightLocalizationOpMode - TeleOp demonstrating robot position tracking with Limelight 3A.
 *
 * This OpMode shows how to:
 * - Initialize and configure the Limelight camera
 * - Get filtered robot poses
 * - Use IMU heading for MegaTag2 improved localization
 * - Display position telemetry
 *
 * Prerequisites:
 * - Limelight 3A configured in your robot configuration as "limelight"
 * - AprilTags placed on the field (FTC 2024-2025 tags or custom)
 * - Limelight pipeline configured for AprilTag detection
 */
@TeleOp(name = "Limelight Localization", group = "TeleOp")
public class LimelightLocalizationOpMode extends OpMode {

    // Limelight wrapper
    private CustomLimelight limelight;

    // IMU for heading fusion (optional but recommended)
    private IMU imu;
    private boolean useIMU = true;

    // Drive motors (for a mecanum drive example)
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // Timing
    private ElapsedTime loopTimer = new ElapsedTime();
    private double loopTime = 0;

    // Position tracking
    private Pose currentPose = new Pose(0, 0, 0);
    private boolean hasValidPose = false;

    // Configuration constants - ADJUST THESE FOR YOUR ROBOT
    private static final double CAMERA_FORWARD_OFFSET = 6.0;  // inches forward from robot center
    private static final double CAMERA_LEFT_OFFSET = 0.0;     // inches left from robot center
    private static final double CAMERA_HEADING_OFFSET = 0.0;  // radians rotation
    private static final int LIMELIGHT_PIPELINE = 0;          // AprilTag pipeline

    @Override
    public void init() {
        telemetry.addLine("Initializing Limelight Localization...");
        telemetry.update();

        // Initialize Limelight
        try {
            limelight = new CustomLimelight(hardwareMap, LIMELIGHT_PIPELINE);
            limelight.setCameraOffset(CAMERA_FORWARD_OFFSET, CAMERA_LEFT_OFFSET, CAMERA_HEADING_OFFSET);
            telemetry.addLine("✓ Limelight initialized");
        } catch (Exception e) {
            telemetry.addLine("✗ Limelight initialization failed: " + e.getMessage());
            limelight = null;
        }

        // Initialize IMU (optional - for MegaTag2 heading fusion)
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            // Configure IMU orientation based on your hub mounting
            IMU.Parameters parameters = new IMU.Parameters(
                    new com.qualcomm.hardware.rev.RevHubOrientationOnRobot(
                            com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
            );
            imu.initialize(parameters);
            imu.resetYaw();
            telemetry.addLine("✓ IMU initialized");
        } catch (Exception e) {
            telemetry.addLine("✗ IMU not found - using Limelight-only heading");
            imu = null;
            useIMU = false;
        }

        // Initialize drive motors (example - adjust for your configuration)
        try {
            frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
            backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
            backRight = hardwareMap.get(DcMotorEx.class, "backRight");

            // Set motor directions (adjust for your robot)
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.FORWARD);

            // Set zero power behavior
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            telemetry.addLine("✓ Drive motors initialized");
        } catch (Exception e) {
            telemetry.addLine("✗ Drive motors not found - position only mode");
            frontLeft = frontRight = backLeft = backRight = null;
        }

        telemetry.addLine("\nReady! Press START to begin.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Update position during init for pre-match positioning
        updatePose();

        telemetry.addLine("=== PRE-MATCH POSITIONING ===");
        if (hasValidPose) {
            telemetry.addData("Position", String.format("X: %.1f  Y: %.1f", currentPose.getX(), currentPose.getY()));
            telemetry.addData("Heading", String.format("%.1f°", Math.toDegrees(currentPose.getHeading())));
        } else {
            telemetry.addLine("No valid pose - check AprilTag visibility");
        }

        if (limelight != null) {
            telemetry.addData("Valid Readings", String.format("%.1f%%", limelight.getValidReadingPercentage()));
        }

        telemetry.update();
    }

    @Override
    public void start() {
        loopTimer.reset();

        // Clear filter history for fresh start
        if (limelight != null) {
            limelight.clearFilterHistory();
        }

        // Reset IMU heading at start if desired
        if (imu != null) {
            imu.resetYaw();
        }
    }

    @Override
    public void loop() {
        // Track loop time for performance monitoring
        loopTime = loopTimer.milliseconds();
        loopTimer.reset();

        // Update robot orientation for MegaTag2 (if using IMU)
        if (useIMU && imu != null && limelight != null) {
            double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            limelight.updateRobotOrientation(imuHeading);
        }

        // Update pose from Limelight
        updatePose();

        // Handle driving (mecanum drive example)
        if (frontLeft != null) {
            handleDriving();
        }

        // Handle gamepad controls for Limelight
        handleLimelightControls();

        // Update telemetry
        updateTelemetry();
    }

    /**
     * Updates the current pose from Limelight.
     */
    private void updatePose() {
        if (limelight == null) {
            hasValidPose = false;
            return;
        }

        Pose newPose = limelight.getLimelightPose();
        if (newPose != null) {
            currentPose = newPose;
            hasValidPose = true;
        } else {
            // Keep last valid pose but mark as stale
            hasValidPose = false;
        }
    }

    /**
     * Handles mecanum drive control.
     */
    private void handleDriving() {
        // Get gamepad inputs
        double drive = -gamepad1.left_stick_y;   // Forward/backward
        double strafe = gamepad1.left_stick_x;   // Left/right
        double rotate = gamepad1.right_stick_x;  // Rotation

        // Apply deadzone
        if (Math.abs(drive) < 0.05) drive = 0;
        if (Math.abs(strafe) < 0.05) strafe = 0;
        if (Math.abs(rotate) < 0.05) rotate = 0;

        // Optional: Field-centric driving using Limelight heading
        if (gamepad1.left_bumper && hasValidPose) {
            // Rotate input by robot heading for field-centric control
            double heading = currentPose.getHeading();
            double temp = drive * Math.cos(heading) + strafe * Math.sin(heading);
            strafe = -drive * Math.sin(heading) + strafe * Math.cos(heading);
            drive = temp;
        }

        // Calculate mecanum wheel powers
        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower = drive - strafe + rotate;
        double backRightPower = drive + strafe - rotate;

        // Normalize powers if any exceed 1.0
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    /**
     * Handles gamepad controls for Limelight settings.
     */
    private void handleLimelightControls() {
        if (limelight == null) return;

        // DPad up/down to change pipeline
        if (gamepad1.dpad_up) {
            int newPipeline = Math.min(9, limelight.getCurrentPipeline() + 1);
            limelight.setPipeline(newPipeline);
            sleep(200); // Debounce
        }
        if (gamepad1.dpad_down) {
            int newPipeline = Math.max(0, limelight.getCurrentPipeline() - 1);
            limelight.setPipeline(newPipeline);
            sleep(200); // Debounce
        }

        // Y button to reset filter history
        if (gamepad1.y) {
            limelight.clearFilterHistory();
            if (imu != null) {
                imu.resetYaw();
            }
        }
    }

    /**
     * Updates telemetry display.
     */
    private void updateTelemetry() {
        telemetry.addLine("=== LIMELIGHT LOCALIZATION ===\n");

        // Position data
        if (hasValidPose) {
            telemetry.addLine("--- POSITION (Valid) ---");
            telemetry.addData("X", String.format("%.2f in", currentPose.getX()));
            telemetry.addData("Y", String.format("%.2f in", currentPose.getY()));
            telemetry.addData("Heading", String.format("%.1f°", Math.toDegrees(currentPose.getHeading())));
        } else {
            telemetry.addLine("--- POSITION (Stale/Invalid) ---");
            telemetry.addData("Last X", String.format("%.2f in", currentPose.getX()));
            telemetry.addData("Last Y", String.format("%.2f in", currentPose.getY()));
            telemetry.addData("Last Heading", String.format("%.1f°", Math.toDegrees(currentPose.getHeading())));
            telemetry.addLine("⚠ No AprilTags visible!");
        }

        // Limelight status
        telemetry.addLine("\n--- LIMELIGHT STATUS ---");
        if (limelight != null) {
            telemetry.addData("Connected", limelight.isConnected() ? "Yes" : "No");
            telemetry.addData("Pipeline", limelight.getCurrentPipeline());
            telemetry.addData("Valid %", String.format("%.1f%%", limelight.getValidReadingPercentage()));
        } else {
            telemetry.addLine("Limelight not initialized!");
        }

        // IMU data
        if (imu != null) {
            telemetry.addLine("\n--- IMU ---");
            telemetry.addData("IMU Heading", String.format("%.1f°",
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
        }

        // Performance
        telemetry.addLine("\n--- PERFORMANCE ---");
        telemetry.addData("Loop Time", String.format("%.1f ms", loopTime));
        telemetry.addData("Loop Rate", String.format("%.0f Hz", 1000.0 / Math.max(1, loopTime)));

        // Controls help
        telemetry.addLine("\n--- CONTROLS ---");
        telemetry.addLine("Left Stick: Drive/Strafe");
        telemetry.addLine("Right Stick: Rotate");
        telemetry.addLine("LB + Stick: Field-Centric");
        telemetry.addLine("DPad Up/Down: Change Pipeline");
        telemetry.addLine("Y: Reset Position Filter");

        telemetry.update();
    }

    /**
     * Simple sleep function for debouncing.
     */
    private void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public void stop() {
        // Stop the Limelight
        if (limelight != null) {
            limelight.stop();
        }

        // Stop all motors
        if (frontLeft != null) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }
}