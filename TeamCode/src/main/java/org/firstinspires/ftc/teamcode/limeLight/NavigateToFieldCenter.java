package org.firstinspires.ftc.teamcode.limeLight;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.GoBildaPinpointDriver;

/**
 * Autonomous OpMode to navigate the robot to field center (0, 0, 0°).
 *
 * This OpMode demonstrates:
 * - Limelight + Odometry sensor fusion for accurate localization
 * - PID control for smooth autonomous navigation
 * - Mecanum drive field-centric movement
 * - Vision-based position correction
 *
 * Target Position: (0, 0, 0°) - Field center, facing forward
 *
 * Configuration Requirements:
 * - Limelight 3A configured with FTC field layout
 * - Limelight orientation set in web interface
 * - GoBilda Pinpoint odometry at robot center (offsets = 0, 0)
 * - Swingarm odometry pods
 * - Mecanum drive with motors: frontLeft, frontRight, backLeft, backRight
 *
 * How it works:
 * 1. Uses odometry for continuous position tracking
 * 2. Limelight provides periodic vision corrections
 * 3. PID controllers calculate velocities to reach target
 * 4. Mecanum drive executes field-centric movement
 * 5. Stops when within tolerance of target position
 */
@Autonomous(name = "Navigate to Field Center", group = "Limelight")
public class NavigateToFieldCenter extends LinearOpMode {

    // Hardware
    private LimelightLocalizer limelightLocalizer;
    private GoBildaPinpointDriver odometry;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Target position (field center)
    private static final double TARGET_X = 0.0;      // meters
    private static final double TARGET_Y = 0.0;      // meters
    private static final double TARGET_HEADING = 0.0; // degrees

    // Position tolerance for "arrived" detection
    private static final double POSITION_TOLERANCE = 0.05;  // 5cm
    private static final double HEADING_TOLERANCE = 2.0;    // 2 degrees

    // PID Controllers
    private PIDController xController;
    private PIDController yController;
    private PIDController headingController;

    // Vision correction
    private static final double VISION_CONFIDENCE_THRESHOLD = 0.5;
    private static final long VISION_UPDATE_INTERVAL_MS = 500;
    private long lastVisionUpdateTime = 0;

    // Speed limits
    private static final double MAX_DRIVE_SPEED = 0.6;
    private static final double MAX_TURN_SPEED = 0.5;
    private static final double MIN_DRIVE_SPEED = 0.1;

    // Timeouts
    private static final double NAVIGATION_TIMEOUT_SEC = 15.0;
    private static final double SETTLE_TIME_SEC = 0.5;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime settleTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize hardware
        initializeHardware();

        // Initialize PID controllers
        initializePIDControllers();

        telemetry.addData("Status", "Ready!");
        telemetry.addData("Target", "X: %.2f, Y: %.2f, H: %.0f°", TARGET_X, TARGET_Y, TARGET_HEADING);
        telemetry.addData("", "Robot will navigate to field center");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            navigateToTarget();
        }
    }

    /**
     * Initialize all hardware components.
     *
     * CRITICAL: This method initializes Pinpoint odometry with ABSOLUTE field heading
     * from Limelight MegaTag2, solving the "odometry reset" issue where Pinpoint
     * starts at 0° regardless of actual robot orientation.
     */
    private void initializeHardware() {
        telemetry.addLine("=== INITIALIZATION ===");
        telemetry.addData("Step 1", "Starting Limelight...");
        telemetry.update();

        // Initialize Limelight Localizer
        limelightLocalizer = new LimelightLocalizer(hardwareMap, "limelight");
        limelightLocalizer.start();
        sleep(500); // Give Limelight time to capture frames

        telemetry.addData("Step 2", "Configuring Pinpoint odometry...");
        telemetry.update();

        // Initialize GoBilda Pinpoint Odometry
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odometry.setOffsets(0.0, 0.0, DistanceUnit.MM); // Center of robot
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometry.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        odometry.resetPosAndIMU();

        telemetry.addData("Step 3", "Getting absolute heading from MegaTag2...");
        telemetry.update();

        // Get initial ABSOLUTE heading from Limelight MegaTag2
        RobotPose initialVisionPose = getInitialAbsolutePose();

        if (initialVisionPose != null && initialVisionPose.getConfidence() > 0.5) {
            telemetry.addData("✓ Vision Heading", "%.1f°", initialVisionPose.getHeading());
            telemetry.addData("  Confidence", "%.2f", initialVisionPose.getConfidence());

            // Initialize Pinpoint with absolute heading from vision
            double xMM = initialVisionPose.getX() * 1000.0; // meters to mm
            double yMM = initialVisionPose.getY() * 1000.0;
            double headingRad = Math.toRadians(initialVisionPose.getHeading());

            odometry.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                DistanceUnit.MM, xMM, yMM,
                AngleUnit.RADIANS, headingRad
            ));
            odometry.update();

            telemetry.addData("✓ Pinpoint Set", "X: %.0fmm, Y: %.0fmm, H: %.1f°",
                xMM, yMM, initialVisionPose.getHeading());
        } else {
            telemetry.addData("⚠ Warning", "No MegaTag2 data available");
            telemetry.addData("", "Ensure multiple AprilTags are visible");
            telemetry.addData("", "Pinpoint will start at 0° (may cause errors)");
            sleep(3000);
        }

        telemetry.update();

        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set motor directions (adjust for your robot)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set brake behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("=== READY ===");
        telemetry.addData("Limelight", "✓ Initialized");
        telemetry.addData("Odometry", "✓ Pinpoint (absolute heading)");
        telemetry.addData("Motors", "✓ Configured");
    }

    /**
     * Gets initial absolute pose from Limelight MegaTag2 WITHOUT providing robot orientation.
     * This is crucial because Pinpoint resets to 0° at startup, but we need TRUE field heading.
     *
     * @return RobotPose with absolute field coordinates, or null if not available
     */
    private RobotPose getInitialAbsolutePose() {
        // Try multiple times to get good MegaTag2 data
        for (int attempt = 0; attempt < 10; attempt++) {
            // DO NOT call updateRobotOrientation() - we need absolute heading!
            RobotPose pose = limelightLocalizer.update();

            if (pose != null && pose.getConfidence() > 0.5) {
                // Verify this is MegaTag2 (not single-tag)
                if (limelightLocalizer.getCurrentMode() == LimelightLocalizer.LocalizationMode.MEGATAG2) {
                    return pose;
                }
            }

            sleep(100); // Wait for next frame
        }

        return null; // No MegaTag2 data available
    }

    /**
     * Initialize PID controllers for position and heading control.
     */
    private void initializePIDControllers() {
        // Position controllers (field coordinates)
        // Tune these values for your robot
        xController = new PIDController(1.5, 0.0, 0.1);
        yController = new PIDController(1.5, 0.0, 0.1);

        // Heading controller
        headingController = new PIDController(0.02, 0.0, 0.005);
        headingController.setInputRange(-180, 180);
        headingController.setContinuous(true);
    }

    /**
     * Main navigation loop - drives robot to target position.
     */
    private void navigateToTarget() {
        boolean targetReached = false;

        while (opModeIsActive() && !targetReached) {
            // Update odometry
            odometry.update();
            Pose2D odoPose = odometry.getPosition();

            // Get current position
            double currentX = odoPose.getX(DistanceUnit.METER);
            double currentY = odoPose.getY(DistanceUnit.METER);
            double currentHeading = odoPose.getHeading(AngleUnit.DEGREES);

            // Update Limelight with current heading for field-centric localization
            limelightLocalizer.updateRobotOrientation(currentHeading);

            // Get vision correction
            RobotPose visionPose = limelightLocalizer.update();
            correctOdometryWithVision(visionPose, odoPose);

            // Calculate errors
            double xError = TARGET_X - currentX;
            double yError = TARGET_Y - currentY;
            double headingError = normalizeAngle(TARGET_HEADING - currentHeading);

            // Calculate distance to target
            double distanceToTarget = Math.sqrt(xError * xError + yError * yError);

            // Check if we've reached the target
            if (distanceToTarget < POSITION_TOLERANCE && Math.abs(headingError) < HEADING_TOLERANCE) {
                if (settleTimer.seconds() > SETTLE_TIME_SEC) {
                    targetReached = true;
                }
            } else {
                settleTimer.reset();
            }

            // Check timeout
            if (runtime.seconds() > NAVIGATION_TIMEOUT_SEC) {
                telemetry.addData("Status", "TIMEOUT - stopping");
                break;
            }

            if (!targetReached) {
                // Calculate drive velocities using PID
                double xVelocity = xController.calculate(currentX, TARGET_X);
                double yVelocity = yController.calculate(currentY, TARGET_Y);
                double turnVelocity = headingController.calculate(currentHeading, TARGET_HEADING);

                // Apply speed limits
                xVelocity = clamp(xVelocity, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
                yVelocity = clamp(yVelocity, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
                turnVelocity = clamp(turnVelocity, -MAX_TURN_SPEED, MAX_TURN_SPEED);

                // Apply minimum speed threshold (prevent stalling)
                if (distanceToTarget > POSITION_TOLERANCE) {
                    double driveSpeed = Math.sqrt(xVelocity * xVelocity + yVelocity * yVelocity);
                    if (driveSpeed > 0 && driveSpeed < MIN_DRIVE_SPEED) {
                        double scale = MIN_DRIVE_SPEED / driveSpeed;
                        xVelocity *= scale;
                        yVelocity *= scale;
                    }
                }

                // Drive using field-centric control
                driveFieldCentric(xVelocity, yVelocity, turnVelocity, currentHeading);

                // Display telemetry
                displayNavigationTelemetry(currentX, currentY, currentHeading,
                                          xError, yError, headingError,
                                          distanceToTarget, visionPose);
            }
        }

        // Stop all motors
        stopDriving();

        // Final status
        telemetry.addData("Status", "TARGET REACHED!");
        telemetry.addData("Final Position", "X: %.3f, Y: %.3f",
                         odometry.getPosition().getX(DistanceUnit.METER),
                         odometry.getPosition().getY(DistanceUnit.METER));
        telemetry.addData("Final Heading", "%.1f°",
                         odometry.getPosition().getHeading(AngleUnit.DEGREES));
        telemetry.update();

        sleep(2000);
    }

    /**
     * Corrects odometry position with high-confidence vision measurements.
     */
    private void correctOdometryWithVision(RobotPose visionPose, Pose2D odoPose) {
        if (visionPose == null) return;

        long currentTime = System.currentTimeMillis();
        long timeSinceLastUpdate = currentTime - lastVisionUpdateTime;

        // Only update if high confidence and enough time has passed
        if (visionPose.getConfidence() >= VISION_CONFIDENCE_THRESHOLD &&
            timeSinceLastUpdate >= VISION_UPDATE_INTERVAL_MS) {

            // Convert vision pose to odometry format
            double xMm = visionPose.getX() * 1000.0;
            double yMm = visionPose.getY() * 1000.0;
            double headingRad = Math.toRadians(visionPose.getHeading());

            // Calculate correction magnitude
            double correctionDistance = Math.sqrt(
                Math.pow(xMm - odoPose.getX(DistanceUnit.MM), 2) +
                Math.pow(yMm - odoPose.getY(DistanceUnit.MM), 2)
            );

            // Apply correction
            Pose2D correctedPose = new Pose2D(DistanceUnit.MM, xMm, yMm,
                                              AngleUnit.RADIANS, headingRad);
            odometry.setPosition(correctedPose);

            lastVisionUpdateTime = currentTime;

            telemetry.addData("Vision Correction", "%.1f mm", correctionDistance);
        }
    }

    /**
     * Drives the robot using field-centric mecanum control.
     *
     * @param fieldX X velocity in field coordinates (forward/back)
     * @param fieldY Y velocity in field coordinates (left/right)
     * @param turn Rotational velocity
     * @param robotHeading Current robot heading in degrees
     */
    private void driveFieldCentric(double fieldX, double fieldY, double turn, double robotHeading) {
        // Convert field-centric to robot-centric
        double headingRad = Math.toRadians(robotHeading);
        double robotX = fieldX * Math.cos(-headingRad) - fieldY * Math.sin(-headingRad);
        double robotY = fieldX * Math.sin(-headingRad) + fieldY * Math.cos(-headingRad);

        // Mecanum drive calculations
        double frontLeftPower = robotX + robotY + turn;
        double frontRightPower = robotX - robotY - turn;
        double backLeftPower = robotX - robotY + turn;
        double backRightPower = robotX + robotY - turn;

        // Normalize powers to [-1, 1] range
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
     * Stops all drive motors.
     */
    private void stopDriving() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /**
     * Displays navigation telemetry.
     */
    private void displayNavigationTelemetry(double x, double y, double heading,
                                           double xError, double yError, double headingError,
                                           double distance, RobotPose visionPose) {
        telemetry.addLine("=== NAVIGATION STATUS ===");
        telemetry.addData("Time", "%.1f / %.0f sec", runtime.seconds(), NAVIGATION_TIMEOUT_SEC);

        telemetry.addLine();
        telemetry.addData("Current X", "%.3f m", x);
        telemetry.addData("Current Y", "%.3f m", y);
        telemetry.addData("Current Heading", "%.1f°", heading);

        telemetry.addLine();
        telemetry.addData("Target X", "%.3f m", TARGET_X);
        telemetry.addData("Target Y", "%.3f m", TARGET_Y);
        telemetry.addData("Target Heading", "%.1f°", TARGET_HEADING);

        telemetry.addLine();
        telemetry.addData("Error X", "%.3f m", xError);
        telemetry.addData("Error Y", "%.3f m", yError);
        telemetry.addData("Error Heading", "%.1f°", headingError);
        telemetry.addData("Distance to Target", "%.3f m", distance);

        telemetry.addLine();
        telemetry.addData("Vision Available", visionPose != null ? "YES" : "NO");
        if (visionPose != null) {
            telemetry.addData("Vision Confidence", "%.2f", visionPose.getConfidence());
            telemetry.addData("Vision Mode", limelightLocalizer.getCurrentMode());
            telemetry.addData("Visible Tags", limelightLocalizer.getVisibleTagCount());
        }

        telemetry.update();
    }

    /**
     * Normalizes angle to [-180, 180] range.
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    /**
     * Clamps value to [min, max] range.
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Simple PID Controller for autonomous navigation.
     */
    private static class PIDController {
        private double kP, kI, kD;
        private double integral = 0;
        private double previousError = 0;
        private double minInput = -Double.MAX_VALUE;
        private double maxInput = Double.MAX_VALUE;
        private boolean continuous = false;

        public PIDController(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        public void setInputRange(double min, double max) {
            this.minInput = min;
            this.maxInput = max;
        }

        public void setContinuous(boolean continuous) {
            this.continuous = continuous;
        }

        public double calculate(double current, double target) {
            double error = target - current;

            // Handle continuous input (angles)
            if (continuous) {
                double range = maxInput - minInput;
                while (error > range / 2) error -= range;
                while (error < -range / 2) error += range;
            }

            // PID calculations
            integral += error;
            double derivative = error - previousError;
            previousError = error;

            return kP * error + kI * integral + kD * derivative;
        }

        public void reset() {
            integral = 0;
            previousError = 0;
        }
    }
}
