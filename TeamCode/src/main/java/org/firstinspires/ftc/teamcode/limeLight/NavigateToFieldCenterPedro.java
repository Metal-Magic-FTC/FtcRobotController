package org.firstinspires.ftc.teamcode.limeLight;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Autonomous OpMode to navigate robot to field center (0, 0, 0°) using PedroPathing.
 *
 * This OpMode demonstrates:
 * - Limelight vision for absolute field localization
 * - PedroPathing's pre-tuned controllers for movement
 * - Vision-based pose correction for accurate autonomous
 *
 * How it works:
 * 1. Uses Limelight to determine current field position
 * 2. Updates PedroPathing Follower's pose with vision data
 * 3. Creates path to field center (0, 0, 0°)
 * 4. PedroPathing handles all movement with pre-tuned PID
 * 5. Continuous vision corrections during movement
 *
 * Advantages over custom PID:
 * - Pre-tuned PID controllers (no tuning required!)
 * - Optimized for FTC mecanum drives
 * - Smooth path following with acceleration limits
 * - Robust to disturbances
 *
 * Configuration:
 * - Limelight 3A with FTC field layout loaded
 * - Limelight orientation set in web interface
 * - PedroPathing configured in Constants.java
 * - Pinpoint odometry (configured in Constants.java)
 *
 * IMPORTANT - Heading Initialization:
 * Since Pinpoint odometry resets to 0° at program start, this OpMode uses
 * Limelight MegaTag2 to determine the robot's ABSOLUTE field heading at startup,
 * then initializes Pinpoint with that heading. This ensures MegaTag2 works
 * correctly regardless of robot orientation at start.
 *
 * Target: (0, 0, 0°) in field coordinates (inches)
 * - X = 0 inches (field center along length)
 * - Y = 0 inches (field center along width)
 * - Heading = 0° (facing forward)
 */
@Autonomous(name = "Navigate to Center (PedroPathing)", group = "Limelight")
public class NavigateToFieldCenterPedro extends LinearOpMode {

    // Localization and Control
    private LimelightLocalizer limelightLocalizer;
    private Follower follower;

    // Target position (field center in inches - PedroPathing uses inches)
    private static final double TARGET_X = 0.0;      // inches
    private static final double TARGET_Y = 0.0;      // inches
    private static final double TARGET_HEADING = 0.0; // radians

    // Vision correction settings
    private static final double VISION_CONFIDENCE_THRESHOLD = 0.5;
    private static final long VISION_UPDATE_INTERVAL_MS = 500;
    private long lastVisionUpdateTime = 0;
    private int visionCorrectionCount = 0;

    // Navigation settings
    private static final double POSITION_TOLERANCE = 2.0;  // inches
    private static final double HEADING_TOLERANCE = Math.toRadians(3.0); // radians
    private static final double NAVIGATION_TIMEOUT_SEC = 20.0;
    private static final double PATH_TIMEOUT_SEC = 15.0;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime pathTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize hardware
        initializeHardware();

        telemetry.addData("Status", "Ready!");
        telemetry.addData("Target", "X: %.1f, Y: %.1f, H: %.0f°",
                         TARGET_X, TARGET_Y, Math.toDegrees(TARGET_HEADING));
        telemetry.addData("", "Robot will navigate to field center");
        telemetry.addLine();
        telemetry.addLine("PedroPathing will handle all movement!");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            // Get initial position from Limelight
            Pose initialPose = getVisionPose();
            if (initialPose != null) {
                follower.setPose(initialPose);
                telemetry.addData("Initial Pose", "X: %.1f, Y: %.1f, H: %.1f°",
                                 initialPose.getX(), initialPose.getY(),
                                 Math.toDegrees(initialPose.getHeading()));
            } else {
                telemetry.addData("Warning", "No vision data, using odometry");
                initialPose = new Pose(0, 0, 0);
                follower.setPose(initialPose);
            }
            telemetry.update();
            sleep(500);

            // Navigate to field center
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

        telemetry.addData("Step 2", "Getting absolute heading from MegaTag2...");
        telemetry.update();

        // Get initial ABSOLUTE heading from Limelight MegaTag2
        // NOTE: We do NOT call updateRobotOrientation() first because Pinpoint is at 0°
        // but we need the TRUE field heading. MegaTag2 can determine this independently.
        RobotPose initialVisionPose = getInitialAbsolutePose();

        if (initialVisionPose != null && initialVisionPose.getConfidence() > 0.5) {
            telemetry.addData("✓ Vision Heading", "%.1f°", initialVisionPose.getHeading());
            telemetry.addData("  Confidence", "%.2f", initialVisionPose.getConfidence());
            telemetry.update();

            // Initialize Pinpoint odometry with absolute heading from vision
            initializePinpointWithAbsoluteHeading(initialVisionPose);

            telemetry.addData("Step 3", "Pinpoint initialized with absolute heading");
            telemetry.update();
        } else {
            telemetry.addData("⚠ Warning", "No MegaTag2 data available");
            telemetry.addData("", "Ensure multiple AprilTags are visible");
            telemetry.addData("", "Pinpoint will start at 0° (may cause errors)");
            telemetry.update();
            sleep(3000);
        }

        telemetry.addData("Step 4", "Starting PedroPathing...");
        telemetry.update();

        // Initialize PedroPathing Follower (will read the heading we just set)
        follower = Constants.createFollower(hardwareMap);

        // Set initial pose if we have vision data
        if (initialVisionPose != null) {
            double xInches = initialVisionPose.getX() * 39.3701;
            double yInches = initialVisionPose.getY() * 39.3701;
            double headingRad = Math.toRadians(initialVisionPose.getHeading());
            follower.setPose(new Pose(xInches, yInches, headingRad));
        } else {
            follower.setPose(new Pose(0, 0, 0));
        }

        telemetry.addLine("=== READY ===");
        telemetry.addData("Limelight", "✓ Initialized");
        telemetry.addData("PedroPathing", "✓ Initialized");
        telemetry.addData("Odometry", "✓ Pinpoint (absolute heading)");
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
     * Initializes Pinpoint odometry with absolute heading from vision.
     * This solves the "odometry reset" problem by setting the true field heading.
     *
     * @param visionPose Pose from Limelight with absolute field heading
     */
    private void initializePinpointWithAbsoluteHeading(RobotPose visionPose) {
        // Create temporary Pinpoint driver to set initial position
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(0.0, 0.0); // Center of robot
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.REVERSED,
            GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        odo.resetPosAndIMU();

        // Convert vision pose to Pinpoint format (meters to mm, degrees to radians)
        double xMM = visionPose.getX() * 1000.0; // meters to mm
        double yMM = visionPose.getY() * 1000.0;
        double headingRad = Math.toRadians(visionPose.getHeading());

        // Set absolute position and heading
        odo.setPosition(new Pose2D(DistanceUnit.MM, xMM, yMM, AngleUnit.RADIANS, headingRad));
        odo.update();

        telemetry.addData("✓ Pinpoint Set", "X: %.1fmm, Y: %.1fmm, H: %.1f°",
            xMM, yMM, visionPose.getHeading());
    }

    /**
     * Navigate to the target position using PedroPathing.
     */
    private void navigateToTarget() {
        Pose currentPose = follower.getPose();

        // Create target pose in PedroPathing format
        Pose targetPose = new Pose(TARGET_X, TARGET_Y, TARGET_HEADING);

        // Build path to target
        PathChain pathToCenter = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, targetPose))
                .setConstantHeadingInterpolation(TARGET_HEADING) // Face target heading
                .build();

        // Start following path
        follower.followPath(pathToCenter);
        pathTimer.reset();

        telemetry.addData("Status", "Following path to center...");
        telemetry.update();

        // Follow path with continuous vision corrections
        while (opModeIsActive() && !isTargetReached()) {
            // Update PedroPathing follower
            follower.update();

            // Get current pose from PedroPathing
            currentPose = follower.getPose();

            // Apply vision correction if available
            applyVisionCorrection();

            // Check for path completion or timeout
            if (follower.atParametricEnd()) {
                telemetry.addData("Status", "Path complete, checking final position...");
            }

            if (pathTimer.seconds() > PATH_TIMEOUT_SEC) {
                telemetry.addData("Status", "Path timeout");
                break;
            }

            if (runtime.seconds() > NAVIGATION_TIMEOUT_SEC) {
                telemetry.addData("Status", "Navigation timeout");
                break;
            }

            // Display telemetry
            displayNavigationTelemetry(currentPose);

            sleep(10); // Small delay to prevent excessive loop frequency
        }

        // Stop the follower
        follower.breakFollowing();

        // Final status
        Pose finalPose = follower.getPose();
        double finalDistance = Math.hypot(
            finalPose.getX() - TARGET_X,
            finalPose.getY() - TARGET_Y
        );
        double finalHeadingError = Math.abs(normalizeAngle(
            finalPose.getHeading() - TARGET_HEADING
        ));

        telemetry.addLine("=== NAVIGATION COMPLETE ===");
        telemetry.addData("Final Position", "X: %.1f, Y: %.1f, H: %.1f°",
                         finalPose.getX(), finalPose.getY(),
                         Math.toDegrees(finalPose.getHeading()));
        telemetry.addData("Distance to Target", "%.2f inches", finalDistance);
        telemetry.addData("Heading Error", "%.1f°", Math.toDegrees(finalHeadingError));
        telemetry.addData("Vision Corrections", visionCorrectionCount);
        telemetry.addData("Total Time", "%.1f sec", runtime.seconds());

        if (finalDistance <= POSITION_TOLERANCE && finalHeadingError <= HEADING_TOLERANCE) {
            telemetry.addData("Result", "✓ TARGET REACHED!");
        } else {
            telemetry.addData("Result", "⚠ Close but not in tolerance");
        }

        telemetry.update();
        sleep(3000);
    }

    /**
     * Checks if robot has reached the target position.
     *
     * @return true if within tolerance, false otherwise
     */
    private boolean isTargetReached() {
        Pose currentPose = follower.getPose();
        if (currentPose == null) {
            return false;
        }

        double dx = currentPose.getX() - TARGET_X;
        double dy = currentPose.getY() - TARGET_Y;
        double distance = Math.hypot(dx, dy);

        double headingError = Math.abs(normalizeAngle(
            currentPose.getHeading() - TARGET_HEADING
        ));

        return distance <= POSITION_TOLERANCE && headingError <= HEADING_TOLERANCE;
    }

    /**
     * Gets current pose from Limelight vision.
     * Returns null if no vision data available.
     *
     * @return Pose in PedroPathing format (inches, radians), or null
     */
    private Pose getVisionPose() {
        // Get vision RobotPose first
        RobotPose visionPose = getVisionRobotPose();

        if (visionPose == null) {
            return null;
        }

        // Convert from meters to inches (PedroPathing uses inches)
        double xInches = visionPose.getX() * 39.3701;
        double yInches = visionPose.getY() * 39.3701;
        double headingRad = Math.toRadians(visionPose.getHeading());

        return new Pose(xInches, yInches, headingRad);
    }

    /**
     * Gets RobotPose from Limelight vision.
     * Returns null if no vision data available.
     *
     * @return RobotPose with confidence, or null
     */
    private RobotPose getVisionRobotPose() {
        // Update Limelight with current heading from odometry
        Pose currentPose = follower.getPose();
        if (currentPose == null) {
            return null;
        }

        limelightLocalizer.updateRobotOrientation(
            Math.toDegrees(currentPose.getHeading())
        );

        // Get vision pose
        return limelightLocalizer.update();
    }

    /**
     * Applies vision correction to PedroPathing pose if available and reliable.
     */
    private void applyVisionCorrection() {
        long currentTime = System.currentTimeMillis();
        long timeSinceLastUpdate = currentTime - lastVisionUpdateTime;

        // Only update at specified interval
        if (timeSinceLastUpdate < VISION_UPDATE_INTERVAL_MS) {
            return;
        }

        // Get vision RobotPose (with confidence)
        RobotPose robotPose = getVisionRobotPose();

        if (robotPose == null) {
            return;
        }

        // Check confidence threshold
        if (robotPose.getConfidence() < VISION_CONFIDENCE_THRESHOLD) {
            return;
        }

        // Convert to PedroPathing Pose format
        double xInches = robotPose.getX() * 39.3701;
        double yInches = robotPose.getY() * 39.3701;
        double headingRad = Math.toRadians(robotPose.getHeading());
        Pose visionPose = new Pose(xInches, yInches, headingRad);

        // Calculate correction magnitude
        Pose currentPose = follower.getPose();
        if (currentPose == null) {
            return;
        }

        double correctionDistance = Math.hypot(
            visionPose.getX() - currentPose.getX(),
            visionPose.getY() - currentPose.getY()
        );

        // Apply correction to PedroPathing
        follower.setPose(visionPose);

        lastVisionUpdateTime = currentTime;
        visionCorrectionCount++;

        telemetry.addData("Vision Correction", "%.2f in (conf: %.2f)",
                         correctionDistance, robotPose.getConfidence());
    }

    /**
     * Displays comprehensive navigation telemetry.
     */
    private void displayNavigationTelemetry(Pose currentPose) {
        // Safety check
        if (currentPose == null) {
            telemetry.addData("Status", "Error: No pose data");
            telemetry.update();
            return;
        }

        telemetry.addLine("=== NAVIGATION STATUS ===");
        telemetry.addData("Time", "%.1f / %.0f sec", runtime.seconds(), NAVIGATION_TIMEOUT_SEC);

        telemetry.addLine();
        telemetry.addData("Current X", "%.1f in", currentPose.getX());
        telemetry.addData("Current Y", "%.1f in", currentPose.getY());
        telemetry.addData("Current Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));

        telemetry.addLine();
        telemetry.addData("Target X", "%.1f in", TARGET_X);
        telemetry.addData("Target Y", "%.1f in", TARGET_Y);
        telemetry.addData("Target Heading", "%.1f°", Math.toDegrees(TARGET_HEADING));

        telemetry.addLine();
        double dx = TARGET_X - currentPose.getX();
        double dy = TARGET_Y - currentPose.getY();
        double distance = Math.hypot(dx, dy);
        double headingError = normalizeAngle(TARGET_HEADING - currentPose.getHeading());

        telemetry.addData("Distance to Target", "%.1f in", distance);
        telemetry.addData("Heading Error", "%.1f°", Math.toDegrees(headingError));

        telemetry.addLine();
        telemetry.addData("Path Status", follower.atParametricEnd() ? "Complete" : "Following");

        // Vision info
        RobotPose visionPose = limelightLocalizer.getLastValidPose();
        if (visionPose != null) {
            telemetry.addData("Vision Mode", limelightLocalizer.getCurrentMode());
            telemetry.addData("Vision Confidence", "%.2f", visionPose.getConfidence());
            telemetry.addData("Visible Tags", limelightLocalizer.getVisibleTagCount());
            telemetry.addData("Vision Corrections", visionCorrectionCount);
        } else {
            telemetry.addData("Vision", "No data available");
        }

        telemetry.addLine();
        telemetry.addData("PedroPathing", "Handling all movement");
        telemetry.addData("Controllers", "Pre-tuned from Constants.java");

        telemetry.update();
    }

    /**
     * Normalizes angle to [-PI, PI] range.
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
