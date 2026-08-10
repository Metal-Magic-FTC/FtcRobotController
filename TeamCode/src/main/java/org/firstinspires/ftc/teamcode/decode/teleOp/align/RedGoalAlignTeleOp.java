package org.firstinspires.ftc.teamcode.decode.teleOp.align;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

import java.util.List;

/**
 * TeleOp that localizes off the RED GOAL AprilTag and can snap the robot to face it.
 *
 * Flow:
 *   1. Robot starts with NO idea where it is. Pedro's pose is seeded at (0,0,0) and
 *      flagged as unknown - the align button is disabled until we get a fix.
 *   2. Driver drives normally (Pedro teleop drive, odometry pods integrating the whole time).
 *   3. Any time the Limelight sees the red goal tag (ID 24, per DECODE manual section 9.10 -
 *      red goal = 24, blue goal = 20), we convert its field-space robot pose into Pedro
 *      coordinates and push it into the follower:
 *        - first valid sighting  -> hard seed (setPose), pose becomes "known"
 *        - later sightings       -> blended correction, gated against outliers, so the
 *                                   odometry pods still carry the pose between sightings
 *      Losing sight of the tag changes nothing: the pods keep the pose alive.
 *   4. Hold A -> heading PID takes over the turn axis and rotates the robot to face the
 *      red goal point. Translation stays fully under driver control.
 *      Alignment disengages when: A is released, the heading error settles inside
 *      tolerance, or the driver touches the right stick. Either way the pose is untouched -
 *      the robot just goes back to normal driving from wherever it is.
 *
 * Hardware: Pedro's usual "odo" Pinpoint (via Constants) + a Limelight3A named "limelight".
 */
@TeleOp(name = "Red Goal Align TeleOp", group = "align")
public class RedGoalAlignTeleOp extends LinearOpMode {

    // ---------------- FIELD / TARGET CONSTANTS ----------------

    /** DECODE red alliance goal AprilTag (manual 9.10). Blue is 20. */
    private static final int RED_GOAL_TAG_ID = 24;

    /**
     * Point we aim at, in Pedro field inches. The red goal sits behind the far corner of the
     * field, so this is slightly outside the 144x144 tile area on purpose. Same numbers the
     * states TeleOp aims with - retune here if the shot moves.
     */
    private static final double RED_GOAL_X = 150;
    private static final double RED_GOAL_Y = 137;

    /**
     * Added to the computed bearing before the heading PID runs. 0 means "point the front of
     * the robot at the goal". If your shooter/camera faces the back, use 180.
     */
    private static final double AIM_HEADING_OFFSET_DEG = 0;

    // ---------------- DRIVE / ALIGN TUNING ----------------

    /** false = field centric drive. Robot centric matches the existing TeleOps. */
    private static final boolean ROBOT_CENTRIC = true;

    /** Right stick past this while aligning hands the turn axis back to the driver. */
    private static final double TURN_TAKEOVER_DEADBAND = 0.15;

    /** Heading error under this (degrees), held for SETTLE_MS, counts as aligned. */
    private static final double ALIGN_TOLERANCE_DEG = 1.5;
    private static final long ALIGN_SETTLE_MS = 150;

    /** Clamp on the PID's turn output so a big error can't slam the drivetrain. */
    private static final double MAX_ALIGN_TURN = 0.6;

    // ---------------- VISION TUNING ----------------

    private static final int LIMELIGHT_PIPELINE = 3; // AprilTag pipeline, same as the rest of the code
    private static final long VISION_UPDATE_INTERVAL_MS = 100;

    /**
     * After we already have a fix, ignore a vision pose that disagrees with odometry by more
     * than this (inches). A bad single-tag solve is usually way off, and blindly trusting it
     * would teleport the robot mid-match.
     */
    private static final double VISION_OUTLIER_INCHES = 24.0;

    /**
     * Blend weight for corrections after the initial seed. 0 = ignore vision, 1 = snap to it.
     * Low values ease the pose over instead of jumping.
     */
    private static final double VISION_BLEND = 0.20;

    // ---------------- STATE ----------------

    private Follower follower;
    private Limelight3A limelight;
    private PIDFController headingController;

    private boolean poseKnown = false;          // have we ever seen the red goal tag?
    private boolean visionCorrectionEnabled = true;
    private long lastVisionUpdateMs = 0;
    private int visionFixCount = 0;
    private double lastCorrectionInches = 0;
    private long lastTagSeenMs = 0;

    private boolean aligning = false;
    private boolean alignDone = false;          // aligned once; wait for A release before re-arming
    private double targetHeading = 0;
    private long alignInToleranceSince = 0;

    private boolean prevDpadUp, prevDpadDown;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        while (opModeInInit()) {
            // Let the driver point the camera at the goal before start if they want -
            // the first fix can land during init.
            tryVisionUpdate();
            telemetry.addLine("Red Goal Align TeleOp");
            telemetry.addData("Pose known", poseKnown);
            telemetry.addData("Red tag (24) visible", System.currentTimeMillis() - lastTagSeenMs < 500);
            if (poseKnown) telemetry.addData("Seeded pose", formatPose(follower.getPose()));
            telemetry.update();
        }

        if (isStopRequested()) return;

        follower.startTeleopDrive(true);
        follower.update();

        while (opModeIsActive()) {
            follower.update();      // odometry pods integrate every loop, always
            tryVisionUpdate();      // vision corrects it whenever the red goal is in frame
            handleVisionButtons();
            drive();
            telemetryOut();
        }

        limelight.stop();
    }

    // ---------------- INIT ----------------

    private void initHardware() {
        follower = Constants.createFollower(hardwareMap);
        // We genuinely don't know where we are yet. This is a placeholder origin; the first
        // AprilTag fix overwrites it, and poseKnown gates anything that depends on it.
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.update();

        headingController = new PIDFController(follower.constants.coefficientsHeadingPIDF);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    // ---------------- VISION ----------------

    /**
     * Looks for the red goal tag and folds its field-space solve into the follower's pose.
     * No-op when the tag isn't visible - odometry just keeps running.
     */
    private void tryVisionUpdate() {
        long now = System.currentTimeMillis();
        if (now - lastVisionUpdateMs < VISION_UPDATE_INTERVAL_MS) return;
        lastVisionUpdateMs = now;

        Pose visionPose = readRedGoalPose();
        if (visionPose == null) return;

        lastTagSeenMs = now;

        if (!poseKnown) {
            // First fix: nothing to blend against, take it whole.
            follower.setPose(visionPose);
            poseKnown = true;
            visionFixCount++;
            lastCorrectionInches = 0;
            return;
        }

        if (!visionCorrectionEnabled) return;

        Pose current = follower.getPose();
        double dist = Math.hypot(visionPose.getX() - current.getX(), visionPose.getY() - current.getY());
        if (dist > VISION_OUTLIER_INCHES) return; // almost certainly a bad solve, keep odometry

        // Blend, taking the shortest way around for heading so 179 -> -179 doesn't spin the estimate.
        double headingDelta = MathFunctions.getTurnDirection(current.getHeading(), visionPose.getHeading())
                * MathFunctions.getSmallestAngleDifference(current.getHeading(), visionPose.getHeading());

        follower.setPose(new Pose(
                current.getX() + VISION_BLEND * (visionPose.getX() - current.getX()),
                current.getY() + VISION_BLEND * (visionPose.getY() - current.getY()),
                MathFunctions.normalizeAngle(current.getHeading() + VISION_BLEND * headingDelta)
        ));

        visionFixCount++;
        lastCorrectionInches = dist;
    }

    /** @return robot pose in Pedro coordinates from the red goal tag, or null if it isn't visible. */
    private Pose readRedGoalPose() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) return null;

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial.getFiducialId() != RED_GOAL_TAG_ID) continue;

            Pose3D robotField = fiducial.getRobotPoseFieldSpace();
            if (robotField == null) return null; // field map not loaded on the Limelight

            return limelightToPedro(
                    robotField.getPosition().x,
                    robotField.getPosition().y,
                    robotField.getOrientation().getYaw()
            );
        }
        return null;
    }

    /**
     * Limelight field space (meters, origin at field center) -> Pedro (inches, origin at a
     * field corner). Same mapping the team's FusedPose uses.
     */
    private static Pose limelightToPedro(double xMeters, double yMeters, double yawDegrees) {
        return new Pose(
                72 + 39.3701 * yMeters,
                72 - 39.3701 * xMeters,
                MathFunctions.normalizeAngle(Math.toRadians(yawDegrees - 90))
        );
    }

    private void handleVisionButtons() {
        // dpad up: toggle continuous correction (off = pure odometry after the initial fix)
        if (gamepad1.dpad_up && !prevDpadUp) visionCorrectionEnabled = !visionCorrectionEnabled;
        prevDpadUp = gamepad1.dpad_up;

        // dpad down: throw away the fix and re-seed from the next sighting
        if (gamepad1.dpad_down && !prevDpadDown) poseKnown = false;
        prevDpadDown = gamepad1.dpad_down;
    }

    // ---------------- DRIVE ----------------

    private void drive() {
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        updateAlignState(turn);

        if (aligning) {
            follower.setTeleOpDrive(forward, strafe, computeAlignTurn(), ROBOT_CENTRIC);
        } else {
            follower.setTeleOpDrive(forward, strafe, turn, ROBOT_CENTRIC);
        }
    }

    /** Decides whether the heading PID owns the turn axis this loop. */
    private void updateAlignState(double driverTurn) {
        boolean aPressed = gamepad1.a;

        if (!aPressed) {
            // Releasing A always drops back to normal driving and re-arms for the next press.
            aligning = false;
            alignDone = false;
            return;
        }

        // Can't aim at the goal if we don't know where we are.
        if (!poseKnown || alignDone) {
            aligning = false;
            return;
        }

        // Driver grabbing the turn stick wins.
        if (Math.abs(driverTurn) > TURN_TAKEOVER_DEADBAND) {
            aligning = false;
            alignDone = true; // stays off until A is released, so it can't fight the driver
            return;
        }

        if (!aligning) {
            aligning = true;
            alignInToleranceSince = 0;
            headingController.reset();
        }
    }

    private double computeAlignTurn() {
        Pose pose = follower.getPose();
        targetHeading = MathFunctions.normalizeAngle(
                Math.atan2(RED_GOAL_Y - pose.getY(), RED_GOAL_X - pose.getX())
                        + Math.toRadians(AIM_HEADING_OFFSET_DEG)
        );

        double error = MathFunctions.getTurnDirection(pose.getHeading(), targetHeading)
                * MathFunctions.getSmallestAngleDifference(pose.getHeading(), targetHeading);

        // "Close enough" - hold it inside tolerance briefly so we don't quit on a single
        // frame while still swinging through the target.
        long now = System.currentTimeMillis();
        if (Math.abs(error) <= Math.toRadians(ALIGN_TOLERANCE_DEG)) {
            if (alignInToleranceSince == 0) alignInToleranceSince = now;
            if (now - alignInToleranceSince >= ALIGN_SETTLE_MS) {
                aligning = false;
                alignDone = true;
                return 0;
            }
        } else {
            alignInToleranceSince = 0;
        }

        headingController.setCoefficients(follower.constants.coefficientsHeadingPIDF);
        headingController.updateError(error);
        return MathFunctions.clamp(headingController.run(), -MAX_ALIGN_TURN, MAX_ALIGN_TURN);
    }

    // ---------------- TELEMETRY ----------------

    private void telemetryOut() {
        Pose pose = follower.getPose();

        telemetry.addData("Pose", poseKnown ? formatPose(pose) : "UNKNOWN - show the red goal tag");
        telemetry.addData("Mode", aligning ? "ALIGNING TO GOAL"
                : (gamepad1.a && alignDone ? "aligned / driver override" : "manual drive"));

        if (poseKnown) {
            double bearing = Math.toDegrees(MathFunctions.normalizeAngle(
                    Math.atan2(RED_GOAL_Y - pose.getY(), RED_GOAL_X - pose.getX())));
            double err = Math.toDegrees(MathFunctions.getTurnDirection(pose.getHeading(), targetHeading)
                    * MathFunctions.getSmallestAngleDifference(pose.getHeading(), targetHeading));
            telemetry.addData("Goal bearing", "%.1f deg", bearing);
            telemetry.addData("Heading error", "%.1f deg", err);
            telemetry.addData("Distance to goal", "%.1f in",
                    Math.hypot(RED_GOAL_X - pose.getX(), RED_GOAL_Y - pose.getY()));
        }

        telemetry.addLine("--- vision ---");
        telemetry.addData("Tag 24 visible", System.currentTimeMillis() - lastTagSeenMs < 500);
        telemetry.addData("Correction (dpad up)", visionCorrectionEnabled ? "ON" : "OFF");
        telemetry.addData("Fixes applied", visionFixCount);
        telemetry.addData("Last correction", "%.1f in", lastCorrectionInches);

        telemetry.addLine("--- controls ---");
        telemetry.addLine("A (hold): face red goal");
        telemetry.addLine("dpad up: toggle vision correction");
        telemetry.addLine("dpad down: re-seed pose from next tag");
        telemetry.update();
    }

    private static String formatPose(Pose pose) {
        return String.format("x %.1f  y %.1f  h %.1f deg",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }
}
