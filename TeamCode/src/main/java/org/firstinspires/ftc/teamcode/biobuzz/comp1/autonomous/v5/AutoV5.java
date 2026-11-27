package org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.v5;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher.FlywheelSpeedTable;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher.advanced.Flywheel;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.RedHiveGoal;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.TurretController;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.Hive;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.HiveTilt;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.TagObservation;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.TiltState;

/**
 * Auto V5 (RED), the full auto: paths from "yeehw.pp (2).json" (PathsV5) with intake, transfer, launcher and turret.
 *
 * THE PLAN - the robot starts on the side that is NOT shooting, and the HIVE tips TWICE:
 *   The teammate fills the far cell with 4, the HIVE tips, and OUR cell (top / SCORING, +Y side) comes up.
 *   1. Wait for the HIVE to tip to our side (bounded: TIP_TIMEOUT_SEC, then go anyway)
 *   2. Shoot the 4 preloads into it
 *   3. Path 1 -> lined up in front of the far wall FLOWER, then straight INTO the FLOWER (slower)
 *   4. Ramp down, intake those 4
 *   5. Shoot them -> our cell now holds 8 and the HIVE tips the SECOND time
 *   6. Path 2 -> red wall FLOWER  (the drive doubles as the wait for that second tip)
 *   7. Intake those 4
 *   8. Path 3 -> shooting spot
 *   8b. TURN IN PLACE to face the HIVE and confirm the second tip (the camera is fixed to the chassis and every pose
 *       on the south leg faces away from it), then turn back to the path heading. Telemetry reports the result; it
 *       shoots anyway if the camera never confirms.
 *   9. Shoot them into the bottom cell that is now up
 *  10. Path 4 -> corner with the ramp still down, intaking the garden balls on the way (+ a short hold)
 *  11. Shoot those from the corner (same bottom cell)
 *  12. Ramp up, intake off, path 5 -> park
 *
 * The whole time: launcher at the speed for the current turret-to-cell distance (same table as the TeleOp), turret
 * aiming at the red HIVE cell on the robot's half of the field (odometry only), intake on, ramp down while
 * collecting and only up again for the park run.
 *
 * BEFORE PRESSING INIT
 *   1. Robot at PathsV5.start (29.0, 133.7) facing the audience (heading 270), STILL while the Pinpoint calibrates.
 *   2. TURRET POINTING STRAIGHT OUT THE ROBOT FRONT (its encoder is zeroed at INIT).
 *   3. 4 preloads loaded. Nothing moves until START.
 */
@Autonomous(name = "BioBuzz Auto V5 (full)", group = "BioBuzz")
public class AutoV5 extends LinearOpMode {

    // ======================= TIMING (tune) =======================
    /**
     * Wait this long for the HIVE to tip to our side, then go anyway. Never wait forever: a camera that sees nothing
     * would cost the whole 30 s and every point. (Was POSITIVE_INFINITY, which contradicts "then go anyway".)
     */
    private static final double TIP_TIMEOUT_SEC = 8.0;
    /** Frames in a row that must agree before a tip counts (used by both tips). */
    private static final int TIP_CONFIRM_FRAMES = 5;
    /**
     * SECOND TIP. Our 8 balls tip the HIVE so the BOTTOM (audience) cell comes up for volleys 3 and 4. The camera
     * points out the robot front and every pose on the south leg faces away from the HIVE, so the auto turns in place
     * to look at it, waits for AUDIENCE_UP, then turns back to the heading the path ended on. The turret does the
     * aiming, so the chassis heading never changes where the shot goes.
     */
    private static final double SECOND_TIP_TIMEOUT_SEC = 12.0;
    /** Each half of that turn (about 110 deg) may take at most this long. */
    private static final double TURN_MAX_SEC = 5.0;
    /** A turn stops within this many degrees of the wanted heading. */
    private static final double TURN_TOL_DEG = 3.0;
    /** Chassis turning is slow and wobbly near full stick, so the turn command is proportional and capped. */
    private static final double TURN_GAIN_PER_DEG = 0.02;
    private static final double TURN_POWER = 0.35;
    /** Before feeding: wait (up to this) for the launcher to be at speed and the turret on target. */
    private static final double READY_TIMEOUT_SEC = 1.0;
    /**
     * How long the transfer feeds per volley. 2.0 s was long enough for 4 balls at the old -1.0 feed; the transfer
     * now feeds at -0.8, so the same 4 balls need ~2.5 s. (Change these two together.)
     */
    private static final double SHOOT_SEC = 2.5;
    /** Hard cap on one volley, fed time plus any paused time, so it can never stall the auto. */
    private static final double SHOOT_MAX_SEC = 5.0;
    /** Parked at a FLOWER pulling balls out. */
    private static final double FLOWER_INTAKE_SEC = 1.5;
    /** Extra time sitting in the corner after path 4 to finish grabbing balls. */
    private static final double CORNER_INTAKE_SEC = 0.5;
    /** Speed for the straight drive into the far wall FLOWER, fraction of top speed (1.0 = full). */
    private static final double INTO_FLOWER_SPEED = 0.4;
    /** Give up on a path after this long so a stuck robot still runs the rest of the auto. */
    private static final double PATH_TIMEOUT_SEC = 6.0;

    // ======================= MECHANISMS (same as the TeleOp) =======================
    private static final double INTAKE_POWER = 1.0;          // + = in
    private static final double TRANSFER_FEED_POWER = -0.7;  // - = up into the launcher
    private static final double TRANSFER_DOWN_POWER = 0.5;   // down, away from the launcher, when not shooting
    private static final double RAMP_DOWN = 0.140;
    private static final double RAMP_UP = 0.730;   // same as the TeleOp
    /** Ready to feed when the launcher is within this of its target and the turret within TURRET_READY_DEG. */
    private static final double AT_SPEED_FRACTION = 0.03;
    private static final double TURRET_READY_DEG = 1.0; // wait for a tight aim before feeding

    private Follower follower;
    private TurretController turret;
    private Flywheel launcher;
    private DcMotor intake, transfer;
    private Servo flowerServo;
    private Limelight3A limelight;
    private RedHiveGoal goal;

    private boolean intakeOn = true;
    private boolean feeding = false;
    /** True for the whole of one volley, so a pause between balls holds the transfer still instead of reversing it. */
    private boolean volley = false;
    private boolean mechanismsOn = false;
    private double launchTps = FlywheelSpeedTable.DEFAULT_TPS;
    private double turretTarget = Double.NaN;
    private String step = "init";
    /** Latest RED HIVE tilt reading (tagCount 0 = nothing visible this frame). */
    private HiveTilt lastRedTilt;
    /** Result of the look-before-the-third-volley: is the BOTTOM cell really up? */
    private boolean secondTipConfirmed = false;

    @Override
    public void runOpMode() {
        turret = new TurretController(hardwareMap); // zeroes the turret encoder: must face FRONT
        follower = Constants.create(hardwareMap);
        follower.setPose(PathsV5.start);
        goal = new RedHiveGoal(PathsV5.start.y());

        launcher = new Flywheel();
        launcher.init(hardwareMap);
        intake = hardwareMap.dcMotor.get("intakeMotor");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer = hardwareMap.dcMotor.get("transferMotor");
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        flowerServo = hardwareMap.get(Servo.class, "flowerServo"); // not moved until START

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(HiveTagGeometry.APRILTAG_PIPELINE);
        limelight.start();

        // Build the paths during INIT so START is instant
        PathsV5 paths = new PathsV5();
        Path path1 = paths.path1(), intoFlower = paths.intoFlower(), path2 = paths.path2(), path3 = paths.path3(),
                path4 = paths.path4(), path5 = paths.path5();

        // INIT: odometry only (nothing moves). Check the start pose on telemetry.
        while (opModeInInit()) {
            follower.update();
            Pose p = follower.pose();
            telemetry.addLine("Auto V5 ready. Turret facing robot FRONT? 4 preloads in?");
            telemetry.addData("Pose", "(%.1f, %.1f) %.1f°  (should be 29.0, 133.7, 270)",
                    p.x(), p.y(), Math.toDegrees(p.heading()));
            telemetry.update();
        }
        if (isStopRequested()) {
            limelight.stop();
            return;
        }

        // START: everything on
        mechanismsOn = true;
        flowerServo.setPosition(RAMP_DOWN);                              // 1. ramp down right away
        intakeOn = true;

        waitForRedHive();                                                // 1. teammate's 4 tip it to our side
        // Limelight stays on: the second tip is confirmed from the south leg (see confirmSecondTip).
        shoot("2. Shooting preloads");                                   // 2.

        runPath(path1, "3. Path 1 -> in front of far wall FLOWER");       // 3.
        runPath(intoFlower, "3. Into the FLOWER (slow)", INTO_FLOWER_SPEED);
        holdFor(FLOWER_INTAKE_SEC, "4. Intaking from FLOWER");           // 4.
        shoot("5. Shooting FLOWER balls");                               // 5. these 8 balls tip the HIVE again

        runPath(path2, "6. Path 2 -> red wall FLOWER");                  // 6.
        holdFor(FLOWER_INTAKE_SEC, "7. Intaking from FLOWER");           // 7.
        runPath(path3, "8. Path 3 -> shooting spot");                    // 8.
        confirmSecondTip();                                              // 8b. look at the HIVE, then turn back
        shoot("9. Shooting");                                            // 9.

        // Ramp stays DOWN through the corner so it can pick up the garden balls; it only comes up before parking.
        runPath(path4, "10. Path 4 -> corner (intaking)");               // 10.
        holdFor(CORNER_INTAKE_SEC, "10. Intaking in corner");
        shoot("11. Shooting from corner");                               // 11.

        intakeOn = false;                                                // 12.
        flowerServo.setPosition(RAMP_UP);                                // ramp up for the park run
        runPath(path5, "12. Path 5 -> park");

        // Done: hold the park spot, everything else off
        mechanismsOn = false;
        step = "Done - parked";
        while (opModeIsActive()) {
            updateRobot();
            telemetry.update();
        }
        stopAll();
    }

    // ======================= STEPS =======================

    /** From the far wall the red tags face away until the HIVE tips, so the first red tag seen = it came down. */
    private void waitForRedHive() {
        ElapsedTime timer = new ElapsedTime();
        int frames = 0;
        while (opModeIsActive() && frames < TIP_CONFIRM_FRAMES && timer.seconds() < TIP_TIMEOUT_SEC) {
            frames = seesRedTag() ? frames + 1 : 0;
            step = String.format("1. Waiting for red HIVE to tip (%.1f s, red tag %d/%d)",
                    timer.seconds(), frames, TIP_CONFIRM_FRAMES);
            updateRobot();
            telemetry.update();
        }
    }

    private boolean seesRedTag() {
        for (TagObservation tag : HiveTagGeometry.readTags(limelight.getLatestResult())) {
            if (tag.info.hive == Hive.RED) return true;
        }
        return false;
    }

    /**
     * SECOND TIP CHECK. Turns the chassis in place to face the HIVE, waits (bounded) until it reports AUDIENCE_UP for
     * TIP_CONFIRM_FRAMES frames in a row, then turns back to the heading the path ended on. The turret keeps aiming
     * and the flywheel keeps spinning the whole time, so this costs match time but not the shot.
     *
     * If it never confirms, the auto still shoots (a cell that filled up is far likelier than a camera that saw
     * nothing); telemetry says which happened.
     */
    private boolean confirmSecondTip() {
        Pose pose = follower.pose();
        double savedHeadingDeg = Math.toDegrees(pose.heading());
        double bearingDeg = Math.toDegrees(Math.atan2(HiveTagGeometry.PIVOT_Y_IN - pose.y(), goal.x() - pose.x()));

        step = "8b. Turning to look at the HIVE (second tip)";
        turnToHeading(bearingDeg, TURN_MAX_SEC);

        ElapsedTime timer = new ElapsedTime();
        int frames = 0;
        while (opModeIsActive() && timer.seconds() < SECOND_TIP_TIMEOUT_SEC) {
            frames = (lastRedTilt.tagCount > 0 && lastRedTilt.state == TiltState.AUDIENCE_UP) ? frames + 1 : 0;
            step = String.format("8b. Checking the second tip (%.1f s, %s %d/%d)",
                    timer.seconds(), tiltLabel(), frames, TIP_CONFIRM_FRAMES);
            updateRobot();
            telemetry.update();
            if (frames >= TIP_CONFIRM_FRAMES) break;
        }
        secondTipConfirmed = frames >= TIP_CONFIRM_FRAMES;

        step = "8b. Turning back to the shooting heading";
        turnToHeading(savedHeadingDeg, TURN_MAX_SEC);
        return secondTipConfirmed;
    }

    /** Turns the chassis in place to headingDeg, bounded by maxSec so it can never spin forever. */
    private void turnToHeading(double headingDeg, double maxSec) {
        ElapsedTime timer = new ElapsedTime();
        double errDeg = headingErrorDeg(headingDeg);
        while (opModeIsActive() && timer.seconds() < maxSec && Math.abs(errDeg) > TURN_TOL_DEG) {
            double turn = Math.max(-TURN_POWER, Math.min(TURN_POWER, errDeg * TURN_GAIN_PER_DEG));
            follower.manual(0, 0, turn); // positive = counter-clockwise, same as heading
            updateRobot();
            telemetry.update();
            errDeg = headingErrorDeg(headingDeg);
        }
        follower.manual(0, 0, 0);
        updateRobot();
    }

    /** Shortest signed turn left to reach headingDeg, in degrees. */
    private double headingErrorDeg(double headingDeg) {
        return Math.toDegrees(HiveTagGeometry.normalizeAngle(
                Math.toRadians(headingDeg) - follower.pose().heading()));
    }

    /** Latest RED HIVE tilt from the current Limelight frame (tagCount 0 / UNKNOWN when nothing is visible). */
    private void readTilt() {
        lastRedTilt = HiveTagGeometry.measureHive(HiveTagGeometry.readTags(limelight.getLatestResult()), Hive.RED);
    }

    private String tiltLabel() {
        return lastRedTilt == null || lastRedTilt.tagCount == 0 ? "not seen"
                : String.format("%s (%.1f°, %d tags)", lastRedTilt.state.label, lastRedTilt.tiltDeg, lastRedTilt.tagCount);
    }

    /**
     * Feeds one volley. First waits (briefly, bounded) until the wheel is at speed and the turret is on target, then
     * feeds only while that is still true, counting only the time actually spent feeding. A dip between balls pauses
     * the transfer (holding it still, not reversing it) instead of pushing a ball into a slow wheel, and the fed time
     * is still the full SHOOT_SEC, so all 4 balls get out.
     */
    private void shoot(String status) {
        ElapsedTime timer = new ElapsedTime();
        step = status + " (getting ready)";
        while (opModeIsActive() && timer.seconds() < READY_TIMEOUT_SEC && !readyToShoot()) {
            updateRobot();
            telemetry.update();
        }

        volley = true;
        feeding = false;
        timer.reset();
        double fed = 0, prev = timer.seconds();
        while (opModeIsActive() && timer.seconds() < SHOOT_MAX_SEC && fed < SHOOT_SEC) {
            feeding = readyToShoot();
            updateRobot();
            double now = timer.seconds();
            if (feeding) fed += now - prev;
            prev = now;
            step = String.format("%s (fed %.1f / %.1f s)", status, fed, SHOOT_SEC);
            telemetry.update();
        }
        feeding = false;
        volley = false;
    }

    private boolean readyToShoot() {
        boolean atSpeed = launcher.getTicksPerSec() >= launchTps * (1 - AT_SPEED_FRACTION);
        boolean aimed = !Double.isNaN(turretTarget) && Math.abs(turretTarget - turret.angleDeg()) <= TURRET_READY_DEG;
        return atSpeed && aimed;
    }

    private void runPath(Path path, String status) {
        runPath(path, status, 1.0);
    }

    /** speedFraction: 0..1 of top speed for just this path (Foresight's maxPathSpeed), restored afterwards. */
    private void runPath(Path path, String status, double speedFraction) {
        step = status;
        Constants.foresightConfig.maxPathSpeed.set(speedFraction >= 1.0 ? ForesightConfig.Constraint.NONE : speedFraction);
        try {
            follower.follow(path);
            ElapsedTime timer = new ElapsedTime();
            while (opModeIsActive() && follower.isBusy() && timer.seconds() < PATH_TIMEOUT_SEC) {
                updateRobot();
                telemetry.update();
            }
        } finally {
            Constants.foresightConfig.maxPathSpeed.set(ForesightConfig.Constraint.NONE);
        }
    }

    /** Holds the end of the last path (the follower keeps holding) while everything keeps running. */
    private void holdFor(double seconds, String status) {
        step = status;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            updateRobot();
            telemetry.update();
        }
    }

    // ======================= EVERY LOOP =======================

    /** Drive, aim the turret, run the launcher / intake / transfer. Call once per loop. */
    private void updateRobot() {
        follower.update();
        Pose pose = follower.pose();
        turret.updateHeading(pose.heading());
        goal.update(pose.y());
        readTilt(); // keep the HIVE line in telemetry live, and feed the second-tip check

        if (!mechanismsOn) {
            turret.stop();
            launcher.stopMotors();
            intake.setPower(0);
            transfer.setPower(0);
            addTelemetry(pose);
            return;
        }

        turretTarget = turret.aimAt(pose.x(), pose.y(), pose.heading(), goal.x(), goal.y());

        // Launcher speed from the turret-to-cell distance (same table as the TeleOp)
        double h = pose.heading();
        double tx = pose.x() + TurretConstants.TURRET_FORWARD_IN * Math.cos(h) - TurretConstants.TURRET_LEFT_IN * Math.sin(h);
        double ty = pose.y() + TurretConstants.TURRET_FORWARD_IN * Math.sin(h) + TurretConstants.TURRET_LEFT_IN * Math.cos(h);
        launchTps = FlywheelSpeedTable.velocityFor(Math.hypot(goal.x() - tx, goal.y() - ty));
        launcher.setMotorRPM(Flywheel.ticksPerSecToRpm(launchTps)); // must run every loop

        intake.setPower(intakeOn ? INTAKE_POWER : 0);
        // Feeding up, holding still while a volley waits for the wheel, or down when we are not shooting.
        transfer.setPower(feeding ? TRANSFER_FEED_POWER : volley ? 0 : TRANSFER_DOWN_POWER);
        addTelemetry(pose);
    }

    private void stopAll() {
        turret.stop();
        launcher.stopMotors();
        intake.setPower(0);
        transfer.setPower(0);
        limelight.stop();
    }

    private void addTelemetry(Pose pose) {
        telemetry.addData("Step", step);
        telemetry.addData("HIVE (red)", tiltLabel());
        telemetry.addData("Second tip", secondTipConfirmed
                ? "CONFIRMED - bottom cell up" : "not seen (checked at step 8b)");
        telemetry.addData("Pose", "(%.1f, %.1f) %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Aiming at", "%s cell (%.1f, %.1f)", goal.aimingTop() ? "TOP" : "BOTTOM", goal.x(), goal.y());
        telemetry.addData("Turret", "%.1f° -> %s", turret.angleDeg(),
                Double.isNaN(turretTarget) ? "-" : String.format("%.1f°", turretTarget));
        telemetry.addData("Launcher", "%.0f / %.0f ticks/s%s", launcher.getTicksPerSec(), launchTps,
                launcher.isBoosting() ? "  BOOST" : "");
        telemetry.addData("Intake / transfer", "%s / %s", intakeOn ? "IN" : "off",
                feeding ? "FEEDING" : volley ? "waiting" : "down");
        telemetry.addData("Limelight", limelight.isRunning() ? "on" : "off");
    }
}