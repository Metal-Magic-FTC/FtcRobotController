package org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.nihalsautos;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.nihalsautos.RedTipPaths.Place;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.Hive;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.HiveTilt;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.TiltState;

import java.util.ArrayList;
import java.util.List;

/**
 * RED Adapt: "always shoot the up cell", going for as many tips as time allows.
 *
 *   Set 1 (preloads) -> cell A from the START spot (tip 1).
 *   Then for each pickup (garden, wall flower, far flower):
 *     1. Pick up 4.
 *     2. Drive to the spot for the cell we PREDICT is up (START shoots A, LAUNCH_B shoots B).
 *     3. Check the HIVE before shooting. Wrong cell up -> switch spots if there's time, else keep the pollen.
 *     4. Shoot, then check again: that reading is the prediction for the next set.
 *   Park (27.5 s park guard still applies).
 *
 * The very first prediction comes from the PARTNER setting picked in INIT (dpad up/down):
 *   partner 8 -> they tip B themselves, so A is up -> garden set goes to A from START.
 *   partner 0 or 4 -> B is up -> garden set goes to B from LAUNCH_B.
 * The HIVE check fixes it if the partner does something else.
 *
 * All positions and path speeds live in RedTipPaths.
 */
@Autonomous(name = "RED Auto Adapt v1", group = "BioBuzz")
public class RedAutoAdaptV1 extends LinearOpMode {

    // =====================================================================================
    // TIMING
    // =====================================================================================
    private static final long   SHOOT_MS          = 1800; // one full volley of 4
    private static final long   GARDEN_DWELL_MS   = 200;  // extra intake time after the garden sweep
    private static final long   FLOWER_INTAKE_MS  = 2700; // pulling 4 out of a flower
    private static final double LEG_TIMEOUT_S     = 6.0;  // safety: never get stuck on one path
    private static final double PARK_AT_S         = 27.5; // drop everything and park at this match time
    /** Rough time to drive START <-> LAUNCH_B through the lane. Used to decide if switching spots is worth it. */
    private static final double SWITCH_SPOT_S     = 4.5;

    // =====================================================================================
    // HIVE CHECKS (Limelight via HiveTagGeometry)
    // =====================================================================================
    private static final boolean USE_HIVE_CHECK     = true;  // false = trust the predictions only
    private static final int     STILL_READS        = 3;     // same stable state this many reads in a row = settled
    private static final double  FIRST_CHECK_WAIT_S = 1.7;   // first check after the partner shoots (more swinging)
    private static final double  PRE_CHECK_WAIT_S   = 1.0;   // before each shot
    private static final double  POST_CHECK_WAIT_S  = 0.8;   // right after each shot

    // =====================================================================================
    // HARDWARE
    // =====================================================================================
    private DcMotorEx launchMotor;
    private DcMotor intakeMotor;
    private Servo hoodServo;
    private Limelight3A limelight;

    private static final double LAUNCH_VEL_A = 1800; // tune: shot into cell A from START
    private static final double LAUNCH_VEL_B = 1600; // tune: shot into cell B from LAUNCH_B
    private static final double HOOD_A       = 0.80; // tune
    private static final double HOOD_B       = 0.80; // tune
    private static final double INTAKE_POWER = -0.6;
    private static final double FEED_POWER   = -1.0;

    private void initHardware() {
        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 12.9));
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        hoodServo   = hardwareMap.get(Servo.class, "hoodServo");
        limelight   = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(HiveTagGeometry.APRILTAG_PIPELINE);
        limelight.start();
    }

    private void launcherFor(Cell c) {
        if (c == Cell.A) { launchMotor.setVelocity(LAUNCH_VEL_A); hoodServo.setPosition(HOOD_A); }
        else             { launchMotor.setVelocity(LAUNCH_VEL_B); hoodServo.setPosition(HOOD_B); }
    }
    private void intakeOn()  { intakeMotor.setPower(INTAKE_POWER); }
    private void intakeOff() { intakeMotor.setPower(0); }

    /** TODO: replace with your BIOBUZZ feed code. It must empty all 4 pollen in about SHOOT_MS. */
    private void feedStart() { intakeMotor.setPower(FEED_POWER); }
    private void feedStop()  { intakeMotor.setPower(0); }

    // =====================================================================================
    // STATE
    // =====================================================================================
    private enum Cell { A, B }
    private enum CellState { UP, DOWN, MOVING, UNSEEN } // about cell B

    private static final Place[] PICKUPS = { Place.GARDEN, Place.WALL_FLOWER, Place.FAR_FLOWER };

    private Follower follower;
    private RedTipPaths paths;
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final List<String> stepLog = new ArrayList<>();

    private int partnerBalls = 4;     // picked in INIT: 0, 4 or 8
    private Place at = Place.START;   // shooting spot the robot is at (or last left)
    private Cell predictedUp;         // which cell we think is up right now
    private int volleys = 0;
    private int tipsSeen = 0;         // tips confirmed by a check right after our shot
    private boolean firstCheck = true;

    private CellState lastCellB = CellState.UNSEEN;
    private double lastTiltDeg = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        follower = Constants.create(hardwareMap);
        paths = new RedTipPaths(); // built during init so start is instant
        paths.setStartPose(follower);
        hoodServo.setPosition(HOOD_A);

        boolean lastUp = false, lastDown = false;
        while (opModeInInit()) {
            if (gamepad1.dpad_up && !lastUp)     partnerBalls = Math.min(8, partnerBalls + 4);
            if (gamepad1.dpad_down && !lastDown) partnerBalls = Math.max(0, partnerBalls - 4);
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            follower.update();
            readHiveOnce();
            telemetry.addLine("RED Adapt. Robot at the START spot on the audience wall.");
            telemetry.addData("Partner scores into B", "%d   (dpad up/down)", partnerBalls);
            telemetry.addData("Garden set goes to", partnerBalls >= 8 ? "cell A (START)" : "cell B (LAUNCH_B)");
            telemetry.addData("Red HIVE", "%s (%.1f°)", lastCellB, lastTiltDeg);
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested()) {
            limelight.stop();
            return;
        }

        matchTimer.reset();
        try {
            runAuto();
        } catch (ParkNow e) {
            parkNow();
        }
        finish();
    }

    /** The whole auto. Any wait/path loop throws ParkNow once the clock reaches PARK_AT_S. */
    private void runAuto() {
        // Set 1: preloads into cell A from START (tip 1).
        launcherFor(Cell.A);
        shoot(Cell.A);
        log("1 shot A (tip 1)");

        // After tip 1, B is up. A partner 8 tips it back, so A would be up again.
        predictedUp = partnerBalls >= 8 ? Cell.A : Cell.B;

        for (Place pickup : PICKUPS) {
            collect(pickup);

            // Drive to the spot for the cell we think is up (spin the launcher up on the way).
            Place spot = spotFor(predictedUp);
            launcherFor(predictedUp);
            runPath(paths.pickupToSpot(pickup, spot), paths.pickupToSpotSpeed(pickup, spot));
            at = spot;

            // Check BEFORE shooting.
            Cell up = checkUpCell(firstCheck ? FIRST_CHECK_WAIT_S : PRE_CHECK_WAIT_S);
            firstCheck = false;
            if (up != null && up != cellAt(at)) {
                if (matchTimer.seconds() + SWITCH_SPOT_S + SHOOT_MS / 1000.0 <= PARK_AT_S) {
                    log("check: " + up + " up, switch spot");
                    launcherFor(up);
                    runPath(paths.switchSpot(at), RedTipPaths.FAST);
                    at = spotFor(up);
                } else {
                    log("check: " + up + " up, no time, keep");
                    predictedUp = up;
                    continue;
                }
            } else {
                log("check: " + (up == null ? "unsure, trust plan" : up + " up"));
            }

            // Shoot the up cell, then check AFTER to see if it tipped.
            Cell target = cellAt(at);
            shoot(target);
            log(volleys + " shot " + target);

            Cell after = checkUpCell(POST_CHECK_WAIT_S);
            if (after == null) after = other(target); // still swinging / not seen = assume it tipped
            if (after != target) {
                tipsSeen++;
                log("tip after set " + volleys);
            } else {
                log("no tip after set " + volleys);
            }
            predictedUp = after;
        }

        startParking();
        runPath(paths.parkFromSpot(at), RedTipPaths.FAST);
        log("parked");
    }

    /** Pick up 4 from a garden/flower, starting from the current shooting spot. */
    private void collect(Place pickup) {
        intakeOn();
        runPath(paths.toPickup(at, pickup), paths.toPickupSpeed(at, pickup));
        Path sweep = paths.pickupSweep(pickup);
        if (sweep != null) runPath(sweep, paths.pickupSweepSpeed(pickup));
        holdMs(pickup == Place.GARDEN ? GARDEN_DWELL_MS : FLOWER_INTAKE_MS);
        intakeOff();
        log(pickup + " loaded");
    }

    private void shoot(Cell c) {
        feedStart();
        holdMs(SHOOT_MS);
        feedStop();
        volleys++;
    }

    private static Place spotFor(Cell c) { return c == Cell.A ? Place.START : Place.LAUNCH_B; }
    private static Cell cellAt(Place spot) { return spot == Place.START ? Cell.A : Cell.B; }
    private static Cell other(Cell c)     { return c == Cell.A ? Cell.B : Cell.A; }

    /** Which cell is up, or null if the HIVE never settled / wasn't seen in time. */
    private Cell checkUpCell(double maxWaitS) {
        if (!USE_HIVE_CHECK) return null;
        CellState b = readCellB(maxWaitS);
        if (b == CellState.UP) return Cell.B;
        if (b == CellState.DOWN) return Cell.A;
        return null;
    }

    // =====================================================================================
    // PARK GUARD
    // =====================================================================================
    private static class ParkNow extends RuntimeException {}

    private boolean parking = false;

    private void checkParkClock() {
        if (!parking && matchTimer.seconds() >= PARK_AT_S) throw new ParkNow();
    }

    private void startParking() {
        parking = true;
        feedStop();
        intakeOff();
        launchMotor.setVelocity(0);
    }

    private void parkNow() {
        startParking();
        log(String.format("! %.1f s: park now", PARK_AT_S));
        runPath(paths.parkFromHere(follower), RedTipPaths.FAST);
        log("parked");
    }

    private void finish() {
        limelight.stop();
        while (opModeIsActive()) { follower.update(); showLog(); }
    }

    // =====================================================================================
    // HELPERS
    // =====================================================================================

    /** Follows a path to the end (the follower keeps holding the end pose). speedFraction is 0..1 of top speed. */
    private void runPath(Path path, double speedFraction) {
        Constants.foresightConfig.maxPathSpeed.set(
                speedFraction >= 1.0 ? ForesightConfig.Constraint.NONE : speedFraction);
        try {
            follower.follow(path);
            ElapsedTime t = new ElapsedTime();
            while (opModeIsActive() && follower.isBusy() && t.seconds() < LEG_TIMEOUT_S) {
                checkParkClock();
                follower.update();
                showLog();
            }
        } finally {
            Constants.foresightConfig.maxPathSpeed.set(ForesightConfig.Constraint.NONE);
        }
    }

    private void holdMs(long ms) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.milliseconds() < ms) { checkParkClock(); follower.update(); }
    }

    /** Watches the red HIVE until it settles. UP/DOWN (cell B) once settled, MOVING if it never settled, UNSEEN if not seen. */
    private CellState readCellB(double maxWaitS) {
        ElapsedTime t = new ElapsedTime();
        CellState prev = null;
        int still = 0;
        boolean seen = false;
        while (opModeIsActive() && t.seconds() < maxWaitS) {
            checkParkClock();
            follower.update();
            CellState now = readHiveOnce();
            if (now != CellState.UNSEEN) {
                seen = true;
                still = (now == prev && now != CellState.MOVING) ? still + 1 : 0;
                prev = now;
                if (still >= STILL_READS) return now;
            }
            sleep(40);
        }
        return seen ? CellState.MOVING : CellState.UNSEEN;
    }

    /** One Limelight read of the red HIVE. SCORING cell up = cell B UP. */
    private CellState readHiveOnce() {
        HiveTilt tilt = HiveTagGeometry.measureHive(
                HiveTagGeometry.readTags(limelight.getLatestResult()), Hive.RED);
        TiltState s = tilt.state;
        lastTiltDeg = tilt.tiltDeg;
        if (s == TiltState.SCORING_UP) lastCellB = CellState.UP;
        else if (s == TiltState.AUDIENCE_UP) lastCellB = CellState.DOWN;
        else if (s == TiltState.TRANSITION) lastCellB = CellState.MOVING;
        else lastCellB = CellState.UNSEEN;
        return lastCellB;
    }

    private void log(String step) {
        stepLog.add(String.format("%-28s %5.1f s", step, matchTimer.seconds()));
        showLog();
    }

    private void showLog() {
        telemetry.addData("Match time", "%.1f s", matchTimer.seconds());
        telemetry.addData("Partner / tips seen", "%d / %d", partnerBalls, tipsSeen);
        telemetry.addData("Red HIVE", "%s (%.1f°)", lastCellB, lastTiltDeg);
        for (String s : stepLog) telemetry.addLine(s);
        telemetry.update();
    }
}
