package org.firstinspires.ftc.teamcode.biobuzz.comp1.Red2Tip;

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

import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.Hive;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.HiveTilt;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.TiltState;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name = "Red V1 2 Tip", group = "BIOBUZZ")
public class RedV1_2Tip extends LinearOpMode {

    // =====================================================================================
    // TIMING AND SPEED
    // =====================================================================================
    private static final long   SHOOT_MS          = 1800; // one full volley of 4 (was 2000)
    private static final long   GARDEN_DWELL_MS   = 200;  // extra intake time after the garden sweep (was 250)
    private static final long   FLOWER_INTAKE_MS  = 2700; // pulling 4 out of a flower (was 3000)
    private static final double CELL_B_EARLIEST_S = 6.5;  // tip 1 is finished turning by about here
    private static final double VOLLEY_3_LATEST_S = 26.0; // later than this, skip the last shot (it would get cut off by PARK_AT_S)
    private static final double LEG_TIMEOUT_S     = 6.0;  // safety: never get stuck on one path
    private static final double PARK_AT_S = 27.5;


    private static final boolean USE_CELL_B_CHECK = true;
    private static final int     STILL_READS      = 3; // same stable tilt state this many reads in a row = settled
    private static final double  TIP_CHECK_WAIT_S = 1.7; // after the third set: max time to wait for the HIVE to settle (was 2.0)
    private static final double  TIP1_CHECK_WAIT_S = 1.7; // at launch B before the first B volley (was 2.0)
    private static final double  SETTLE_WAIT_S     = 0.8; // before the third set (was 1.0)

    private DcMotorEx launchMotor;
    private DcMotor intakeMotor;
    private Servo hoodServo;
    private Limelight3A limelight;

    private static final double LAUNCH_VEL_A = 1800; // tune: 43 in shot into cell A
    private static final double LAUNCH_VEL_B = 1600; // tune: 31 in shot into cell B
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

    private void launcherForCellA() { launchMotor.setVelocity(LAUNCH_VEL_A); hoodServo.setPosition(HOOD_A); }
    private void launcherForCellB() { launchMotor.setVelocity(LAUNCH_VEL_B); hoodServo.setPosition(HOOD_B); }
    private void intakeOn()         { intakeMotor.setPower(INTAKE_POWER); }
    private void intakeOff()        { intakeMotor.setPower(0); }

    /** TODO: replace with your BIOBUZZ feed code. It must empty all 4 pollen in about SHOOT_MS. */
    private void feedStart() { intakeMotor.setPower(FEED_POWER); }
    private void feedStop()  { intakeMotor.setPower(0); }

    // =====================================================================================
    // RUN
    // =====================================================================================
    private Follower follower;
    private Red2TipPaths paths;
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final List<String> stepLog = new ArrayList<>();
    private enum CellState { UP, DOWN, MOVING, UNSEEN }
    private CellState lastCellB = CellState.UNSEEN;
    private double lastTiltDeg = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        follower = Constants.create(hardwareMap);
        paths = new Red2TipPaths(); // built during init so start is instant
        paths.setStartPose(follower);
        hoodServo.setPosition(HOOD_A);


        while (opModeInInit()) {
            follower.update();
            readHiveOnce();
            telemetry.addLine("RED. Robot on the audience wall, facing your start cell (the one with your 3 nectar).");
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
            parkNow(); // PARK_AT_S hit in the middle of something: drop it and park
        }
        finish();
    }

    /** The whole auto. Any wait/path loop throws ParkNow once the clock reaches PARK_AT_S. */
    private void runAuto() {
        // 1. Tip 1: shoot preloads into cell A from the start spot.
        launcherForCellA();
        shootVolley();
        log("1 shot cell A (tip 1)");

        // 2. Garden. Leaving the wall = LEAVE.
        runPath(paths.toGarden(), Red2TipPaths.FAST);
        intakeOn();
        runPath(paths.gardenSweep(), Red2TipPaths.SWEEP);
        holdMs(GARDEN_DWELL_MS);
        intakeOff();
        log("2 garden loaded");

        // 3. Lane to launch spot B. Make sure tip 1 is done and B is up before shooting.
        launcherForCellB();
        runPath(paths.gardenToLaunchB(), Red2TipPaths.FAST);
        holdUntil(CELL_B_EARLIEST_S);
        if (USE_CELL_B_CHECK && readCellB(TIP1_CHECK_WAIT_S) == CellState.DOWN) {
            rescueTip1ThenPark(); // tip 1 never happened: put the garden pollen in cell A instead
            return;
        }
        shootVolley();
        log("3 shot B (4 in)");

        // 4. Alliance-wall flower.
        runPath(paths.launchBToRedWallFlower(), Red2TipPaths.FAST);
        intakeOn();
        runPath(paths.redWallFlowerCreep(), Red2TipPaths.CREEP);
        holdMs(FLOWER_INTAKE_MS);
        intakeOff();
        log("4 wall flower loaded");

        // 5. Back to B, shoot. 8 in B should tip it.
        runPath(paths.redWallFlowerToLaunchB(), Red2TipPaths.FAST);
        if (USE_CELL_B_CHECK) readCellB(SETTLE_WAIT_S); // lets the robot settle and waits out any wobble
        shootVolley();
        log("5 shot B (8 in)");

        // 5b. Did the HIVE tip after the third set? B DOWN = tip 2 done.
        //     Anything else (still UP, still swinging, not seen) counts as NOT tipped, so we go back and shoot.
        boolean tipped = USE_CELL_B_CHECK && readCellB(TIP_CHECK_WAIT_S) == CellState.DOWN;
        log(tipped ? "5 HIVE tipped" : "5 HIVE not tipped");

        // 6. Far-wall flower (always intake the last set).
        intakeOn();
        runPath(paths.launchBToFarWallFlower(), Red2TipPaths.SHORT_HOP);
        holdMs(FLOWER_INTAKE_MS);
        intakeOff();
        log("6 far flower loaded");

        if (tipped) {
            // 7. Already tipped: keep the pollen and go straight to park.
            startParking();
            runPath(paths.farWallFlowerToPark(), Red2TipPaths.FAST);
            log("8 parked (kept pollen)");
        } else {
            // 7. Not tipped: back to B and shoot, unless it would cost the park.
            runPath(paths.farWallFlowerToLaunchB(), Red2TipPaths.SHORT_HOP);
            if (matchTimer.seconds() <= VOLLEY_3_LATEST_S) {
                shootVolley();
                log("7 shot B (12 in)");
            } else {
                log("7 out of time, skipped shot");
            }
            // 8. Park.
            startParking();
            runPath(paths.parkPath(), Red2TipPaths.FAST);
            log("8 parked");
        }
    }

    /** Tip 1 failed: go back to cell A with the garden pollen, shoot, then park. */
    private void rescueTip1ThenPark() {
        log("3 B still down: rescue tip 1");
        launcherForCellA();
        runPath(paths.rescueToStart(), Red2TipPaths.FAST);
        shootVolley();
        log("R shot cell A");
        startParking();
        runPath(paths.rescueToPark(), Red2TipPaths.FAST);
        log("8 parked");
    }

    // =====================================================================================
    // PARK GUARD
    // =====================================================================================
    private static class ParkNow extends RuntimeException {}

    /** true once the robot is on its way to park; the guard stops interrupting. */
    private boolean parking = false;

    /** Called inside every wait/path loop. */
    private void checkParkClock() {
        if (!parking && matchTimer.seconds() >= PARK_AT_S) throw new ParkNow();
    }

    private void startParking() {
        parking = true;
        feedStop();
        intakeOff();
        launchMotor.setVelocity(0);
    }

    /** Out of time: straight to park from wherever the robot is. */
    private void parkNow() {
        startParking();
        log(String.format("! %.1f s: park now", PARK_AT_S));
        runPath(paths.parkFromHere(follower), Red2TipPaths.FAST);
        log("8 parked");
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

    /** Robot is stopped while shooting (it can't move and shoot). Holds position while feeding. */
    private void shootVolley() {
        feedStart();
        holdMs(SHOOT_MS);
        feedStop();
    }

    private void holdMs(long ms) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.milliseconds() < ms) { checkParkClock(); follower.update(); }
    }

    private void holdUntil(double matchSeconds) {
        while (opModeIsActive() && matchTimer.seconds() < matchSeconds) { checkParkClock(); follower.update(); }
    }

    /**
     * Watches the red HIVE until its tilt settles in a stable position.
     * UP/DOWN (cell B) once settled, MOVING if it never settled, UNSEEN if no red tags were visible.
     */
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
        stepLog.add(String.format("%-26s %5.1f s", step, matchTimer.seconds()));
        showLog();
    }

    private void showLog() {
        telemetry.addData("Match time", "%.1f s", matchTimer.seconds());
        telemetry.addData("Red HIVE", "%s (%.1f°)", lastCellB, lastTiltDeg);
        for (String s : stepLog) telemetry.addLine(s);
        telemetry.update();
    }
}
