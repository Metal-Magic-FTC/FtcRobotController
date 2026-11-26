package org.firstinspires.ftc.teamcode.biobuzz.comp1.Red2Tip;

public class RedV1_2Tip {



import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;

@Autonomous(name = "Solo 2 Tip (pollen only)", group = "BIOBUZZ")
public class Solo2TipAuto extends LinearOpMode {

    // =====================================================================================
    // TIMING AND SPEED
    // =====================================================================================
    private static final long   SHOOT_MS          = 2000; // one full volley of 4
    private static final long   GARDEN_DWELL_MS   = 250;  // extra intake time after the garden sweep
    private static final long   FLOWER_INTAKE_MS  = 3000; // pulling 4 out of a flower
    private static final double FAST              = 1.0;  // max power on drive legs
    private static final double SWEEP             = 0.45; // garden sweep
    private static final double CREEP             = 0.35; // last few inches into a flower
    private static final double SHORT_HOP         = 0.7;  // launch spot <-> far-wall flower
    private static final double CELL_B_EARLIEST_S = 6.5;  // tip 1 is finished turning by about here
    private static final double VOLLEY_3_LATEST_S = 26.3; // later than this, skip volley 3 so you still park
    private static final double LEG_TIMEOUT_S     = 6.0;  // safety: never get stuck on one path

    // =====================================================================================
    // CELL B CHECK (Limelight AprilTags)
    // Team 12303's mapping: red second cell = tags 30-33, blue second cell = tags 38-41. Verify on your field.
    // The camera has to see those tags from the launch spot (they are high, so it may need an upward tilt).
    // If it can't, set USE_CELL_B_CHECK = false and the auto runs on timing only.
    // =====================================================================================
    private static final boolean USE_CELL_B_CHECK  = true;
    private static final int     APRILTAG_PIPELINE = 3;
    private static final int[]   RED_CELL_B_TAGS   = {30, 31, 32, 33};
    private static final int[]   BLUE_CELL_B_TAGS  = {38, 39, 40, 41};
    private static final double  CELL_B_UP_MIN_TY  = 10.0; // tune: tag ty (deg) when B is up, seen from the launch spot
    private static final double  STILL_TY_DEG      = 0.5;  // ty change below this = not moving
    private static final int     STILL_READS       = 3;    // this many still reads in a row = settled

    // =====================================================================================
    // HARDWARE: EDIT THIS SECTION TO MATCH YOUR BIOBUZZ ROBOT
    // Names are from your DECODE autos. Change any that are different on the new robot.
    // =====================================================================================
    private DcMotorEx launchMotor;
    private DcMotor   intakeMotor;
    private Servo     hoodServo;
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
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
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
    private Solo2TipPaths paths;
    private boolean blue = false;
    private int[] cellBTags = RED_CELL_B_TAGS;
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final List<String> stepLog = new ArrayList<>();
    private enum CellState { UP, DOWN, MOVING, UNSEEN }

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        follower = Constants.createFollower(hardwareMap);
        hoodServo.setPosition(HOOD_A);

        // Pick the alliance during INIT.
        while (opModeInInit()) {
            if (gamepad1.x) blue = true;
            if (gamepad1.b) blue = false;
            telemetry.addLine("X = BLUE, B = RED");
            telemetry.addData("Alliance", blue ? "BLUE" : "RED");
            telemetry.addLine("Robot on the audience wall, facing your start cell (the one with your 3 nectar).");
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested()) return;

        paths = new Solo2TipPaths(blue);
        cellBTags = blue ? BLUE_CELL_B_TAGS : RED_CELL_B_TAGS;
        follower.setStartingPose(paths.start);
        matchTimer.reset();

        // 1. Tip 1: shoot preloads into cell A from the start spot.
        launcherForCellA();
        shootVolley();
        log("1 shot cell A (tip 1)");

        // 2. Garden. Leaving the wall = LEAVE.
        runPath(paths.toGarden(), FAST);
        intakeOn();
        runPath(paths.gardenSweep(), SWEEP);
        holdMs(GARDEN_DWELL_MS);
        intakeOff();
        log("2 garden loaded");

        // 3. Lane to launch spot B. Make sure tip 1 is done and B is up before shooting.
        launcherForCellB();
        runPath(paths.gardenToLaunchB(), FAST);
        holdUntil(CELL_B_EARLIEST_S);
        if (USE_CELL_B_CHECK && readCellB(2.0) == CellState.DOWN) {
            rescueTip1ThenPark(); // tip 1 never happened: put the garden pollen in cell A instead
            return;
        }
        shootVolley();
        log("3 shot B (4 in)");

        // 4. Alliance-wall flower.
        runPath(paths.launchBToRedWallFlower(), FAST);
        intakeOn();
        runPath(paths.redWallFlowerCreep(), CREEP);
        holdMs(FLOWER_INTAKE_MS);
        intakeOff();
        log("4 wall flower loaded");

        // 5. Back to B, shoot. 8 in B should tip it.
        runPath(paths.redWallFlowerToLaunchB(), FAST);
        if (USE_CELL_B_CHECK) readCellB(1.0); // lets the robot settle and waits out any wobble
        shootVolley();
        log("5 shot B (8 in)");

        // 6. Far-wall flower.
        intakeOn();
        runPath(paths.launchBToFarWallFlower(), SHORT_HOP);
        holdMs(FLOWER_INTAKE_MS);
        intakeOff();
        log("6 far flower loaded");

        // 7. Insurance volley. Skipped only if B already tipped (keep the pollen for cell A in teleop)
        //    or if it would cost you the park.
        runPath(paths.farWallFlowerToLaunchB(), SHORT_HOP);
        boolean fire = matchTimer.seconds() <= VOLLEY_3_LATEST_S;
        if (fire && USE_CELL_B_CHECK) {
            CellState b = readCellB(1.0); // holds fire while B is still turning
            if (b == CellState.DOWN) fire = false;
        }
        if (fire && matchTimer.seconds() <= VOLLEY_3_LATEST_S + 0.5) {
            shootVolley();
            log("7 shot B (12 in)");
        } else {
            log("7 skipped volley 3");
        }

        // 8. Park.
        runPath(paths.parkPath(), FAST);
        log("8 parked");

        while (opModeIsActive()) { follower.update(); showLog(); }
    }

    /** Tip 1 failed: go back to cell A with the garden pollen, shoot, then park. */
    private void rescueTip1ThenPark() {
        log("3 B still down: rescue tip 1");
        launcherForCellA();
        runPath(paths.rescueToStart(), FAST);
        shootVolley();
        log("R shot cell A");
        runPath(paths.rescueToPark(), FAST);
        log("8 parked");
        while (opModeIsActive()) { follower.update(); showLog(); }
    }

    // =====================================================================================
    // HELPERS
    // =====================================================================================

    /** Follows a path to the end and keeps holding that position afterward. */
    private void runPath(Path path, double maxPower) {
        follower.setMaxPower(maxPower);
        follower.followPath(path, true);
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && follower.isBusy() && t.seconds() < LEG_TIMEOUT_S) {
            follower.update();
            showLog();
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
        while (opModeIsActive() && t.milliseconds() < ms) follower.update();
    }

    private void holdUntil(double matchSeconds) {
        while (opModeIsActive() && matchTimer.seconds() < matchSeconds) follower.update();
    }

    /** Watches cell B's tags until they stop moving. UP/DOWN once settled, MOVING if never settled, UNSEEN if not visible. */
    private CellState readCellB(double maxWaitS) {
        ElapsedTime t = new ElapsedTime();
        Double lastTy = null;
        int still = 0;
        boolean seen = false;
        while (opModeIsActive() && t.seconds() < maxWaitS) {
            follower.update();
            Double ty = cellBTy();
            if (ty != null) {
                seen = true;
                still = (lastTy != null && Math.abs(ty - lastTy) < STILL_TY_DEG) ? still + 1 : 0;
                lastTy = ty;
                if (still >= STILL_READS) return ty >= CELL_B_UP_MIN_TY ? CellState.UP : CellState.DOWN;
            }
            sleep(40);
        }
        return seen ? CellState.MOVING : CellState.UNSEEN;
    }

    /** Average vertical angle of the visible cell B tags, or null if none are visible. */
    private Double cellBTy() {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return null;
        double sum = 0;
        int n = 0;
        for (LLResultTypes.FiducialResult f : r.getFiducialResults()) {
            for (int id : cellBTags) {
                if (f.getFiducialId() == id) { sum += f.getTargetYDegrees(); n++; }
            }
        }
        return n == 0 ? null : sum / n;
    }

    private void log(String step) {
        stepLog.add(String.format("%-26s %5.1f s", step, matchTimer.seconds()));
        showLog();
    }

    private void showLog() {
        telemetry.addData("Alliance", blue ? "BLUE" : "RED");
        telemetry.addData("Match time", "%.1f s", matchTimer.seconds());
        for (String s : stepLog) telemetry.addLine(s);
        telemetry.update();
    }
}

