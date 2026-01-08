package org.firstinspires.ftc.teamcode.decode.pedroPathing.autov2.blueback;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.teleOp.CustomMecanumDrive;

import java.util.List;

@Autonomous(name = "!! Blue Close FULL AUTO", group = "Auto")
public class BlueBack extends LinearOpMode {

    /* =============================
     * HARDWARE
     * ============================= */
    private Follower follower;
    private GeneratedPathsBlueBack paths;
    private CustomMecanumDrive drivetrain;

    private DcMotor intakeMotor, launchMotor, spinMotor;
    private Servo hoodServo, flickServo;
    private NormalizedColorSensor intakeColor, intakeColor2;
    private Limelight3A limelight;

    /* =============================
     * SPINDEXER V2
     * ============================= */
    private enum Ball { EMPTY, PURPLE, GREEN }
    private final Ball[] slots = {Ball.EMPTY, Ball.EMPTY, Ball.EMPTY};

    private static final int[] INTAKE_POS  = {125, 375, 625};
    private static final int[] OUTTAKE_POS = {500, 0, 250};

    private int index = 0;
    private boolean intakeActive = false;
    private boolean waitingForBall = false;
    private boolean waitingToRotate = false;

    private long colorDetectedTime = 0;
    private static final long COLOR_DELAY_MS = 500;
    private int nextIndexAfterDelay = -1;

    private final double spinMotorSpeed = 0.35;
    private final float colorGain = 20;

    /* =============================
     * PATTERNS
     * ============================= */
    private final Ball[] pattern21 = {Ball.GREEN, Ball.PURPLE, Ball.PURPLE};
    private final Ball[] pattern22 = {Ball.PURPLE, Ball.GREEN, Ball.PURPLE};
    private final Ball[] pattern23 = {Ball.PURPLE, Ball.PURPLE, Ball.GREEN};

    @Override
    public void runOpMode() {

        initHardware();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(GeneratedPathsBlueBack.START_POSE);
        paths = new GeneratedPathsBlueBack(follower);

        waitForStart();
        if (isStopRequested()) return;

        hoodServo.setPosition(0.57);
        flickServo.setPosition(0.9);

        /* =============================
         * SCAN + TAG
         * ============================= */
        runPath(paths.scan(), 50, 1.0);
        Ball[] pattern = getPatternFromTag();

        /* =============================
         * PRELOAD SHOOT
         * ============================= */
        runPath(paths.shoot(), 50, 1.0);
        shoot(pattern);

        /* =============================
         * INTAKE 1–3
         * ============================= */
        intakeActive = true;
        runPath(paths.toIntake1(), 50, 0.75);
        rotateToIndex(0);

        runPath(paths.intakeball3(), 50, 0.3);
        intakeActive = false;

        runPath(paths.shoot2(), 50, 1.0);
        shoot(pattern);

        /* =============================
         * INTAKE 4–6
         * ============================= */
        intakeActive = true;
        runPath(paths.toIntake2(), 50, 0.75);
        runPath(paths.intakeball6(),50,0.3);
        intakeActive = false;

        runPath(paths.shoot3(), 50, 1.0);
        shoot(pattern);

        /* =============================
         * INTAKE 7–9
         * ============================= */
        intakeActive = true;
        runPath(paths.toIntake3(),50,.75);
        runPath(paths.intakeball9(),50,.75);
        intakeActive = false;

        runPath(paths.shoot4(), 50, 1.0);
        shoot(new Ball[]{Ball.PURPLE, Ball.GREEN, Ball.PURPLE});
    }

    /* =============================
     * INTAKE STATE MACHINE
     * ============================= */
    private void runIntakePath(PathChain path) {

        follower.followPath(path);
        intakeMotor.setPower(-0.85);

        waitingForBall = true;
        waitingToRotate = false;

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            if (intakeActive && waitingForBall && !waitingToRotate && !spinMotor.isBusy()) {
                Ball detected = detectColor(intakeColor, intakeColor2);

                if (detected != Ball.EMPTY) {
                    slots[index] = detected;
                    waitingForBall = false;
                    waitingToRotate = true;
                    colorDetectedTime = System.currentTimeMillis();
                    nextIndexAfterDelay = findNextEmpty();
                }
            }

            if (waitingToRotate &&
                    System.currentTimeMillis() - colorDetectedTime >= COLOR_DELAY_MS) {

                if (nextIndexAfterDelay != -1) {
                    rotateToIndex(nextIndexAfterDelay);
                    waitingForBall = true;
                } else {
                    rotateToIndex(0);
                    intakeActive = false;
                }
                waitingToRotate = false;
            }
        }

        intakeMotor.setPower(0);
    }

    /* =============================
     * SHOOTING
     * ============================= */
    private void shoot(Ball[] order) {
        launchMotor.setPower(0.9);
        sleep(250);

        for (Ball b : order) {
            int idx = findClosest(b);
            if (idx == -1) continue;

            rotateToIndex(idx);
            while (opModeIsActive() && spinMotor.isBusy()) idle();

            flickServo.setPosition(0.75);
            sleep(450);
            flickServo.setPosition(0.9);

            slots[idx] = Ball.EMPTY;
        }
        launchMotor.setPower(0);
    }

    /* =============================
     * SPINDEXER CORE
     * ============================= */
    private void rotateToIndex(int target) {
        index = target;
        int base = intakeActive ? INTAKE_POS[target] : OUTTAKE_POS[target];
        int pos = closestModular(base, spinMotor.getCurrentPosition());

        spinMotor.setTargetPosition(pos);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(spinMotorSpeed);
    }

    private int closestModular(int mod, int current) {
        int best = mod, min = Integer.MAX_VALUE;
        for (int k = -2; k <= 2; k++) {
            int c = mod + 750 * k;
            int d = Math.abs(c - current);
            if (d < min) { min = d; best = c; }
        }
        return best;
    }

    private int findNextEmpty() {
        for (int i = 0; i < 3; i++) {
            int idx = (index + i) % 3;
            if (slots[idx] == Ball.EMPTY) return idx;
        }
        return -1;
    }

    private int findClosest(Ball target) {
        for (int i = 0; i < 3; i++) {
            int idx = (index + i) % 3;
            if (slots[idx] == target) return idx;
        }
        return -1;
    }

    /* =============================
     * COLOR + TAG
     * ============================= */
    private Ball detectColor(NormalizedColorSensor s1, NormalizedColorSensor s2) {
        Ball b1 = detectSingle(s1);
        Ball b2 = detectSingle(s2);
        if (b1 == Ball.PURPLE || b2 == Ball.PURPLE) return Ball.PURPLE;
        if (b1 == Ball.GREEN || b2 == Ball.GREEN) return Ball.GREEN;
        return Ball.EMPTY;
    }

    private Ball detectSingle(NormalizedColorSensor s) {
        NormalizedRGBA c = s.getNormalizedColors();
        float r = c.red, g = c.green, b = c.blue;
        if (r + g + b < 0.07f) return Ball.EMPTY;
        if (b > r * 1.35f && b > g * 1.25f && b > 0.12f) return Ball.PURPLE;
        if (g > r * 1.15f && g > b * 1.15f && g > 0.15f) return Ball.GREEN;
        return Ball.EMPTY;
    }

    private Ball[] getPatternFromTag() {
        int id = detectAprilTag(1500);
        if (id == 21) return pattern21;
        if (id == 23) return pattern23;
        return pattern22;
    }

    private int detectAprilTag(long timeout) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < timeout) {
            LLResult r = limelight.getLatestResult();
            if (r != null && r.isValid() && !r.getFiducialResults().isEmpty())
                return r.getFiducialResults().get(0).getFiducialId();
            sleep(15);
        }
        return 22;
    }

    /* =============================
     * PATH HELPERS
     * ============================= */
    private void runPath(PathChain path, int delay, double speed) {
        follower.setMaxPower(speed);
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) follower.update();
        follower.breakFollowing();
        if (delay > 0) sleep(delay);
    }

    /* =============================
     * INIT
     * ============================= */
    private void initHardware() {

        drivetrain = new CustomMecanumDrive(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");
        spinMotor   = hardwareMap.get(DcMotor.class, "spinMotor");

        hoodServo = hardwareMap.servo.get("hoodServo");
        flickServo = hardwareMap.servo.get("flickServo");

        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeColor  = hardwareMap.get(NormalizedColorSensor.class, "intakeColor");
        intakeColor2 = hardwareMap.get(NormalizedColorSensor.class, "intakeColor2");
        intakeColor.setGain(colorGain);
        intakeColor2.setGain(colorGain);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);
        limelight.start();
    }
}
