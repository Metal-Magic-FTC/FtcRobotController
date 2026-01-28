package org.firstinspires.ftc.teamcode.decode.pedroPathing.optStatesAutos;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.teleOp.tests.CustomMecanumDrive;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "!!Red Close V5 Qualy 2")
public class Close12Auto extends LinearOpMode {

    private int index = 0;

    private float gain = 20;

    private enum Ball { EMPTY, PURPLE, GREEN }
    private Ball[] slots = {Ball.EMPTY, Ball.EMPTY, Ball.EMPTY};

    // ---------------- DRIVE ----------------
    private Follower f;
    private Close12Path p;
    private CustomMecanumDrive drivetrain;
    private Limelight3A limelight;

    // ---------------- INTAKE, TRANSFER, SCORING ----------------

    private DcMotor spinMotor;
    private DcMotorEx launchMotor;
    private DcMotor intakeMotor;
    Servo hoodServo;
    Servo flickServo;
    NormalizedColorSensor intakeColor;
    NormalizedColorSensor intakeColor2;

    private static final int[] OUTTAKE_POS = {504, 2, 252};
    private static final int[] INTAKE_POS  = {125, 375, 625};

    private double spinMotorSpeed = 0.35;

    private boolean intakeActive = false;
    private boolean waitingToRotate = false;
    private boolean waitingForBall = false;
    private long colorDetectedTime = 0;
    private static final long COLOR_DELAY_MS = 50; // 100 ms delay before spinning
    private int nextIndexAfterDelay = -1;

    private static final int SPIN_TOLERANCE_TICKS = 5;
    private static final long SPIN_TIMEOUT_MS = 10000;

    private int lastSpinTarget = 0;

    private double flickUp = 0.75;
    private double flickDown = 1;

    // ---------------- RUN ----------------
    @Override
    public void runOpMode() throws InterruptedException {


        initHardware();
        resetSlots();

        slots[0] = Ball.PURPLE;
        slots[1] = Ball.GREEN;
        slots[2] = Ball.PURPLE;

        f = Constants.createFollower(hardwareMap);
        p = new Close12Path(f);
        f.setPose(p.start);
        hoodServo.setPosition(0.77);


        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        launchMotor.setPower(0.91); // 0.935

        flickServo.setPosition(flickDown);

        // scan balls
        //scanBallsInSlots(5000);

//        runPath(p.scan(), 0, 1.0);
//        telemetry.addData("X SCAN", f.getPose().getX());
//        telemetry.addData("Y SCAN", f.getPose().getY());

        Ball[] pattern = getPatternFromTag();

        aimClosest(pattern[0]);

        runPath(p.scoreP(), 0, 1);
        telemetry.addData("X SHOOT", f.getPose().getX());
        telemetry.addData("Y SHOOT", f.getPose().getY());

        // ---- SHOOT ----
        shoot(pattern);

        intakeMotor.setPower(-0.8);
        intakeActive = true;
        rotateToIndex(0);
        resetSlots();

        // ---- INTAKE 1–3 ----

        double startTime;

        intakeActive = true;
        rotateToIndex(0);
//        runPath(p.intake1(), 0, 1);
        telemetry.addData("X intake1", f.getPose().getX());
        telemetry.addData("Y intake2", f.getPose().getY());
        resetSlots();

        runPathWithIntake(p.intake1(), 0, 0.3);
//        startTime = System.currentTimeMillis();
//        //slots[0] = Ball.PURPLE;
//        while (System.currentTimeMillis() < startTime + 650) {
//            waitingForBall = true;
//            intakeActive = true;
//            intake();
//        }
//
//        runPathWithIntake(paths.intakeball2(), 0, 0.3);
//        startTime = System.currentTimeMillis();
//        //slots[0] = Ball.PURPLE;
//        while (System.currentTimeMillis() < startTime + 650) {
//            waitingForBall = true;
//            intakeActive = true;
//            intake();
//        }
//
//        runPathWithIntake(paths.intakeball3(), 0, 0.3);
//        startTime = System.currentTimeMillis();
//        //slots[0] = Ball.GREEN;
//        while (System.currentTimeMillis() < startTime + 250) {
//            waitingForBall = true;
//            intakeActive = true;
//            intake();
//        }

        intakeMotor.setPower(-0.8);
        slots[0] = Ball.PURPLE;
        slots[1] = Ball.PURPLE;
        slots[2] = Ball.GREEN;

        aimClosest(pattern[0]);

        slots[0] = Ball.PURPLE;
        slots[1] = Ball.PURPLE;
        slots[2] = Ball.GREEN;

        intakeMotor.setPower(0.6);
        runPath(p.score1(), 5, 1);
        intakeMotor.setPower(-0.8);

        // ---- SHOOT ----
        shoot(pattern);

//        intakeMotor.setPower(-0.6);
//        intakeActive = true;
        rotateToIndex(0);
        resetSlots();

        // ---- INTAKE 4–6 ----
//        intakeActive = false;
//        rotateToIndex(0);
//        runPath(paths.toIntake2(), 50, 1);

        intakeActive = true;
        rotateToIndex(0);
//        runPath(p.toIntake2(), 0, 1);
//        telemetry.addData("X intake1", f.getPose().getX());
//        telemetry.addData("Y intake2", f.getPose().getY());
        resetSlots();

        runPathWithIntake(p.intake2(), 0, 0.3);

        runPath(p.gate(), 5, 1);
//        startTime = System.currentTimeMillis();
//        //slots[0] = Ball.PURPLE;
//        while (System.currentTimeMillis() < startTime + 650) {
//            waitingForBall = true;
//            intakeActive = true;
//            intake();
//        }
//
//        runPathWithIntake(paths.intakeball5(), 0, 0.3);
//        startTime = System.currentTimeMillis();
//        //slots[0] = Ball.GREEN;
//        while (System.currentTimeMillis() < startTime + 650) {
//            waitingForBall = true;
//            intakeActive = true;
//            intake();
//        }
//
//        runPathWithIntake(paths.intakeball6(), 0, 0.3);
//        startTime = System.currentTimeMillis();
//        //slots[0] = Ball.PURPLE;
//        while (System.currentTimeMillis() < startTime + 250) {
//            waitingForBall = true;
//            intakeActive = true;
//            intake();
//        }

        intakeMotor.setPower(-0.8);
        slots[0] = Ball.PURPLE;
        slots[1] = Ball.GREEN;
        slots[2] = Ball.PURPLE;

        aimClosest(pattern[0]);

        slots[0] = Ball.PURPLE;
        slots[1] = Ball.GREEN;
        slots[2] = Ball.PURPLE;

        intakeMotor.setPower(0.6);
        runPath(p.score2(), 5, 1);
        intakeMotor.setPower(-0.8);

        // ---- SHOOT ----
        shoot(pattern);

        intakeMotor.setPower(-0.8);
        intakeActive = true;
        rotateToIndex(0);
        resetSlots();

        // ---- INTAKE 1–3 ----
        
        intakeActive = true;
        rotateToIndex(0);
//        runPath(p.intake1(), 0, 1);
        telemetry.addData("X intake1", f.getPose().getX());
        telemetry.addData("Y intake2", f.getPose().getY());
        resetSlots();

        runPathWithIntake(p.intake1(), 0, 0.3);

        intakeMotor.setPower(-0.8);
        slots[0] = Ball.PURPLE;
        slots[1] = Ball.PURPLE;
        slots[2] = Ball.GREEN;

        aimClosest(pattern[0]);

        slots[0] = Ball.PURPLE;
        slots[1] = Ball.PURPLE;
        slots[2] = Ball.GREEN;

        intakeMotor.setPower(0.6);
        runPath(p.score1(), 5, 1);
        intakeMotor.setPower(-0.8);

        // ---- SHOOT ----
        shoot(pattern);

        telemetry.addLine("Finished");
        telemetry.update();
    }

    // ---------------- SPINDEXER ----------------

    private void rotateToIndex(int target) {
        index = target;
        int base = intakeActive ? INTAKE_POS[target] : OUTTAKE_POS[target];
        int targetPos = closestModular(base, spinMotor.getCurrentPosition());

        lastSpinTarget = targetPos; // store target

        spinMotor.setTargetPosition(targetPos);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(spinMotorSpeed);
    }

    private int closestModular(int mod, int current) {
        int best = mod;
        int minDiff = Integer.MAX_VALUE;
        for (int k = -2; k <= 2; k++) {
            int candidate = mod + 750 * k;
            int diff = Math.abs(candidate - current);
            if (diff < minDiff) {
                minDiff = diff;
                best = candidate;
            }
        }
        return best;
    }

    private void aimClosest(Ball target) {
        intakeActive = false;
        for (int i = 0; i < 3; i++) {
            int idx = (index + i) % 3;
            if (slots[idx] == target) {
                rotateToIndex(idx);
                return;
            }
        }
    }

    private int findNextEmpty() {
        for (int i = 0; i < 3; i++) {
            int idx = (index + i) % 3;
            if (slots[idx] == Ball.EMPTY) return idx;
        }
        return -1;
    }

    private void shootOne(Ball target) {

        List<Ball> targetAL = Arrays.asList(slots);
        if (!targetAL.contains(target)) return; // check if has a ball

        aimClosest(target);
        waitForSpindexer();
        sleep(350); // 400

        flickServo.setPosition(flickUp);
        sleep(400); // 500
        flickServo.setPosition(flickDown);
        sleep(250); // 300

        if (slots[index] == target) {
            slots[index] = Ball.EMPTY;
        }

    }

    private void shoot(Ball[] pattern) {

        intakeActive = false;
        intakeMotor.setPower(0);

        for (Ball ball : pattern) {
            //shootOne(ball);
            if (ball != Ball.EMPTY) {
                shootOne(ball);
            }
        }

        intakeMotor.setPower(-0.8);

    }

    private void intake() {

        // spindexer logic (COLOR-BASED DETECTION) with delay
        if (waitingForBall && intakeActive && !spinMotor.isBusy() && !waitingToRotate) {
            Ball detected = detectColor(intakeColor, intakeColor2);

            if (detected != Ball.EMPTY) {
                slots[index] = detected;
                waitingForBall = false;

                // compute next index, but don't rotate yet
                int nextEmpty = findNextEmpty();
                nextIndexAfterDelay = nextEmpty;
                colorDetectedTime = System.currentTimeMillis();
                waitingToRotate = true;
                intakeMotor.setPower(0);
            }
        }

        // after COLOR_DELAY_MS, rotate if needed
        if (waitingToRotate) {
            if (System.currentTimeMillis() - colorDetectedTime >= COLOR_DELAY_MS) {
//                int nextEmpty = findNextEmpty();
//                nextIndexAfterDelay = nextEmpty;
                if (nextIndexAfterDelay != -1) {
                    rotateToIndex(nextIndexAfterDelay);
                    waitingForBall = true; // continue intake
                    intakeMotor.setPower(-0.6);
                } else {
//                    intakeActive = false;
//                    rotateToIndex(0); // back to home
                    // Stay in intake position, keep waiting
                    waitingForBall = true;
                }
                waitingToRotate = false; // reset
            }
        }
    }

    private void scanBallsInSlots(long timeoutMs) {

        long start = System.currentTimeMillis();

        int nextEmpty = findNextEmpty();

        if (nextEmpty != -1) {
            // normal intake
            intakeActive = true;
            waitingForBall = true;
            rotateToIndex(nextEmpty);
        } else {
            // all full then go shoot
            intakeActive = false;
            waitingForBall = false;

            int nextLoaded = findClosestLoaded();
            if (nextLoaded != -1) {
                rotateToIndex(nextLoaded);
            }
        }

        intakeActive = true;
        while (opModeIsActive()
                && System.currentTimeMillis() - start < timeoutMs
                && Arrays.asList(slots).contains(Ball.EMPTY)) {
            intake();
        }

        int purpleCount = 0;
        int greenCount = 0;
        int emptyIndex = -1;

        // 1. Scan the array to see what we have
        for (int i = 0; i < slots.length; i++) {
            if (slots[i] == Ball.PURPLE) purpleCount++;
            else if (slots[i] == Ball.GREEN) greenCount++;
            else if (slots[i] == Ball.EMPTY) emptyIndex = i;
        }

        // 2. If we found an empty slot, fill it based on what's missing
        if (emptyIndex != -1) {
            if (purpleCount < 2) {
                slots[emptyIndex] = Ball.PURPLE;
            } else if (greenCount < 1) {
                slots[emptyIndex] = Ball.GREEN;
            }
        }

        intakeActive = false;
        waitingForBall = false;

        rotateToIndex(0);

    }

    private int findClosestLoaded() {
        for (int i = 0; i < 3; i++) {
            int idx = (index + i) % 3;
            if (slots[idx] != Ball.EMPTY) return idx;
        }
        return -1;
    }

    // ---------------- PATH HELPERS ----------------
    private void runPath(PathChain path, int stopDelay, double speed) {
        f.setMaxPower(speed);
        f.followPath(path);
        while (opModeIsActive() && f.isBusy()) f.update();
        f.breakFollowing();
        if (stopDelay > 0) sleep(stopDelay);
    }

    private void runPathWithIntake(PathChain path, int stopDelay, double speed) {

//        int nextEmpty = findNextEmpty();
//
//        if (nextEmpty != -1) {
//            // normal intake
//            intakeActive = true;
//            waitingForBall = true;
//            rotateToIndex(nextEmpty);
//        } else {
//            // all full then go shoot
//            intakeActive = false;
//            waitingForBall = false;
//
//            int nextLoaded = findClosestLoaded();
//            if (nextLoaded != -1) {
//                rotateToIndex(nextLoaded);
//            }
//        }

        launchMotor.setPower(0);
        intakeActive = true;
        waitingForBall = true;

        f.setMaxPower(speed);
        f.followPath(path);
        while (opModeIsActive() && f.isBusy()) {
            f.update();

            waitingForBall = true;
            intakeActive = true;
            intake();

        }
        f.breakFollowing();
        if (stopDelay > 0) sleep(stopDelay);

        intakeActive = false;
        waitingForBall = false;

        launchMotor.setPower(0.91); // 0.935

    }

    private void waitForSpindexer() {
        long start = System.currentTimeMillis();

        while (opModeIsActive()) {
            int error = Math.abs(spinMotor.getCurrentPosition() - lastSpinTarget);

            if (error <= SPIN_TOLERANCE_TICKS) {
                break;
            }

            if (System.currentTimeMillis() - start > SPIN_TIMEOUT_MS) {
                break; // safety exit
            }

            sleep(5); // yield to system
        }
    }

    // ---------------- APRILTAG AND COLOR SENSORS ----------------
    private Ball[] getPatternFromTag() {
        int id = detectAprilTag(2000);
//        telemetry.addData("id", id);
//        telemetry.update();
        if (id == 21) return new Ball[]{Ball.GREEN, Ball.PURPLE, Ball.PURPLE};
        if (id == 23) return new Ball[]{Ball.PURPLE, Ball.PURPLE, Ball.GREEN};
        return new Ball[]{Ball.PURPLE, Ball.GREEN, Ball.PURPLE};
    }

    private int detectAprilTag(long timeoutMs) {
        long start = System.currentTimeMillis();
        //while (opModeIsActive() && System.currentTimeMillis() - start < timeoutMs) {
        int count = 0;
        while (System.currentTimeMillis() - start < timeoutMs) {

            LLResult r = limelight.getLatestResult();
            if (r == null) {
                count++;
            }
            if (r != null && r.isValid() && !r.getFiducialResults().isEmpty()) {
                int id = r.getFiducialResults().get(0).getFiducialId();
                if (21 <= id && id <= 23) {
//                telemetry.addData("count", count);
//                telemetry.update();
                    return id;
                }
            }
            sleep(15);
        }
//        telemetry.addData("count", count);
//        telemetry.update();
        return 22;
    }

    private void enableLight(NormalizedColorSensor s) {
        if (s instanceof SwitchableLight) {
            ((SwitchableLight) s).enableLight(true);
        }
    }

    private Ball detectColor(NormalizedColorSensor sensor1, NormalizedColorSensor sensor2) {
        Ball ball1 = detectSingleSensor(sensor1);
        Ball ball2 = detectSingleSensor(sensor2);

        // Prioritize detected balls
        if (ball1 == Ball.PURPLE || ball2 == Ball.PURPLE) return Ball.PURPLE;
        if (ball1 == Ball.GREEN  || ball2 == Ball.GREEN)  return Ball.GREEN;

        return Ball.EMPTY;
    }

    // helper function for a single sensor
    private Ball detectSingleSensor(NormalizedColorSensor sensor) {
        NormalizedRGBA c = sensor.getNormalizedColors();
        float r = c.red, g = c.green, b = c.blue;

        // reject far / floor
        float total = r + g + b;
        if (total < 0.07f) return Ball.EMPTY;

        // PURPLE: blue-dominant (keep strict)
        if (b > r * 1.35f && b > g * 1.25f && b > 0.12f) {
            return Ball.PURPLE;
        }

        // GREEN: looser dominance + absolute floor
        if (g > r * 1.15f && g > b * 1.15f && g > 0.15f) {
            return Ball.GREEN;
        }

        return Ball.EMPTY;
    }

    // ---------------- INIT ----------------
    private void initHardware() {

        drivetrain = new CustomMecanumDrive(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);  // APRILTAG PIPELINE
        limelight.start();
//        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(3);

        spinMotor = hardwareMap.get(DcMotor.class, "spinMotor");
        intakeColor = hardwareMap.get(NormalizedColorSensor.class, "intakeColor");
        intakeColor2 = hardwareMap.get(NormalizedColorSensor.class, "intakeColor2");

        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeColor.setGain(gain);
        enableLight(intakeColor);

        intakeColor2.setGain(gain);
        enableLight(intakeColor2);

        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        launchMotor.setDirection(DcMotorEx.Direction.REVERSE); // same as TeleOp_Flick_Launch
        launchMotor.setPower(0);

        hoodServo = hardwareMap.servo.get("hoodServo");
        flickServo = hardwareMap.servo.get("flickServo");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    private void resetSlots() {
        for (int i = 0; i < 3; i++) slots[i] = Ball.EMPTY;
    }
}