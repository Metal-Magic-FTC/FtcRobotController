package org.firstinspires.ftc.teamcode.decode.pedroPathing.statesAutos.twelveClose.temp;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.teleOp.tests.CustomMecanumDrive;

import java.util.Arrays;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class BlueClose12Ball extends LinearOpMode {

    private int index = 0;

    private float gain = 20;

    private enum Ball { EMPTY, PURPLE, GREEN }
    private Ball[] slots = {Ball.EMPTY, Ball.EMPTY, Ball.EMPTY};

    // ---------------- DRIVE ----------------
    private Follower follower;
    private GeneratedPathsBlue12BallClose paths;
    private CustomMecanumDrive drivetrain;
    private Limelight3A limelight;

    // ---------------- INTAKE, TRANSFER, SCORING ----------------

    private DcMotor spinMotor;
    private DcMotorEx launchMotor;
    private DcMotor intakeMotor;
    private DcMotorEx flickMotor;

    Servo hoodServo;

    NormalizedColorSensor intakeColor;
    NormalizedColorSensor intakeColor2;

    private static final int[] OUTTAKE_POS = {500, 0, 250};
    private static final int[] INTAKE_POS  = {125, 375, 625};

    private double spinMotorSpeed = 0.38;

    private boolean intakeActive = false;
    private boolean waitingToRotate = false;
    private boolean waitingForBall = false;
    private long colorDetectedTime = 0;
    private static final long COLOR_DELAY_MS = 2; // 100 ms delay before spinning
    private int nextIndexAfterDelay = -1;

    private static final int SPIN_TOLERANCE_TICKS = 5;
    private static final long SPIN_TIMEOUT_MS = 10000;

    private int lastSpinTarget = 0;

    // ---------------- RUN ----------------
    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();
        resetSlots();

        slots[0] = Ball.PURPLE;
        slots[1] = Ball.GREEN;
        slots[2] = Ball.PURPLE;

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(GeneratedPathsBlue12BallClose.START_POSE);
        paths = new GeneratedPathsBlue12BallClose(follower);
        hoodServo.setPosition(0.80);


        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        launchMotor.setVelocity(1700);

        // scan balls
        //scanBallsInSlots(5000);

        //runPath(paths.scan(), 0, 1.0);

        runPath(paths.scan(), 0, 1);
        Ball[] pattern = getPatternFromTag();

        aimToPattern(pattern);
        telemetry.addData("pattern", pattern[0].toString() + " " + pattern[1].toString() + " " + pattern[2].toString());
        telemetry.update();

        runPath(paths.shoot(), 400, 1);

        // ---- SHOOT ----
        //shootAllPattern(pattern);
        shootAll();

        intakeMotor.setPower(-0.6);
        intakeActive = true;
        rotateToIndex(0);
        resetSlots();

        // ---- INTAKE 1–3 ----
        intakeActive = true;
        rotateToIndex(0);
        runPath(paths.toIntake1(), 0, 1);
        resetSlots();

        runPathWithIntake(paths.intake1(), 0, 0.21);
        double startTime = System.currentTimeMillis();
//        while (System.currentTimeMillis() < startTime + 500) {
//            waitingForBall = true;
//            intakeActive = true;
//            intake();
//        }



        runPathWithIntake(paths.gate(), 750, 1);

        intakeMotor.setPower(-0.6);
        slots[0] = Ball.PURPLE;
        slots[1] = Ball.PURPLE;
        slots[2] = Ball.GREEN;

        aimToPattern(pattern);

        slots[0] = Ball.PURPLE;
        slots[1] = Ball.PURPLE;
        slots[2] = Ball.GREEN;

        runPath(paths.shoot2(), 0, 1);

        // ---- SHOOT ----
        //shootAllPattern(pattern);
        shootAll();

        intakeMotor.setPower(-0.6);
        intakeActive = true;
        rotateToIndex(0);
        resetSlots();

        // ---- INTAKE 4–6 ----
        intakeActive = true;
        rotateToIndex(0);
        runPath(paths.toIntake2(), 0, 1);
        runPathWithIntake(paths.intake2(), 0, 0.21);

        intakeMotor.setPower(-0.6);
        slots[0] = Ball.PURPLE;
        slots[1] = Ball.GREEN;
        slots[2] = Ball.PURPLE;

        aimToPattern(pattern);

        slots[0] = Ball.PURPLE;
        slots[1] = Ball.GREEN;
        slots[2] = Ball.PURPLE;

        runPath(paths.shoot3(), 0, 1);

        //shootAllPattern(pattern);
        shootAll();

        intakeMotor.setPower(-0.6);
        intakeActive = true;
        rotateToIndex(0);
        resetSlots();

        runPath(paths.toIntake3(), 0, 1);
        runPathWithIntake(paths.intake3(), 0, 0.21);

        intakeMotor.setPower(-0.6);
        slots[0] = Ball.GREEN;
        slots[1] = Ball.PURPLE;
        slots[2] = Ball.PURPLE;

        aimToPattern(pattern);

        slots[0] = Ball.GREEN;
        slots[1] = Ball.PURPLE;
        slots[2] = Ball.PURPLE;

        runPath(paths.shoot4(), 0, 1);

        //shootAllPattern(pattern);
        shootAll();

        runPath(paths.leave(), 0, 1);
        intakeMotor.setPower(-0.6);
        intakeActive = true;
        rotateToIndex(0);
        resetSlots();

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

    private void aimToPattern(Ball[] pattern) {

        intakeActive = false;
        int idx = findBestStartIndex(pattern);
        index = idx;
        rotateToIndex(idx);
    }

    private int indexAimToPattern(Ball[] pattern) {
        int greenInPattern = findGreen(pattern);
        int greenInSlots = findGreen(slots);

        int idx = greenInSlots - greenInPattern;

        idx %= 3;

        return idx;
    }

    private int findGreen(Ball[] pattern) {
        return Arrays.asList(pattern).indexOf(Ball.GREEN);
    }

    private int findNextEmpty() {
        for (int i = 0; i < 3; i++) {
            int idx = (index + i) % 3;
            if (slots[idx] == Ball.EMPTY) return idx;
        }
        return -1;
    }

    private boolean spinAtTarget() {
        return Math.abs(
                spinMotor.getTargetPosition() - spinMotor.getCurrentPosition()
        ) <= 8;
    }

    private void intake() {

        // spindexer logic (COLOR-BASED DETECTION) with delay
        if (waitingForBall && intakeActive && spinAtTarget() && !waitingToRotate) {
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
        follower.setMaxPower(speed);
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) follower.update();
        follower.breakFollowing();
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

        //launchMotor.setVelocity(900);
        intakeActive = true;
        waitingForBall = true;

        intakeMotor.setPower(-0.6);

        follower.setMaxPower(speed);
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            waitingForBall = true;
            intakeActive = true;
            intake();

        }
        follower.breakFollowing();
        if (stopDelay > 0) sleep(stopDelay);

        intakeMotor.setPower(0);

        intakeActive = false;
        waitingForBall = false;

        launchMotor.setVelocity(1700);

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
        if (id == 21) return new Ball[]{Ball.GREEN, Ball.PURPLE, Ball.PURPLE};
        if (id == 23) return new Ball[]{Ball.PURPLE, Ball.PURPLE, Ball.GREEN};
        return new Ball[]{Ball.PURPLE, Ball.GREEN, Ball.PURPLE};
    }

    private int detectAprilTag(long timeoutMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < timeoutMs) {
            LLResult r = limelight.getLatestResult();
            if (r != null && r.isValid() && !r.getFiducialResults().isEmpty())
                return r.getFiducialResults().get(0).getFiducialId();
            sleep(15);
        }
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
        if (total < 0.03f) return Ball.EMPTY;

        // PURPLE: blue-dominant (keep strict)
        if (b > r * 1.2f && b > g * 1.1f && b > 0.12f) {
            return Ball.PURPLE;
        }

        // GREEN: looser dominance + absolute floor
        if (g > r * 1.15f && g > b * 1.15f && g > 0.15f) {
            return Ball.GREEN;
        }

        return Ball.EMPTY;
    }

    private int findBestStartIndex(Ball[] pattern) {

        int greenIndex = -1;

        // Find the green ball on the spindexer
        for (int i = 0; i < 3; i++) {
            if (slots[i] == Ball.GREEN) {
                greenIndex = i;
                break;
            }
        }

        if (greenIndex == -1) return -1; // safety

        // Determine start index based on pattern
        // Patterns: PPG, PGP, GPP
        if (pattern[0] == Ball.GREEN) {
            // GPP
            return greenIndex;
        }

        if (pattern[1] == Ball.GREEN) {
            // PGP
            return (greenIndex + 2) % 3;
        }

        if (pattern[2] == Ball.GREEN) {
            // PPG
            return (greenIndex + 1) % 3;
        }

        return -1;
    }

    private void shootAllPattern(Ball[] pattern) {

        intakeActive = false;
        intakeMotor.setPower(0);

        int startIndex = findBestStartIndex(pattern);
        if (startIndex == -1) return; // nothing valid to shoot

        // Move spindexer to correct start slot
        index = startIndex;

        int current = spinMotor.getCurrentPosition();
        int currentMod = ((current % 750) + 750) % 750;

        int startPos = OUTTAKE_POS[startIndex];
        if (startPos <= currentMod) {
            startPos += 750;
        }

        int startTarget = current - currentMod + startPos;

        spinMotor.setTargetPosition(startTarget);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(0.38);

        waitForSpindexer();

        sleep(100);

        // ---- START SHOOTING ----
        flickMotor.setPower(1);
        launchMotor.setVelocity(1700);

        sleep(100);

        // Compute end sweep position (clockwise through 3 slots)
        int endSlot = (startIndex + 2) % 3;
        int endPos = OUTTAKE_POS[endSlot] + 750;

        int sweepTarget = startTarget + (endPos - startPos);

        spinMotor.setTargetPosition(sweepTarget);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(0.38);

        while (opModeIsActive() && spinMotor.isBusy()) {
            // let balls fire naturally
        }

        spinMotor.setPower(0);

        flickMotor.setPower(1);
        sleep(100);
        flickMotor.setPower(0);

        // ---- STOP ----
        flickMotor.setPower(0);
        //launchMotor.setVelocity(900);

        // Clear slots in pattern order
        slots[0] = Ball.EMPTY;
        slots[1] = Ball.EMPTY;
        slots[2] = Ball.EMPTY;

        index = endSlot;
    }

    private void shootAll() {

        intakeMotor.setPower(0);

        flickMotor.setPower(1);
        launchMotor.setVelocity(1700);
        sleep(200);

        int endPosition = spinMotor.getCurrentPosition() + 500;
        spinMotor.setTargetPosition(endPosition);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(0.3);

        while (opModeIsActive() && spinMotor.isBusy()) {
            // let balls fire naturally
        }

        spinMotor.setPower(0);

        flickMotor.setPower(1);
        sleep(200);
        flickMotor.setPower(0);

        slots[0] = Ball.EMPTY;
        slots[1] = Ball.EMPTY;
        slots[2] = Ball.EMPTY;

        index = (index + 2) % 3;

        intakeMotor.setPower(-0.6);

    }

    // ---------------- INIT ----------------
    private void initHardware() {

        drivetrain = new CustomMecanumDrive(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);  // APRILTAG PIPELINE
        limelight.start();

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
        launchMotor.setDirection(DcMotorEx.Direction.FORWARD); // same as TeleOp_Flick_Launch

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(300, 0, 0, 12.9);
        launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        launchMotor.setVelocity(0);

        hoodServo = hardwareMap.servo.get("hoodServo");

        flickMotor = hardwareMap.get(DcMotorEx.class, "flickMotor");
        flickMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flickMotor.setPower(0);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    private void resetSlots() {
        for (int i = 0; i < 3; i++) slots[i] = Ball.EMPTY;
    }
}