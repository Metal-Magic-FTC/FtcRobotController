package org.firstinspires.ftc.teamcode.decode.pedroPathing.autov2.redback;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.teleOp.CustomMecanumDrive;

import java.util.List;

@Autonomous(name = "!! Red Close Jan 10", group = "Auto")
public class RedBackV2 extends LinearOpMode {

    // -----------------------------
    // DRIVE / PATHING
    // -----------------------------
    private Follower follower;
    private GeneratedPathsRedBack paths;
    private CustomMecanumDrive drivetrain;
    private Limelight3A limelight3A;

    // -----------------------------
    // SPINDEXER (IDENTICAL TO TELEOP)
    // -----------------------------
    private DcMotor spinMotor;
    private DcMotor intakeMotor;
    private DcMotor launchMotor;
    private Servo hoodServo;
    private Servo flickServo;

    private NormalizedColorSensor intakeColor;
    private NormalizedColorSensor intakeColor2;

    private static final int[] OUTTAKE_POS = {500, 0, 250};
    private static final int[] INTAKE_POS  = {125, 375, 625};

    private enum Ball { EMPTY, PURPLE, GREEN }
    private Ball[] slots = {Ball.EMPTY, Ball.EMPTY, Ball.EMPTY};

    private int index = 0;
    private boolean intakeActive = false;
    private boolean waitingForBall = false;

    private float gain = 20;
    private double spinMotorSpeed = 0.35;

    // ---- COLOR SENSOR DELAY (EXACT MATCH) ----
    private boolean waitingToRotate = false;
    private long colorDetectedTime = 0;
    private static final long COLOR_DELAY_MS = 100;
    private int nextIndexAfterDelay = -1;

    // ---- FLICK TIMING (EXACT MATCH) ----
    private static final long FLICK_TIME_MS = 500;
    private static final long POST_FLICK_DELAY_MS = 250;

    // -----------------------------
    // RUN OPMODE
    // -----------------------------
    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();
        resetSlots();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(GeneratedPathsRedBack.START_POSE);
        paths = new GeneratedPathsRedBack(follower);

        telemetry.addLine("RedBack V2 Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ---- SCAN EXISTING BALLS (USING SAME LOGIC) ----
        runPath(paths.scan(), 50, 1);
        scanAllSlots();

        // ---- APRILTAG ----
        int tagId = detectAprilTag(2000);
        Ball[] correctPattern;
        switch (tagId) {
            case 21: correctPattern = new Ball[]{Ball.GREEN, Ball.PURPLE, Ball.PURPLE}; break;
            case 23: correctPattern = new Ball[]{Ball.PURPLE, Ball.PURPLE, Ball.GREEN}; break;
            default: correctPattern = new Ball[]{Ball.PURPLE, Ball.GREEN, Ball.PURPLE}; break;
        }

        // ---- SHOOT FIRST SET ----
        runPath(paths.shoot(), 50, 1);
        shootPattern(correctPattern);

        // ---- INTAKE BALLS 1–3 (CONTINUOUS) ----
        runPath(paths.toIntake1(), 50, 1);
        runPathWithIntake(paths.intakeball3(), 250, 0.2);

        // ---- SHOOT SECOND SET ----
        runPath(paths.shoot2(), 50, 1);
        shootPattern(correctPattern);

        // ---- INTAKE BALLS 4–6 (CONTINUOUS) ----
        runPath(paths.toIntake2(), 250, 1);
        runPathWithIntake(paths.intakeball6(), 250, 0.2);

        telemetry.addLine("RedBack V2 Finished");
        telemetry.update();
    }

    // -----------------------------
    // PATH HELPERS
    // -----------------------------
    private void runPath(PathChain path, int stopDelayMs, double speed) {
        follower.setMaxPower(speed);
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) follower.update();
        follower.breakFollowing();
        stopDriveMotors();
        if (stopDelayMs > 0) sleep(stopDelayMs);
    }

    private void stopDriveMotors() {
        for (String m : new String[]{"frontLeft","frontRight","backLeft","backRight"}) {
            DcMotor motor = hardwareMap.get(DcMotor.class, m);
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    // -----------------------------
    // APRILTAG
    // -----------------------------
    private int detectAprilTag(long timeoutMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < timeoutMs) {
            LLResult ll = limelight3A.getLatestResult();
            if (ll != null && ll.isValid()) {
                List<LLResultTypes.FiducialResult> tags = ll.getFiducialResults();
                if (tags != null && !tags.isEmpty()) {
                    int id = tags.get(0).getFiducialId();
                    if (id == 21 || id == 22 || id == 23) return id;
                }
            }
            sleep(15);
        }
        return 22;
    }

    // -----------------------------
    // SPINDEXER — IDENTICAL LOGIC
    // -----------------------------
    private void scanAllSlots() {

        intakeActive = true;
        waitingForBall = true;
        intakeMotor.setPower(-0.8);

        for (int i = 0; i < 3 && opModeIsActive(); i++) {
            rotateToIndex(i);
            waitForSpin();

            long start = System.currentTimeMillis();
            while (opModeIsActive() && System.currentTimeMillis() - start < 600) {
                Ball detected = detectColor(intakeColor, intakeColor2);
                if (detected != Ball.EMPTY) {
                    slots[i] = detected;
                    break;
                }
            }
        }

        intakeMotor.setPower(0);
        intakeActive = false;
    }

    private void shootPattern(Ball[] pattern) {

        launchMotor.setPower(1);
        sleep(500);

        for (Ball target : pattern) {
            int idx = findSlot(target);
            if (idx == -1) continue;

            rotateToIndex(idx);
            waitForSpin();

            flickServo.setPosition(0.75);
            sleep(FLICK_TIME_MS);
            flickServo.setPosition(0.9);

            slots[idx] = Ball.EMPTY;
            index = idx;
            sleep(POST_FLICK_DELAY_MS);
        }

        launchMotor.setPower(0);
    }

    private void runPathWithIntake(PathChain path, int stopDelayMs, double speed) {

        intakeActive = true;
        waitingForBall = true;
        intakeMotor.setPower(-0.8);

        follower.setMaxPower(speed);
        follower.followPath(path);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateSpindexer();   // EXACT SAME LOOP AS TELEOP
        }

        intakeMotor.setPower(0);
        intakeActive = false;

        follower.breakFollowing();
        stopDriveMotors();
        if (stopDelayMs > 0) sleep(stopDelayMs);
    }

    private void updateSpindexer() {

        if (waitingForBall && intakeActive && !spinMotor.isBusy() && !waitingToRotate) {

            Ball detected = detectColor(intakeColor, intakeColor2);
            if (detected != Ball.EMPTY) {

                slots[index] = detected;
                waitingForBall = false;

                int nextEmpty = findNextEmpty();
                nextIndexAfterDelay = nextEmpty;
                colorDetectedTime = System.currentTimeMillis();
                waitingToRotate = true;
            }
        }

        if (waitingToRotate && System.currentTimeMillis() - colorDetectedTime >= COLOR_DELAY_MS) {

            if (nextIndexAfterDelay != -1) {
                rotateToIndex(nextIndexAfterDelay);
                waitingForBall = true;
            } else {
                intakeActive = false;
            }

            waitingToRotate = false;
        }
    }

    private void rotateToIndex(int target) {
        index = target;
        int base = intakeActive ? INTAKE_POS[target] : OUTTAKE_POS[target];
        int targetPos = closestModular(base, spinMotor.getCurrentPosition());

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

    private void waitForSpin() {
        while (opModeIsActive() && spinMotor.isBusy()) sleep(10);
    }

    private int findNextEmpty() {
        for (int i = 0; i < 3; i++) {
            int idx = (index + i) % 3;
            if (slots[idx] == Ball.EMPTY) return idx;
        }
        return -1;
    }

    private int findSlot(Ball target) {
        for (int i = 0; i < 3; i++) {
            if (slots[i] == target) return i;
        }
        return -1;
    }

    // ---- COLOR DETECTION (EXACT COPY) ----
    private Ball detectColor(NormalizedColorSensor s1, NormalizedColorSensor s2) {
        Ball b1 = detectSingleSensor(s1);
        Ball b2 = detectSingleSensor(s2);

        if (b1 == Ball.PURPLE || b2 == Ball.PURPLE) return Ball.PURPLE;
        if (b1 == Ball.GREEN  || b2 == Ball.GREEN)  return Ball.GREEN;

        return Ball.EMPTY;
    }

    private Ball detectSingleSensor(NormalizedColorSensor sensor) {
        NormalizedRGBA c = sensor.getNormalizedColors();
        float r = c.red, g = c.green, b = c.blue;

        float total = r + g + b;
        if (total < 0.07f) return Ball.EMPTY;

        if (b > r * 1.35f && b > g * 1.25f && b > 0.12f) return Ball.PURPLE;
        if (g > r * 1.15f && g > b * 1.15f && g > 0.15f) return Ball.GREEN;

        return Ball.EMPTY;
    }

    // -----------------------------
    // HARDWARE INIT
    // -----------------------------
    private void initializeHardware() {

        drivetrain = new CustomMecanumDrive(hardwareMap);
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");

        spinMotor   = hardwareMap.get(DcMotor.class, "spinMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");

        hoodServo  = hardwareMap.servo.get("hoodServo");
        flickServo = hardwareMap.servo.get("flickServo");

        intakeColor  = hardwareMap.get(NormalizedColorSensor.class, "intakeColor");
        intakeColor2 = hardwareMap.get(NormalizedColorSensor.class, "intakeColor2");

        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeColor.setGain(gain);
        intakeColor2.setGain(gain);
        enableLight(intakeColor);
        enableLight(intakeColor2);

        hoodServo.setPosition(0.8);
        flickServo.setPosition(0.9);
    }

    private void enableLight(NormalizedColorSensor s) {
        if (s instanceof SwitchableLight) ((SwitchableLight) s).enableLight(true);
    }

    private void resetSlots() {
        for (int i = 0; i < 3; i++) slots[i] = Ball.EMPTY;
    }
}