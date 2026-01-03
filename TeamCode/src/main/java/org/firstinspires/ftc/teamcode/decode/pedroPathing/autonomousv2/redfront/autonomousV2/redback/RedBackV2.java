package org.firstinspires.ftc.teamcode.decode.pedroPathing.autonomousv2.redfront.autonomousV2.redback;

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

@Autonomous(name = "! Red Back V2 Auto", group = "Auto")
public class RedBackV2 extends LinearOpMode {

    // -----------------------------
    // HARDWARE + GLOBAL VARS
    // -----------------------------
    private Follower follower;
    private GeneratedPathsRedBack paths;
    private CustomMecanumDrive drivetrain;

    private DcMotor intakeMotor, launchMotor, spinMotor;
    private Servo hoodServo, flickServo;
    private NormalizedColorSensor intakeColor, intakeColor2;
    private Limelight3A limelight3A;

    private final int[] OUTTAKE_POS = {500, 0, 250};
    private final int[] INTAKE_POS = {125, 375, 625};

    private enum Ball { EMPTY, PURPLE, GREEN }
    private Ball[] slots = {Ball.EMPTY, Ball.EMPTY, Ball.EMPTY};

    private int index = 0;
    private boolean intakeActive = false;
    private boolean waitingForBall = false;
    private boolean autoLaunching = false;
    private int autoLaunchTarget = -1;

    private long flickStartTime = 0;
    private boolean flicking = false;
    private boolean waitingAfterFlick = false;
    private long flickEndTime = 0;
    private static final long FLICK_TIME_MS = 500;
    private static final long POST_FLICK_DELAY_MS = 250;

    private boolean waitingToRotate = false;
    private long colorDetectedTime = 0;
    private int nextIndexAfterDelay = -1;

    private float gain = 20;
    private double spinMotorSpeed = 0.35;

    // -----------------------------
    // RUNOPMODE
    // -----------------------------
    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();
        resetSlots();

        // Initialize path follower
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(GeneratedPathsRedBack.START_POSE);
        paths = new GeneratedPathsRedBack(follower);

        telemetry.addLine("Ready to start RedBack V2 Auto");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        hoodServo.setPosition(0.8);
        flickServo.setPosition(0.9);

        // 1. Scan initial balls
        scanBalls();

        intakeMotor.setPower(1);
        runPath(paths.scan(), 50, 1);

        // 2. Detect AprilTag
        int tagId = detectAprilTag(2000);
        Ball[] correctPattern;
        switch (tagId) {
            case 21: correctPattern = new Ball[]{Ball.GREEN, Ball.PURPLE, Ball.PURPLE}; break;
            case 23: correctPattern = new Ball[]{Ball.PURPLE, Ball.PURPLE, Ball.GREEN}; break;
            default: correctPattern = new Ball[]{Ball.PURPLE, Ball.GREEN, Ball.PURPLE}; break;
        }

        // 3. Move to shoot position and shoot first set
        runPath(paths.shoot(), 50, 1);
        shootPattern(correctPattern);

        moveSpindexer(0, INTAKE_POS);

        // 4. Intake balls with color detection
        intakeMotor.setPower(0);
        runPath(paths.toIntake1(), 50, 1);
        intakeMotor.setPower(1);

        runIntakePath(paths.intakeball1(), 50, 0.5);
        sleep(1000); moveSpindexer(1, INTAKE_POS); sleep(500);

        runIntakePath(paths.intakeball2(), 50, 0.5);
        sleep(1000); moveSpindexer(2, INTAKE_POS); sleep(500);

        runIntakePath(paths.intakeball3(), 50, 0.5);
        sleep(1000); moveSpindexer(0, INTAKE_POS); sleep(500);

        scanBalls();

        runPath(paths.shoot2(), 50, 1);
        shootPattern(correctPattern);
        moveSpindexer(0, INTAKE_POS);

        intakeMotor.setPower(0);
        runPath(paths.toIntake2(), 250, 1);

        telemetry.addLine("RedBack V2 Auto Finished");
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

    private void runIntakePath(PathChain path, int stopDelayMs, double speed) {
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
    // SPINDEXER + LAUNCH
    // -----------------------------
    private void moveSpindexer(int targetIndex, int[] table) {
        index = targetIndex;
        int targetPos = closestModular(table[targetIndex], spinMotor.getCurrentPosition());
        spinMotor.setTargetPosition(targetPos);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(spinMotorSpeed);
    }

    private int closestModular(int target, int current) {
        int best = target;
        int minDiff = Integer.MAX_VALUE;
        for (int k = -2; k <= 2; k++) {
            int candidate = target + 750 * k;
            int diff = Math.abs(candidate - current);
            if (diff < minDiff) { minDiff = diff; best = candidate; }
        }
        return best;
    }

    private void shootPattern(Ball[] pattern) {
        launchMotor.setPower(0.9);
        sleep(750);

        for (Ball desired : pattern) {
            int slot = findClosestBall(desired);
            if (slot == -1) continue;

            moveSpindexer(slot, OUTTAKE_POS);

            // wait for sensor to detect ball
            long start = System.currentTimeMillis();
            while (opModeIsActive() && System.currentTimeMillis() - start < 1500) {
                if (detectColor(intakeColor, intakeColor2) == desired) break;
                sleep(20);
            }

            // fire
            flickServo.setPosition(0.75);
            hoodServo.setPosition(0.76);
            sleep(500);
            flickServo.setPosition(0.9);
            hoodServo.setPosition(0.8);

            slots[slot] = Ball.EMPTY;
            sleep(250);
        }

        launchMotor.setPower(0);
    }

    private int findClosestBall(Ball b) {
        for (int i = 0; i < 3; i++) {
            int idx = (index + i) % 3;
            if (slots[idx] == b) return idx;
        }
        return -1;
    }

    private void scanBalls() {
        slots[0] = detectColor(intakeColor, intakeColor2);
        slots[1] = detectColor(intakeColor, intakeColor2);
        slots[2] = detectColor(intakeColor, intakeColor2);
    }

    private Ball detectColor(NormalizedColorSensor s1, NormalizedColorSensor s2) {
        Ball b1 = detectSingle(s1);
        Ball b2 = detectSingle(s2);
        if (b1 == Ball.PURPLE || b2 == Ball.PURPLE) return Ball.PURPLE;
        if (b1 == Ball.GREEN  || b2 == Ball.GREEN) return Ball.GREEN;
        return Ball.EMPTY;
    }

    private Ball detectSingle(NormalizedColorSensor s) {
        NormalizedRGBA c = s.getNormalizedColors();
        float r = c.red, g = c.green, b = c.blue;
        float sum = r + g + b;
        if (sum < 0.07f) return Ball.EMPTY;
        if (b > r * 1.35f && b > g * 1.25f && b > 0.12f) return Ball.PURPLE;
        if (g > r * 1.15f && g > b * 1.15f && g > 0.15f) return Ball.GREEN;
        return Ball.EMPTY;
    }

    // -----------------------------
    // APRILTAG
    // -----------------------------
    private int detectAprilTag(long timeoutMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < timeoutMs) {
            LLResult llResult = limelight3A.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    int id = fiducials.get(0).getFiducialId();
                    if (id == 21 || id == 22 || id == 23) return id;
                }
            }
            sleep(15);
        }
        return 22; // default
    }

    // -----------------------------
    // HARDWARE INIT
    // -----------------------------
    private void initializeHardware() {
        spinMotor = hardwareMap.get(DcMotor.class, "spinMotor");
        intakeColor = hardwareMap.get(NormalizedColorSensor.class, "intakeColor");
        intakeColor2 = hardwareMap.get(NormalizedColorSensor.class, "intakeColor2");

        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeColor.setGain(gain); enableLight(intakeColor);
        intakeColor2.setGain(gain); enableLight(intakeColor2);

        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");
        launchMotor.setDirection(DcMotor.Direction.REVERSE);
        launchMotor.setPower(0);

        hoodServo = hardwareMap.servo.get("hoodServo");
        flickServo = hardwareMap.servo.get("flickServo");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        drivetrain = new CustomMecanumDrive(hardwareMap);
    }

    private void enableLight(NormalizedColorSensor s) {
        if (s instanceof SwitchableLight) ((SwitchableLight) s).enableLight(true);
    }

    private void resetSlots() {
        for (int i = 0; i < 3; i++) slots[i] = Ball.EMPTY;
    }
}