package org.firstinspires.ftc.teamcode.decode.pedroPathing.autonomousv2.redfront.autonomousV2.blueback;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.teleOp.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.decode.teleOp.comptwotests.SpindexerV2;

import java.util.List;

@Autonomous(name = "! Blue Close Auto V2 Refactored", group = "Auto")
public class BlueBack extends LinearOpMode {

    // -----------------------------
    // HARDWARE
    // -----------------------------
    private Follower follower;
    private GeneratedPathsBlueBack paths;
    private CustomMecanumDrive drivetrain;

    private DcMotor intakeMotor, launchMotor, spinMotor;
    private Servo hoodServo, flickServo;
    private NormalizedColorSensor intakeColor, intakeColor2;
    private Limelight3A limelight3A;

    // -----------------------------
    // SPINDEXER (USE TELEOP LOGIC)
    // -----------------------------
    private enum Ball { EMPTY, PURPLE, GREEN }
    private Ball[] slots = {Ball.EMPTY, Ball.EMPTY, Ball.EMPTY};

    private static final int[] OUTTAKE_POS = {500, 0, 250};
    private static final int[] INTAKE_POS  = {125, 375, 625};

    private int index = 0;
    private boolean intakeActive = false;
    private boolean waitingForBall = false;

    private boolean waitingToRotate = false;
    private long colorDetectedTime = 0;
    private static final long COLOR_DELAY_MS = 100;
    private int nextIndexAfterDelay = -1;

    private double spinMotorSpeed = 0.35;
    private float gain = 20;

    // -----------------------------
    // BALL PATTERNS
    // -----------------------------
    private final Ball[] pattern21 = {Ball.GREEN, Ball.PURPLE, Ball.PURPLE};
    private final Ball[] pattern22 = {Ball.PURPLE, Ball.GREEN, Ball.PURPLE};
    private final Ball[] pattern23 = {Ball.PURPLE, Ball.PURPLE, Ball.GREEN};

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(GeneratedPathsBlueBack.START_POSE);
        paths = new GeneratedPathsBlueBack(follower);

        waitForStart();
        if (isStopRequested()) return;

        hoodServo.setPosition(0.8);
        flickServo.setPosition(0.90);

        // ------------------- START AUTO -------------------
        pivotServoOrHood(0.57);

//        // -------- PRE-SCAN BALLS --------
//        intakeActive = true; // mark intake active for INTAKE_POS
//        for (int slot = 0; slot < 3; slot++) {
//            rotateToIndex(slot); // rotate spindexer to intake position
//            sleep(250); // allow spinMotor to reach position
//
//            // detect ball at this slot
//            Ball detected = detectColor(intakeColor, intakeColor2);
//            slots[slot] = detected;
//        }
//        intakeActive = false; // done scanning

        // Scan path
        runPath(paths.scan(), 50, 1);

        // Detect AprilTag for pattern
        int tagId = detectAprilTag(1500);
        Ball[] correctPattern =
                tagId == 21 ? pattern21 :
                        tagId == 23 ? pattern23 : pattern22;

        for (Ball id : correctPattern) {
            telemetry.addData("id", id);
        }

        // First 3 balls
        runPath(paths.shoot(), 50, 1);
//        shootThreeBalls(correctPattern);

        // Intake 4th-6th balls using spindexer logic
        intakeActive = true;
        runPath(paths.toIntake1(), 50, 1);
//        runIntakePath(paths.intakeball3());
        runPath(paths.intakeball3(), 50, 0.6);


        runPath(paths.shoot2(), 50, 1);
//        shootThreeBalls(correctPattern);

        intakeActive = true;
        runPath(paths.toIntake2(), 250, 0.75);
//        runIntakePath(paths.intakeball6());
        runPath(paths.intakeball6(), 50, 0.6);

        runPath(paths.shoot3(), 250, 0.75);
//        shootThreeBalls(new Ball[]{Ball.PURPLE, Ball.GREEN, Ball.PURPLE});
    }

    // -----------------------------
    // TELEOP-SPINDEXER METHODS
    // -----------------------------
    private void runIntakePath(PathChain path) {
        follower.followPath(path);
        intakeMotor.setPower(-0.8);
        waitingForBall = true;

        

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            if (waitingForBall && intakeActive && !spinMotor.isBusy() && !waitingToRotate) {
                Ball detected = detectColor(intakeColor, intakeColor2);
                if (detected != Ball.EMPTY) {
                    slots[index] = detected;
                    waitingForBall = false;

                    nextIndexAfterDelay = findNextEmpty();
                    colorDetectedTime = System.currentTimeMillis();
                    waitingToRotate = true;
                }
            }

            if (waitingToRotate &&
                    System.currentTimeMillis() - colorDetectedTime >= COLOR_DELAY_MS) {
                if (nextIndexAfterDelay != -1) {
                    rotateToIndex(nextIndexAfterDelay);
                    waitingForBall = true;
                } else {
                    intakeActive = false;
                    rotateToIndex(0); // home
                }
                waitingToRotate = false;
            }
        }

        intakeMotor.setPower(0);
    }

    private void shootThreeBalls(Ball[] order) throws InterruptedException {
        launchMotor.setPower(0.9);
        sleep(300);

        for (Ball desired : order) {
            int target = findClosest(desired);
            if (target == -1) continue;

            rotateToIndex(target);
            while (opModeIsActive() && spinMotor.isBusy()) idle();

            flickServo.setPosition(0.75);
            sleep(500);
            flickServo.setPosition(0.9);

            slots[target] = Ball.EMPTY;
        }
        launchMotor.setPower(0);
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

    private Ball detectColor(NormalizedColorSensor s1, NormalizedColorSensor s2) {
        Ball ball1 = detectSingleSensor(s1);
        Ball ball2 = detectSingleSensor(s2);

        if (ball1 == Ball.PURPLE || ball2 == Ball.PURPLE) return Ball.PURPLE;
        if (ball1 == Ball.GREEN || ball2 == Ball.GREEN) return Ball.GREEN;
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
    // PATH + APRILTAG
    // -----------------------------
    private void runPath(PathChain path, int delayMs, double speed) {
        follower.setMaxPower(speed);
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) follower.update();
        follower.breakFollowing();
        stopDriveMotors();
        if (delayMs > 0) sleep(delayMs);
    }

    private void stopDriveMotors() {
        for (String name : new String[]{"frontLeft","frontRight","backLeft","backRight"}) {
            DcMotor m = hardwareMap.get(DcMotor.class, name);
            m.setPower(0);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private int detectAprilTag(long timeout) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < timeout) {
            LLResult r = limelight3A.getLatestResult();
            if (r != null && r.isValid()) {
                List<LLResultTypes.FiducialResult> f = r.getFiducialResults();
                if (!f.isEmpty()) return f.get(0).getFiducialId();
            }
            sleep(15);
        }
        return 22;
    }

    // -----------------------------
    // HARDWARE INIT (MATCH TELEOP)
    // -----------------------------
    private void initHardware() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");
        spinMotor   = hardwareMap.get(DcMotor.class, "spinMotor");

        hoodServo = hardwareMap.servo.get("hoodServo");
        flickServo = hardwareMap.servo.get("flickServo");

        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeColor = hardwareMap.get(NormalizedColorSensor.class, "intakeColor");
        intakeColor2 = hardwareMap.get(NormalizedColorSensor.class, "intakeColor2");

        intakeColor.setGain(gain);
        intakeColor2.setGain(gain);

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(3);
        limelight3A.start();

        drivetrain = new CustomMecanumDrive(hardwareMap);
    }

    private void pivotServoOrHood(double pos) {
        hoodServo.setPosition(pos);
    }
}