package org.firstinspires.ftc.teamcode.decode.pedroPathing.statesAutos.twelveClose.blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.teleOp.tests.CustomMecanumDrive;

import java.util.Arrays;

@Autonomous(name = "!!!!! STATES BLUE BACK 12 BALL TESTING TESTING DUDUDUDU")
public class BlueClose12Ball extends LinearOpMode {

    private int index = 0;
    private float gain = 20;

    private enum Ball { EMPTY, PURPLE, GREEN }
    private Ball[] slots = {Ball.EMPTY, Ball.EMPTY, Ball.EMPTY};

    // ---------------- DRIVE ----------------
    private Follower follower;
    private GeneratedPathsBlue12BallCloseV3 paths;
    private CustomMecanumDrive drivetrain;
    private Limelight3A limelight;

    // ---------------- HARDWARE ----------------
    private DcMotor spinMotor;
    private DcMotorEx launchMotor;
    private DcMotor intakeMotor;
    private DcMotorEx flickMotor;
    private Servo hoodServo;

    NormalizedColorSensor intakeColor;
    NormalizedColorSensor intakeColor2;

    private static final int[] OUTTAKE_POS = {500, 0, 250};
    private static final int[] INTAKE_POS  = {125, 375, 625};

    private double spinMotorSpeed = 0.38;

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();
        resetSlots();

        slots[0] = Ball.PURPLE;
        slots[1] = Ball.GREEN;
        slots[2] = Ball.PURPLE;

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(GeneratedPathsBlue12BallCloseV3.START_POSE);
        paths = new GeneratedPathsBlue12BallCloseV3(follower);

        hoodServo.setPosition(0.80);

        telemetry.addLine("Blue Auto Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        launchMotor.setVelocity(1700);

        Ball[] pattern = getPatternFromTag();
        aimToPattern(pattern);

        runPath(paths.shoot(), 400, 1);
        shootAll();

        runPath(paths.toIntake1(), 0, 1);
        runPathWithIntake(paths.intake1(), 0, 0.21);
        runPath(paths.gate(), 750, 1);
        shootAll();

        runPath(paths.toIntake2(), 0, 1);
        runPathWithIntake(paths.intake2(), 0, 0.21);
        shootAll();

        runPath(paths.toIntake3(), 0, 1);
        runPathWithIntake(paths.intake3(), 0, 0.21);
        shootAll();

        runPath(paths.leave(), 0, 1);

        telemetry.addLine("Blue Auto Finished");
        telemetry.update();
    }

    // ---------------- SPINDEXER ----------------
    private void rotateToIndex(int target) {
        index = target;
        int base = INTAKE_POS[target];
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

    private void aimToPattern(Ball[] pattern) {
        int idx = findBestStartIndex(pattern);
        rotateToIndex(idx);
    }

    private int findBestStartIndex(Ball[] pattern) {
        int greenIndex = -1;
        for (int i = 0; i < 3; i++) {
            if (slots[i] == Ball.GREEN) {
                greenIndex = i;
                break;
            }
        }
        if (greenIndex == -1) return 0;

        if (pattern[0] == Ball.GREEN) return greenIndex;
        if (pattern[1] == Ball.GREEN) return (greenIndex + 2) % 3;
        return (greenIndex + 1) % 3;
    }

    // ---------------- SHOOT ----------------
    private void shootAll() {

        intakeMotor.setPower(0);
        flickMotor.setPower(1);
        launchMotor.setVelocity(1700);
        sleep(200);

        int endPosition = spinMotor.getCurrentPosition() + 500;
        spinMotor.setTargetPosition(endPosition);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(0.3);

        while (opModeIsActive() && spinMotor.isBusy()) {}

        spinMotor.setPower(0);
        flickMotor.setPower(0);

        Arrays.fill(slots, Ball.EMPTY);
        index = (index + 2) % 3;
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
        intakeMotor.setPower(-0.6);
        follower.setMaxPower(speed);
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) follower.update();
        follower.breakFollowing();
        intakeMotor.setPower(0);
    }

    // ---------------- APRILTAG ----------------
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
        }
        return 22;
    }

    // ---------------- INIT ----------------
    private void initHardware() {

        drivetrain = new CustomMecanumDrive(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);
        limelight.start();

        spinMotor = hardwareMap.get(DcMotor.class, "spinMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        flickMotor = hardwareMap.get(DcMotorEx.class, "flickMotor");
        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        hoodServo = hardwareMap.servo.get("hoodServo");

        intakeColor = hardwareMap.get(NormalizedColorSensor.class, "intakeColor");
        intakeColor2 = hardwareMap.get(NormalizedColorSensor.class, "intakeColor2");
    }

    private void resetSlots() {
        Arrays.fill(slots, Ball.EMPTY);
    }
}