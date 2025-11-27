package org.firstinspires.ftc.teamcode.decode.pedroPathing.autonomous.redback;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.teleOp.CustomMecanumDrive;

@Autonomous(name = "!!!!!RedBack New Auto", group = "Auto")
public class RedBackNew extends LinearOpMode {

    // -----------------------------
    // HARDWARE + GLOBAL VARS
    // -----------------------------
    private Follower follower;
    private GeneratedPathsRedBack paths;

    private CustomMecanumDrive drivetrain;

    DcMotor intakeMotor;
    DcMotor launchMotor;
    DcMotor spinMotor;

    Servo pivotServo;
    Servo flickServo;

    NormalizedColorSensor backColor, leftColor, rightColor;

    int[] POSITIONS = {0, 250, 500};
    int[] INTAKE_POSITIONS = {352, -115, 142};

    ballColors[] balls = new ballColors[3];
    int index = 0;
    int currentTarget = 0;

    float gain = 20;

    boolean spinControlWas = false;

    enum ballColors {
        PURPLE, GREEN, EMPTY, UNKNOWN
    }

    // -----------------------------
    // RUNOPMODE
    // -----------------------------
    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();
        resetBallArray();

        // Initialize path follower
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(GeneratedPathsRedBack.START_POSE);
        paths = new GeneratedPathsRedBack(follower);

        telemetry.addLine("Ready to start RedBack New Auto");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ----------------------
        // 1. Scan all balls
        // ----------------------
        scanAllBalls();
        telemetry.addData("Balls", balls[0] + ", " + balls[1] + ", " + balls[2]);
        telemetry.update();

        intakeMotor.setPower(0.6);

        runPath(paths.scan(), 250, 0.75);

        // ----------------------
        // 2. Move to shooting position
        // ----------------------
        runPath(paths.shoot(), 250, 0.75);

        // ----------------------
        // 3. Shoot in order: purple → green → purple
        // ----------------------
        shootBallsByColorOrder(new ballColors[]{ballColors.PURPLE, ballColors.GREEN, ballColors.PURPLE});

        // ----------------------
        // 4. Continue auto sequence
        // ----------------------
        runPath(paths.toIntake1(), 250, 0.75);
        runIntakePath(paths.intakeball1(), 250, 0.5);
        sleep(250);
        moveSpindexer(0, INTAKE_POSITIONS);
        sleep(250);
        runIntakePath(paths.intakeball2(), 250, 0.5);
        sleep(250);
        moveSpindexer(1, INTAKE_POSITIONS);
        sleep(250);
        runIntakePath(paths.intakeball3(), 250, 0.5);
        sleep(250);
        moveSpindexer(2, INTAKE_POSITIONS);
        scanAllBalls();
        shootBallsByColorOrder(new ballColors[]{ballColors.PURPLE, ballColors.GREEN, ballColors.PURPLE});
        moveSpindexer(0, INTAKE_POSITIONS);

        runPath(paths.shoot2(), 250, 0.75);
        shootBallsByColorOrder(new ballColors[]{ballColors.PURPLE, ballColors.GREEN, ballColors.PURPLE});

        runPath(paths.toIntake2(), 250, 0.75);
        runIntakePath(paths.intakeball4(), 250, 0.5);
        sleep(250);
        moveSpindexer(0, INTAKE_POSITIONS);
        sleep(250);
        runIntakePath(paths.intakeball5(), 250, 0.5);
        sleep(250);
        moveSpindexer(1, INTAKE_POSITIONS);
        sleep(250);
        runIntakePath(paths.intakeball6(), 250, 0.5);
        sleep(250);
        moveSpindexer(2, INTAKE_POSITIONS);

        runPath(paths.shoot3(), 250, 0.75);
        shootBallsByColorOrder(new ballColors[]{ballColors.PURPLE, ballColors.GREEN, ballColors.PURPLE});

        telemetry.addLine("RedBack New Auto Finished");
        telemetry.update();
    }

    // -----------------------------
    // PATH HELPERS
    // -----------------------------
    private void runPath(PathChain path, int stopDelayMs, double speed) {
        follower.setMaxPower(speed);
        follower.followPath(path);

        while (opModeIsActive() && !isStopRequested() && follower.isBusy()) {
            follower.update();
        }

        follower.breakFollowing();
        stopDriveMotors();

        if (stopDelayMs > 0) sleep(stopDelayMs);
    }

    private void runIntakePath(PathChain path, int stopDelayMs, double speed) {
        follower.setMaxPower(speed);
        follower.followPath(path);

        while (opModeIsActive() && !isStopRequested() && follower.isBusy()) {
            follower.update();
        }

        follower.breakFollowing();
        stopDriveMotors();

        if (stopDelayMs > 0) sleep(stopDelayMs);
    }

    private void stopDriveMotors() {
        String[] driveMotors = {"frontLeft", "frontRight", "backLeft", "backRight"};
        for (String m : driveMotors) {
            DcMotor motor = hardwareMap.get(DcMotor.class, m);
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    // -----------------------------
    // BALL SHOOTING HELPERS
    // -----------------------------
    private void shootBallsByColorOrder(ballColors[] order) {
        for (ballColors color : order) {
            int idx = findClosestColor(color, 0);
            if (balls[idx] != ballColors.EMPTY) {
                moveToPosition(idx, POSITIONS);
                // Charge launcher 1 second
                launchMotor.setPower(1);
                sleep(1000);
                launchBallAt(idx);
                launchMotor.setPower(0);
                sleep(250);
            }
        }
    }

    private void launchBallAt(int index) {
        if (balls[index] != ballColors.EMPTY) {
            //0.6
            flickServo.setPosition(0.22);
            sleep(200);
            flickServo.setPosition(0);

            launchMotor.setPower(1);
            sleep(300);
            launchMotor.setPower(0);

            balls[index] = ballColors.EMPTY;
        }
    }

    // -----------------------------
    // TELEOP METHODS (copied line-for-line)
    // -----------------------------

    private void moveSpindexer(int newIndex, int[] table) {
        currentTarget = table[newIndex];
        runToPosition(spinMotor, currentTarget, 0.4);
    }

    private void moveToPosition(int newIndex, int[] table) {
        currentTarget = table[newIndex];
        runToPosition(spinMotor, currentTarget, 0.4);
    }

    private void runToPosition(DcMotor motor, int targetTicks, double power) {
        motor.setTargetPosition(targetTicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    public int findClosestColor(ballColors target, int currentIndex) {
        for (int offset = 0; offset < balls.length; offset++) {
            int right = (currentIndex + offset) % balls.length;
            int left  = (currentIndex - offset + balls.length) % balls.length;
            if (balls[right] == target) return right;
            if (balls[left] == target)  return left;
        }
        return currentIndex;
    }

    public int findClosestEmpty(int currentIndex) {
        return findClosestColor(ballColors.EMPTY, currentIndex);
    }

    public void scanAllBalls() {
        balls[0] = detectBallColorFromSensor(backColor);
        balls[1] = detectBallColorFromSensor(rightColor);
        balls[2] = detectBallColorFromSensor(leftColor);
    }

    private ballColors detectBallColorFromSensor(NormalizedColorSensor sensor) {
        NormalizedRGBA c = sensor.getNormalizedColors();

        float r = c.red, g = c.green, b = c.blue;
        float tol = 0.20f;

        if (b > r * (1 + tol) && b > g * (1 + tol)) return ballColors.PURPLE;
        if (g > r * (1 + tol) && g > b * (1 + tol)) return ballColors.GREEN;

        if (r > 0.01 || g > 0.01 || b > 0.01) return ballColors.UNKNOWN;
        return ballColors.EMPTY;
    }

    // -----------------------------
    // HARDWARE INIT
    // -----------------------------
    private void resetBallArray() {
        balls[0] = ballColors.EMPTY;
        balls[1] = ballColors.EMPTY;
        balls[2] = ballColors.EMPTY;
    }

    private void initializeHardware() {
        initLauncher();
        initIntake();

        backColor  = hardwareMap.get(NormalizedColorSensor.class, "backColor");
        leftColor  = hardwareMap.get(NormalizedColorSensor.class, "leftColor");
        rightColor = hardwareMap.get(NormalizedColorSensor.class, "rightColor");

        backColor.setGain(gain);
        leftColor.setGain(gain);
        rightColor.setGain(gain);

        if (backColor instanceof SwitchableLight)
            ((SwitchableLight) backColor).enableLight(true);
        if (leftColor instanceof SwitchableLight)
            ((SwitchableLight) leftColor).enableLight(true);
        if (rightColor instanceof SwitchableLight)
            ((SwitchableLight) rightColor).enableLight(true);

        drivetrain = new CustomMecanumDrive(hardwareMap);
    }

    private void initLauncher() {
        pivotServo = hardwareMap.servo.get("launchServo");
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");
        flickServo = hardwareMap.servo.get("flickServo");
    }

    private void initIntake() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        spinMotor = hardwareMap.get(DcMotor.class, "spinMotor");
        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinMotor.setDirection(DcMotor.Direction.REVERSE);
    }
}