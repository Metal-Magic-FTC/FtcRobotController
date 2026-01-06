package org.firstinspires.ftc.teamcode.decode.pedroPathing.autov2.redback;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.teleOp.CustomMecanumDrive;

@Autonomous(name = "!! Red Close Jan 10", group = "Auto")
public class RedBackV2 extends LinearOpMode {

    // ---------------- DRIVE ----------------
    private Follower follower;
    private GeneratedPathsRedBack paths;
    private CustomMecanumDrive drivetrain;
    private Limelight3A limelight;

    // ---------------- SPINDEXER ----------------
    private DcMotor spinMotor, intakeMotor, launchMotor;
    private Servo hoodServo, flickServo;
    private NormalizedColorSensor intakeColor, intakeColor2;

    private static final int[] OUTTAKE_POS = {500, 0, 250};
    private static final int[] INTAKE_POS  = {125, 375, 625};

    private enum Ball { EMPTY, PURPLE, GREEN }
    private Ball[] slots = {Ball.EMPTY, Ball.EMPTY, Ball.EMPTY};

    private int index = 0;
    private boolean intakeActive = false;
    private boolean waitingForBall = false;
    private boolean waitingToRotate = false;

    private long colorDetectedTime = 0;
    private int nextIndexAfterDelay = -1;

    private static final long COLOR_DELAY_MS = 100;
    private static final long FLICK_TIME_MS = 500;
    private static final long POST_FLICK_DELAY_MS = 250;

    private float gain = 20;
    private double spinMotorSpeed = 0.35;

    private int scanIndex = 0;

    // ---------------- RUN ----------------
    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();
        resetSlots();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(GeneratedPathsRedBack.START_POSE);
        paths = new GeneratedPathsRedBack(follower);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        scanAllBalls();

        // ---- SCAN WHILE MOVING (OLD LOGIC) ----
        runPath(paths.scan(), 250, 1.0);
        runPath(paths.shoot(), 250, 1.0);

        // ---- APRILTAG ----
        Ball[] pattern = getPatternFromTag();

        // ---- SHOOT ----
        shootPattern(pattern);

        // ---- INTAKE 1–3 ----
        runPath(paths.toIntake1(), 50, 1);
        runPathWithIntake(paths.intakeball3(), 0.3);

        // ---- SHOOT AGAIN ----
        runPath(paths.shoot2(), 50, 1);
        shootPattern(pattern);

        // ---- INTAKE 4–6 ----
        runPath(paths.toIntake2(), 50, 1);
        runPathWithIntake(paths.intakeball6(), 0.3);

        telemetry.addLine("Finished");
        telemetry.update();
    }

    // ---------------- PATH HELPERS ----------------
    private void runPath(PathChain path, int stopDelay, double speed) {
        follower.setMaxPower(speed);
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) follower.update();
        follower.breakFollowing();
        if (stopDelay > 0) sleep(stopDelay);
    }

    private void runPathWithScan(PathChain path, double speed) {
        follower.setMaxPower(speed);
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateSpindexer(); // OLD SCAN = just normal intake logic
        }
        follower.breakFollowing();
    }

    // ---------------- OLD SCAN LOGIC ----------------
    private void scanAllBalls() {
        intakeMotor.setPower(-0.8); // run intake
        scanIndex = 0;

        // scan all 3 slots
        while (opModeIsActive() && scanIndex < 3) {
            // move spindexer to current slot
            rotateToIndex(scanIndex);
            waitForSpin();

            // detect color
            Ball detected = detectColor(intakeColor, intakeColor2);
            if (detected != Ball.EMPTY) {
                slots[scanIndex] = detected;
            } else {
                slots[scanIndex] = Ball.PURPLE; // default assumption
            }

            scanIndex++;
        }

        intakeMotor.setPower(0); // stop intake
    }

    // ---------------- INTAKE ----------------
    private void runPathWithIntake(PathChain path, double speed) {

        intakeActive = true;
        waitingForBall = true;

        moveSpindexerToIntake(); // ✅ intake alignment fix
        intakeMotor.setPower(-0.8);

        follower.setMaxPower(speed);
        follower.followPath(path);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateSpindexer();
        }

        intakeMotor.setPower(0);
        intakeActive = false;
        follower.breakFollowing();
    }

    private void moveSpindexerToIntake() {
        rotateToIndex(index);
        waitForSpin();
    }

    // ---------------- SPINDEXER CORE ----------------
    private void updateSpindexer() {

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

        if (waitingToRotate && System.currentTimeMillis() - colorDetectedTime >= COLOR_DELAY_MS) {

            if (nextIndexAfterDelay != -1) {
                rotateToIndex(nextIndexAfterDelay);
                waitingForBall = true;
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
        int best = mod, min = Integer.MAX_VALUE;
        for (int k = -2; k <= 2; k++) {
            int c = mod + 750 * k;
            int d = Math.abs(c - current);
            if (d < min) { min = d; best = c; }
        }
        return best;
    }

    private void waitForSpin() {
        while (opModeIsActive() && spinMotor.isBusy()) sleep(10);
    }

    // ---------------- SHOOTING ----------------
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

    // ---------------- UTILS ----------------
    private Ball detectColor(NormalizedColorSensor s1, NormalizedColorSensor s2) {
        Ball b1 = detectSingle(s1), b2 = detectSingle(s2);
        if (b1 == Ball.PURPLE || b2 == Ball.PURPLE) return Ball.PURPLE;
        if (b1 == Ball.GREEN || b2 == Ball.GREEN) return Ball.GREEN;
        return Ball.EMPTY;
    }

    private Ball detectSingle(NormalizedColorSensor s) {
        NormalizedRGBA c = s.getNormalizedColors();
        float r = c.red, g = c.green, b = c.blue;
        if (r + g + b < 0.07) return Ball.EMPTY;
        if (b > r * 1.35 && b > g * 1.25 && b > 0.12) return Ball.PURPLE;
        if (g > r * 1.15 && g > b * 1.15 && g > 0.15) return Ball.GREEN;
        return Ball.EMPTY;
    }

    private int findNextEmpty() {
        for (int i = 0; i < 3; i++) {
            int idx = (index + i) % 3;
            if (slots[idx] == Ball.EMPTY) return idx;
        }
        return -1;
    }

    private int findSlot(Ball t) {
        for (int i = 0; i < 3; i++) if (slots[i] == t) return i;
        return -1;
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
            sleep(15);
        }
        return 22;
    }

    // ---------------- INIT ----------------
    private void initHardware() {

        drivetrain = new CustomMecanumDrive(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        spinMotor = hardwareMap.get(DcMotor.class, "spinMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");

        hoodServo = hardwareMap.servo.get("hoodServo");
        flickServo = hardwareMap.servo.get("flickServo");

        intakeColor = hardwareMap.get(NormalizedColorSensor.class, "intakeColor");
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
        if (intakeColor instanceof SwitchableLight) ((SwitchableLight) intakeColor).enableLight(true);
        if (intakeColor2 instanceof SwitchableLight) ((SwitchableLight) intakeColor2).enableLight(true);

        hoodServo.setPosition(0.8);
        flickServo.setPosition(0.9);
    }

    private void resetSlots() {
        for (int i = 0; i < 3; i++) slots[i] = Ball.EMPTY;
    }
}