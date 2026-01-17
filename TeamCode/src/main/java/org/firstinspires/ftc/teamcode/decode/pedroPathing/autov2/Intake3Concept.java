package org.firstinspires.ftc.teamcode.decode.pedroPathing.autov2;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

@Autonomous(name = "Intake 3 in a row", group = "Examples")
public class Intake3Concept extends OpMode {

    // ---------------- SPINDEXER STATE ----------------
    private int index = 0;
    private enum Ball { EMPTY, PURPLE, GREEN }
    private Ball[] slots = {Ball.EMPTY, Ball.EMPTY, Ball.EMPTY};

    // ---------------- HARDWARE ----------------
    private DcMotor spinMotor;
    private DcMotor intakeMotor;
    private NormalizedColorSensor intakeColor;
    private NormalizedColorSensor intakeColor2;

    // ---------------- SPINDEXER CONSTANTS ----------------
    private static final int[] OUTTAKE_POS = {500, 0, 250};
    private static final int[] INTAKE_POS  = {125, 375, 625};
    private static final int SPIN_TOLERANCE_TICKS = 5;
    private static final int SPINDEXER_FULL_ROTATION = 750;

    private double spinMotorSpeed = 0.38;
    private float colorSensorGain = 20;
    private int lastSpinTarget = 0;

    // ---------------- INTAKE STATE ----------------
    private boolean intakeActive = false;

    // ---------------- PATH FOLLOWING ----------------
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // ---------------- POSES ----------------
    private final Pose startPose = new Pose(0, 0, Math.toRadians(90));
    private final Pose pickup1Pose = new Pose(0, 5, Math.toRadians(90));
    private final Pose pickup2Pose = new Pose(0, 12, Math.toRadians(90));
    private final Pose pickup3Pose = new Pose(0, 17, Math.toRadians(90));

    // ---------------- PATHS ----------------
    private PathChain intakeAllThree;

    // ================== PATH BUILDING ==================
    public void buildPaths() {
        intakeAllThree = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pickup3Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup3Pose.getHeading())

                // At 25% of path (near ball 1) - check for ball and handle intake
                .addParametricCallback(0.25, () -> {
                    handleBallIntake();
                })

                // At 50% of path (near ball 2) - check for ball and handle intake
                .addParametricCallback(0.50, () -> {
                    handleBallIntake();
                })

                // At 75% of path (near ball 3) - check for ball and handle intake
                .addParametricCallback(0.75, () -> {
                    handleBallIntake();
                })

                .build();
    }

    // ================== INTAKE HANDLING (called from callbacks) ==================
    private void handleBallIntake() {
        // Detect if there's a ball
        Ball detected = detectColor(intakeColor, intakeColor2);

        if (detected != Ball.EMPTY) {
            // Pause path following immediately
            follower.pausePathFollowing();

            // Store ball in current slot
            slots[index] = detected;

            // Find next empty slot
            int nextEmpty = findNextEmpty();

            if (nextEmpty != -1) {
                // Rotate spindexer to next slot
                rotateToIndex(nextEmpty);
            } else {
                // All slots full - stop intake
                intakeActive = false;
                intakeMotor.setPower(0);
            }
        }
    }

    // ================== STATE MACHINE ==================
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start intake path
                intakeActive = true;
                intakeMotor.setPower(-0.6);
                rotateToIndex(0);
                follower.followPath(intakeAllThree, true);
                setPathState(1);
                break;

            case 1: // Following path, handling intake via callbacks
                // Check if spindexer has finished rotating while paused
                if (!follower.isBusy()) {
                    // Path complete
                    setPathState(2);
                } else if (isSpindexerAtTarget() && follower.isBusy()) {
                    // Spindexer done rotating, resume path
                    follower.resumePathFollowing();
                }
                break;

            case 2: // Path complete
                intakeActive = false;
                intakeMotor.setPower(0);
                // Done - could transition to shooting state here
                break;
        }
    }

    // ================== MAIN LOOP ==================
    @Override
    public void loop() {
        // Update follower
        follower.update();

        // Run state machine
        autonomousPathUpdate();

        // Telemetry
        telemetry.addData("Path State", pathState);
//        telemetry.addData("Path Paused", follower.isPathFollowingPaused());
        telemetry.addData("Current Index", index);
        telemetry.addData("Slots", slots[0] + ", " + slots[1] + ", " + slots[2]);
        telemetry.addData("Spindexer Pos", spinMotor.getCurrentPosition());
        telemetry.addData("Spindexer Target", lastSpinTarget);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    // ================== SPINDEXER METHODS ==================
    private void rotateToIndex(int target) {
        index = target;
        int base = intakeActive ? INTAKE_POS[target] : OUTTAKE_POS[target];
        int targetPos = closestModular(base, spinMotor.getCurrentPosition());

        lastSpinTarget = targetPos;

        spinMotor.setTargetPosition(targetPos);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(spinMotorSpeed);
    }

    private int closestModular(int mod, int current) {
        int best = mod;
        int minDiff = Integer.MAX_VALUE;
        for (int k = -2; k <= 2; k++) {
            int candidate = mod + SPINDEXER_FULL_ROTATION * k;
            int diff = Math.abs(candidate - current);
            if (diff < minDiff) {
                minDiff = diff;
                best = candidate;
            }
        }
        return best;
    }

    private boolean isSpindexerAtTarget() {
        int error = Math.abs(spinMotor.getCurrentPosition() - lastSpinTarget);
        return error <= SPIN_TOLERANCE_TICKS;
    }

    private int findNextEmpty() {
        for (int i = 0; i < 3; i++) {
            int idx = (index + i) % 3;
            if (slots[idx] == Ball.EMPTY) return idx;
        }
        return -1;
    }

    private void resetSlots() {
        for (int i = 0; i < 3; i++) slots[i] = Ball.EMPTY;
    }

    // ================== COLOR DETECTION ==================
    private Ball detectColor(NormalizedColorSensor sensor1, NormalizedColorSensor sensor2) {
        Ball ball1 = detectSingleSensor(sensor1);
        Ball ball2 = detectSingleSensor(sensor2);

        if (ball1 == Ball.PURPLE || ball2 == Ball.PURPLE) return Ball.PURPLE;
        if (ball1 == Ball.GREEN  || ball2 == Ball.GREEN)  return Ball.GREEN;

        return Ball.EMPTY;
    }

    private Ball detectSingleSensor(NormalizedColorSensor sensor) {
        NormalizedRGBA c = sensor.getNormalizedColors();
        float r = c.red, g = c.green, b = c.blue;

        float total = r + g + b;
        if (total < 0.07f) return Ball.EMPTY;

        if (b > r * 1.35f && b > g * 1.25f && b > 0.12f) {
            return Ball.PURPLE;
        }

        if (g > r * 1.15f && g > b * 1.15f && g > 0.15f) {
            return Ball.GREEN;
        }

        return Ball.EMPTY;
    }

    private void enableLight(NormalizedColorSensor s) {
        if (s instanceof SwitchableLight) {
            ((SwitchableLight) s).enableLight(true);
        }
    }

    // ================== INIT ==================
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        initHardware();
        buildPaths();
        resetSlots();
    }

    private void initHardware() {
        // Spindexer motor
        spinMotor = hardwareMap.get(DcMotor.class, "spinMotor");
        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinMotor.setDirection(DcMotor.Direction.REVERSE);

        // Intake motor
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Color sensors
        intakeColor = hardwareMap.get(NormalizedColorSensor.class, "intakeColor");
        intakeColor2 = hardwareMap.get(NormalizedColorSensor.class, "intakeColor2");

        intakeColor.setGain(colorSensorGain);
        enableLight(intakeColor);

        intakeColor2.setGain(colorSensorGain);
        enableLight(intakeColor2);
    }

    @Override
    public void init_loop() {
    }

    // ================== START ==================
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    // ================== STOP ==================
    @Override
    public void stop() {
        intakeMotor.setPower(0);
        spinMotor.setPower(0);
    }
}