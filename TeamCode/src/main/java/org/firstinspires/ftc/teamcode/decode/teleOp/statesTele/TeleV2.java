package org.firstinspires.ftc.teamcode.decode.teleOp.statesTele;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.decode.teleOp.tests.CustomMecanumDrive;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.teleOp.states.tests.limelightV2.FusedPose;

@TeleOp(name = "!!!!!!!!! STATES TELEOP V2")
public class TeleV2 extends LinearOpMode {

    //launch motor - left bumper
    //flick servo - right trigger
    //spindexer intake - up arrow
    //spindexer shoot purple - b
    //spindexer shoot green - a
    private DcMotor spinMotor;
    private DcMotorEx launchMotor;
    private DcMotorEx flickMotor;
    private DcMotor intakeMotor;

    private Follower follower;
    private Pose savePose;
    private boolean oneTime;

    private CustomMecanumDrive drivetrain;

    private NormalizedColorSensor intakeColor;
    private NormalizedColorSensor intakeColor2;

    private static final int[] OUTTAKE_POS = {504, 2, 252};
    private static final int[] INTAKE_POS  = {125, 375, 625};

    private enum Ball { EMPTY, PURPLE, GREEN }
    private Ball[] slots = {Ball.EMPTY, Ball.EMPTY, Ball.EMPTY};

    private int index = 0;
    private boolean intakeActive = false;
    private boolean waitingForBall = false;

    private float gain = 20;

    // ---- RISING EDGE DETECTORS ----
    private boolean
            prevA,
            prevX,
            prevY,
            prevB,
            prevLeftBumper,
            prev2LeftBumper,
            prevRightBumper,
            prevLeftTrigger,
            prevRightTrigger,
            prev2RightBumper;

    // ---- AUTO LAUNCH ALL ----
    private boolean autoLaunching = false;

    // new auto launch
    private int shootAllTargetPos = 0;

    private double spinMotorSpeed = 0.38;

    // ---- COLOR SENSOR DEBOUNCE / SETTLE ----
    // Root cause of the "false ball" / "skips a real ball" bugs: the old code
    // made a decision off a single sensor frame the instant the spindexer
    // stopped moving - the worst possible moment (residual vibration, ball
    // still settling in the tube). Now we (1) wait a short settle window
    // after arriving at a slot before sampling at all, and (2) require
    // several consecutive matching reads before accepting a color.
    private boolean waitingToRotate = false;
    private long colorDetectedTime = 0;
    private static final long COLOR_DELAY_MS = 80; // delay after confirmed detection before rotating
    private int nextIndexAfterDelay = -1;

    private static final long SLOT_SETTLE_MS = 120;      // ignore sensor for this long after arriving
    private static final int  DEBOUNCE_SAMPLES_REQUIRED = 4; // consecutive matching reads to accept
    private boolean armedForSample = false;
    private long arrivedAtSlotTime = 0;
    private Ball debounceCandidate = Ball.EMPTY;
    private int debounceCount = 0;

    // ---- BUTTON STATES ----
    private boolean prev2A, prev2B;

    private double targetVelocity = 1800;

    private static double TARGET_X = 150;
    private static double TARGET_Y = 137;

    public static Pose startPose = new Pose(
            109,
            130,
            Math.toRadians(180)
    );

    double targetHeading;
    PIDFController controller;
    boolean headingLock;

    FusedPose fusedPose;

    private boolean
            intakePressed,
            aimGreenPressed,
            aimPurplePressed,
            shootPressed,
            runLaunch,
            intakePower,
            intakePowerReverse,
            launchAllPressed,
            nextIntake2,
            prevNextIntake2;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        flickMotor.setPower(0);

        while (opModeIsActive()) {
            follower.update();
            fusedPose.update();
            Pose limelightPose = fusedPose.getRobotPose(true); // CONVERTED pose
            if (limelightPose != null) {
                follower.setPose(limelightPose);
            }

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            intakePressed      = gamepad1.dpad_up;
            aimGreenPressed    = gamepad1.a && !prevA;
            aimPurplePressed   = gamepad1.b && !prevB;
            shootPressed       = gamepad1.right_bumper;
            runLaunch          = (gamepad1.left_bumper && !prevLeftBumper || gamepad2.left_bumper && !prev2LeftBumper) != runLaunch;
            intakePower        = ((gamepad1.right_trigger >= 0.3f && !prevRightTrigger) || (gamepad2.right_bumper && !prev2RightBumper))!= intakePower;
            intakePowerReverse = (gamepad1.x && !prevX) != intakePowerReverse;
            launchAllPressed = gamepad1.dpad_left;

            prevA = gamepad1.a;
            prevY = gamepad1.y;
            prevB = gamepad1.b;
            prevX = gamepad1.x;
            prevLeftBumper = gamepad1.left_bumper;
            prev2LeftBumper = gamepad2.left_bumper;
            prevRightBumper = gamepad1.right_bumper;
            prevLeftTrigger = gamepad1.left_trigger >= 0.3F;
            prevRightTrigger = gamepad1.right_trigger >= 0.3F;
            prev2RightBumper = gamepad2.right_bumper;

            if (gamepad2.dpad_left) {
                targetVelocity = 1600;
            }
            if (gamepad2.dpad_down) {
                targetVelocity = 1800;
            }
            if (gamepad2.dpad_right) {
                targetVelocity = 6000;
            }

            // ----- GAMEPAD 2 MANUAL COLOR OVERRIDE -----
            boolean manualGreen  = gamepad2.a && !prev2A;
            boolean manualPurple = gamepad2.b && !prev2B;

            prev2A = gamepad2.a;
            prev2B = gamepad2.b;

            boolean idle =
                    Math.abs(drive) < 0.05 &&
                            Math.abs(strafe) < 0.05 &&
                            Math.abs(turn) < 0.05 && (gamepad1.left_bumper || gamepad2.left_stick_button);

            telemetry.addData("Move: ", drive + strafe + turn);
            if (idle) {
                if (oneTime) {
                    follower.update();
                    savePose = follower.getPose();
                    oneTime = false;
                    follower.holdPoint(savePose);
                }
                telemetry.addLine("Holding Position");
                follower.update();
            } else {
                drivetrain.driveMecanum(strafe, drive, turn);
                oneTime = true;
                telemetry.addLine("Manual Drive");
            }

            updateGoalHeading();

            headingLock = gamepad1.left_bumper;

            controller.setCoefficients(follower.constants.coefficientsHeadingPIDF);
            double v = getHeadingError();
            controller.updateError(v);

            double otherV = 0;

            if (headingLock) {
                otherV = controller.run();
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, otherV, true);
            } else
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

            if (nextIntake2 && !prevNextIntake2) {
                index++;
                index = index % 3;

                intakeActive = true;
                waitingForBall = true;
                rotateToIndex(index);
            }

            if (autoLaunching) {
                intakeMotor.setPower(0);
            } else {
                if (intakePower) {
                    intakeMotor.setPower(0);
                } else if (intakePowerReverse) {
                    intakeMotor.setPower(0.8);
                } else {
                    intakeMotor.setPower(-0.6);
                }
            }

            if (autoLaunching) {
                spinMotorSpeed = 0.35;
            } else {
                spinMotorSpeed = 0.38;
            }

            // ----- MANUAL COLOR OVERRIDE (still instant - explicit driver input, no debounce needed) -----
            if (waitingForBall && intakeActive && !spinMotor.isBusy()) {
                if (manualGreen || manualPurple) {
                    Ball forced = manualGreen ? Ball.GREEN : Ball.PURPLE;
                    acceptBall(forced);
                }
            }

            // ----- MANUAL SPINDEXER CONTROL (HOLD X on gamepad2) -----
            if (gamepad2.x) {

                spinMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                double manualPower = 0.2*(gamepad2.right_trigger - gamepad2.left_trigger);

                spinMotor.setPower(manualPower);

                if (gamepad2.dpad_up) {
                    spinMotor.setPower(0);
                    spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    slots[0] = Ball.EMPTY;
                    slots[1] = Ball.EMPTY;
                    slots[2] = Ball.EMPTY;
                    index = 0;
                }

                telemetry.addLine("MANUAL SPINDEXER MODE");
                telemetry.update();
                continue;
            }

            if (launchAllPressed && !autoLaunching) {
                autoLaunching = true;
                intakeActive = false;

                int current = spinMotor.getCurrentPosition();
                int currentMod = ((current % 750) + 750) % 750;

                int targetInRev = OUTTAKE_POS[2];
                if (targetInRev <= currentMod) {
                    targetInRev += 750;
                }

                shootAllTargetPos = current - currentMod + targetInRev;

                spinMotor.setTargetPosition(shootAllTargetPos);
                spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spinMotor.setPower(0.35);

                flickMotor.setPower(1);
                launchMotor.setVelocity(targetVelocity);
            }

            if (autoLaunching) {
                launchMotor.setVelocity(targetVelocity);

                if (!spinMotor.isBusy()) {
                    flickMotor.setPower(0);

                    slots[0] = Ball.EMPTY;
                    slots[1] = Ball.EMPTY;
                    slots[2] = Ball.EMPTY;

                    index = 2;
                    autoLaunching = false;
                }
            }

            // intake
            if (intakePressed) {
                int nextEmpty = findNextEmpty();

                if (nextEmpty != -1) {
                    intakeActive = true;
                    waitingForBall = true;
                    rotateToIndex(nextEmpty);
                } else {
                    intakeActive = false;
                    waitingForBall = false;
                    rotateToIndex(0);
                }
            }

            if (aimGreenPressed) {
                aimClosest(Ball.GREEN);
            }

            if (aimPurplePressed) {
                aimClosest(Ball.PURPLE);
            }

            if (shootPressed && !autoLaunching) {
                flickMotor.setPower(1);
            } else if (!autoLaunching) {
                flickMotor.setPower(0);
            }

            if (shootPressed && !autoLaunching) {
                if (slots[index] != Ball.EMPTY) {
                    slots[index] = Ball.EMPTY;
                }
            }

            if (runLaunch) {
                launchMotor.setVelocity(targetVelocity);
            } else {
                launchMotor.setVelocity(targetVelocity);
            }

            // ---- SPINDEXER AUTO COLOR DETECTION (settled + debounced) ----
            intake();

            // after COLOR_DELAY_MS, rotate if needed
            if (waitingToRotate) {
                if (System.currentTimeMillis() - colorDetectedTime >= COLOR_DELAY_MS) {
                    if (nextIndexAfterDelay != -1) {
                        rotateToIndex(nextIndexAfterDelay);
                        waitingForBall = true; // continue intake
                    } else {
                        intakeActive = false;
                        rotateToIndex(0); // back to home
                    }
                    waitingToRotate = false; // reset
                }
            }

            telemetry.addData("Index", index);
            telemetry.addData("Position", spinMotor.getCurrentPosition());
            telemetry.addData("Armed for sample", armedForSample);
            telemetry.addData("Debounce candidate", debounceCandidate + " x" + debounceCount);
            telemetry.addLine("Slots:");
            for (int i = 0; i < 3; i++) {
                telemetry.addData("Slot " + i, slots[i]);
            }
            NormalizedRGBA c = intakeColor.getNormalizedColors();
            telemetry.addData("R", "%.2f", c.red);
            telemetry.addData("G", "%.2f", c.green);
            telemetry.addData("B", "%.2f", c.blue);
            telemetry.addData("Sum", "%.2f", c.red + c.green + c.blue);

            NormalizedRGBA c2 = intakeColor2.getNormalizedColors();
            telemetry.addData("R2", "%.2f", c2.red);
            telemetry.addData("G2", "%.2f", c2.green);
            telemetry.addData("B2", "%.2f", c2.blue);
            telemetry.addData("Sum2", "%.2f", c2.red + c2.green + c2.blue);

            telemetry.addLine("----------------------------");
            telemetry.addData("Launch Velocity", launchMotor.getVelocity());
            telemetry.addData("Launch Power", launchMotor.getPower());
            telemetry.addData("launch target velocity", targetVelocity);

            telemetry.update();
        }
    }

    public double getHeadingError() {
        double headingError = MathFunctions.getTurnDirection(follower.getPose().getHeading(), targetHeading) * MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), targetHeading);
        return headingError;
    }

    public void updateGoalHeading() {
        Pose robotPos = follower.getPose();
        double angleToTarget = MathFunctions.normalizeAngle(Math.atan2(
                TARGET_Y - robotPos.getY(),
                TARGET_X - robotPos.getX()
        ));
        targetHeading = angleToTarget;
    }

    // ---------------- SPINDEXER ----------------

    private void rotateToIndex(int target) {
        index = target;
        int base = intakeActive ? INTAKE_POS[target] : OUTTAKE_POS[target];
        int targetPos = closestModular(base, spinMotor.getCurrentPosition());

        spinMotor.setTargetPosition(targetPos);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(spinMotorSpeed);

        // Any time we start a new move, invalidate the settle/debounce state -
        // we are no longer sitting still at a confirmed slot.
        armedForSample = false;
        debounceCandidate = Ball.EMPTY;
        debounceCount = 0;
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

    private int findNextEmpty() {
        for (int i = 0; i < 3; i++) {
            int idx = (index + i) % 3;
            if (slots[idx] == Ball.EMPTY) return idx;
        }
        return -1;
    }

    private int findClosestLoaded() {
        for (int i = 0; i < 3; i++) {
            int idx = (index + i) % 3;
            if (slots[idx] != Ball.EMPTY) return idx;
        }
        return -1;
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

    // Shared acceptance path for both auto color-detect and manual override.
    private void acceptBall(Ball detected) {
        slots[index] = detected;
        waitingForBall = false;
        armedForSample = false;

        int nextEmpty = findNextEmpty();
        nextIndexAfterDelay = nextEmpty;
        colorDetectedTime = System.currentTimeMillis();
        waitingToRotate = true;
    }

    // ---- SETTLED + DEBOUNCED AUTO DETECTION ----
    // Called every loop. Only samples once the spindexer has been stopped and
    // settled for SLOT_SETTLE_MS, and only accepts a color once it has read
    // the same non-empty result DEBOUNCE_SAMPLES_REQUIRED times in a row.
    // This is what fixes both false-positive-when-empty and skipping a real
    // ball: a single noisy/glare frame can no longer decide anything on its
    // own, and we never even look during the first moments after arriving,
    // when vibration and ball settling make readings least trustworthy.
    private void intake() {

        if (!(waitingForBall && intakeActive && !spinMotor.isBusy() && !waitingToRotate)) {
            return;
        }

        if (!armedForSample) {
            // Just confirmed stopped at this slot - start the settle timer,
            // don't trust any reading yet.
            armedForSample = true;
            arrivedAtSlotTime = System.currentTimeMillis();
            debounceCandidate = Ball.EMPTY;
            debounceCount = 0;
            return;
        }

        if (System.currentTimeMillis() - arrivedAtSlotTime < SLOT_SETTLE_MS) {
            return; // still settling, don't sample yet
        }

        Ball detected = detectColor(intakeColor, intakeColor2);

        if (detected == Ball.EMPTY) {
            // Any empty read resets the streak - a real ball should read
            // consistently, not flicker between colored/empty.
            debounceCandidate = Ball.EMPTY;
            debounceCount = 0;
            return;
        }

        if (detected == debounceCandidate) {
            debounceCount++;
        } else {
            debounceCandidate = detected;
            debounceCount = 1;
        }

        if (debounceCount >= DEBOUNCE_SAMPLES_REQUIRED) {
            acceptBall(detected);
        }
    }

    // COLOR SENSOR (WIDE MARGIN + FLOOR REJECTION)

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

    private void enableLight(NormalizedColorSensor s) {
        if (s instanceof SwitchableLight) {
            ((SwitchableLight) s).enableLight(true);
        }
    }

    public void initialize() {
        spinMotor = hardwareMap.get(DcMotor.class, "spinMotor");
        intakeColor = hardwareMap.get(NormalizedColorSensor.class, "intakeColor");
        intakeColor2 = hardwareMap.get(NormalizedColorSensor.class, "intakeColor2");

        follower = Constants.createFollower(hardwareMap);
        follower.setDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.15, 0.001, 0.00001, 0.6, 0.003));
        follower.setStartingPose(startPose == null ? new Pose() : startPose);
        follower.update();

        follower.startTeleopDrive(true);
        follower.update();

        follower.updateDrivetrain();

        fusedPose = new FusedPose(hardwareMap, startPose);

        targetHeading = Math.toRadians(180); // Radians
        controller = new PIDFController(follower.constants.coefficientsHeadingPIDF);
        headingLock = false;

        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeColor.setGain(gain);
        enableLight(intakeColor);

        intakeColor2.setGain(gain);
        enableLight(intakeColor2);

        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        launchMotor.setDirection(DcMotor.Direction.FORWARD); // same as TeleOp_Flick_Launch

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(300, 0, 0, 12.93);
        launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        flickMotor = hardwareMap.get(DcMotorEx.class, "flickMotor");
        flickMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        drivetrain = new CustomMecanumDrive(hardwareMap);
    }
}