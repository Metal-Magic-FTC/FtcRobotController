package org.firstinspires.ftc.teamcode.decode.teleOp.comptwotests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.decode.teleOp.tests.CustomMecanumDrive;

@TeleOp(name = "!!SpinV3")
public class SpindexerV3 extends LinearOpMode {

    //launch motor - left bumper
    //flick servo - right trigger
    //spindexer intake - up arrow
    //spindexer shoot purple - b
    //spindexer shoot green - a
    private DcMotorEx spinMotor;
    private DcMotorEx launchMotor;
    private DcMotor intakeMotor;

    private CustomMecanumDrive drivetrain;

    Servo hoodServo;
    Servo flickServo;

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
    private int autoLaunchTarget = -1;

    private long flickStartTime = 0;
    private boolean flicking = false;

    private static final long FLICK_TIME_MS = 500; // 500 ms flick

    // ---- POST FLICK DELAY ----
    private boolean waitingAfterFlick = false;
    private long flickEndTime = 0;
    private static final long POST_FLICK_DELAY_MS = 250; // 100 ms delay after flick retract

    private double spinMotorSpeed = 0.5;

    // ---- COLOR SENSOR DELAY ----
    private boolean waitingToRotate = false;
    private long colorDetectedTime = 0;
    private static final long COLOR_DELAY_MS = 100; // 100 ms delay before spinning
    private int nextIndexAfterDelay = -1;

    // ---- BUTTON STATES ----

    // ---- GAMEPAD 2 FAILSAFES ----
    private boolean prev2A, prev2B;

    private double flickUp = 0.75;
    private double flickDown = 1;

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

        hoodServo.setPosition(0.83);
        flickServo.setPosition(flickDown);

        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            drivetrain.driveMecanum(strafe, drive, turn);

            //launch motor - left bumper
            //flick servo - right bumper
            //spindexer intake - up arrow (dpad)
            //spindexer shoot purple - b
            //spindexer shoot green - a
            //launch all three - left arrow (dpad)
            //reverse intake - left trigger
            intakePressed      = gamepad1.dpad_up && !prevX;
            aimGreenPressed    = gamepad1.a && !prevA;
            aimPurplePressed   = gamepad1.b && !prevB;
            shootPressed       = gamepad1.right_bumper; // && !prevB;
            runLaunch          = (gamepad1.left_bumper && !prevLeftBumper || gamepad2.left_bumper && !prev2LeftBumper) != runLaunch;
            intakePower        = ((gamepad1.right_trigger >= 0.3f && !prevRightTrigger) || (gamepad2.right_bumper && !prev2RightBumper))!= intakePower;
            intakePowerReverse = (gamepad1.left_trigger >= 0.3f && !prevLeftTrigger) != intakePowerReverse;
            launchAllPressed = gamepad1.dpad_left;

            nextIntake2 = gamepad2.dpad_down;

            prevA = gamepad1.a;
            prevX = gamepad1.x;
            prevY = gamepad1.y;
            prevB = gamepad1.b;
            prevLeftBumper = gamepad1.left_bumper;
            prev2LeftBumper = gamepad2.left_bumper;
            prevRightBumper = gamepad1.right_bumper;
            prevLeftTrigger = gamepad1.left_trigger >= 0.3F;
            prevRightTrigger = gamepad1.right_trigger >= 0.3F;
            prev2RightBumper = gamepad2.right_bumper;

            prevNextIntake2 = gamepad2.dpad_down;

            // ----- GAMEPAD 2 MANUAL COLOR OVERRIDE -----
            boolean manualGreen  = gamepad2.a && !prev2A;
            boolean manualPurple = gamepad2.b && !prev2B;

            prev2A = gamepad2.a;
            prev2B = gamepad2.b;

            if (nextIntake2 && !prevNextIntake2) {
                index++;
                index = index % 3;

                // normal intake
                intakeActive = true;
                waitingForBall = true;
                rotateToIndex(index);
            }

            if (intakePower) {
                intakeMotor.setPower(0);
            } else if (intakePowerReverse) {
                intakeMotor.setPower(0.8);
            } else {
                intakeMotor.setPower(-0.6);
            }

            if (waitingForBall && intakeActive && !spinMotor.isBusy()) {

                if (manualGreen || manualPurple) {
                    Ball forced = manualGreen ? Ball.GREEN : Ball.PURPLE;

                    slots[index] = forced;
                    waitingForBall = false;

                    int nextEmpty = findNextEmpty();
                    nextIndexAfterDelay = nextEmpty;
                    colorDetectedTime = System.currentTimeMillis();
                    waitingToRotate = true;
                }
            }

            // ----- MANUAL SPINDEXER CONTROL (HOLD X) -----
            if (gamepad2.x) {

                spinMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                double manualPower = 0.2*(gamepad2.right_trigger - gamepad2.left_trigger);

                spinMotor.setPower(manualPower);

                // Encoder reset while holding X + DPAD UP
                if (gamepad2.dpad_up) {
                    spinMotor.setPower(0);
                    spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    slots[0] = Ball.EMPTY;
                    slots[1] = Ball.EMPTY;
                    slots[2] = Ball.EMPTY;
                    index = 0;
                }

                // HARD EXIT so auto logic doesn't fight you
                telemetry.addLine("MANUAL SPINDEXER MODE");
                telemetry.update();
                continue;
            }

            if (launchAllPressed && !autoLaunching) {
                autoLaunching = true;
                autoLaunchTarget = findClosestLoaded();
                launchMotor.setPower(0.9); // spin to shooting speed
            }

            if (autoLaunching) {

                // No balls left - stop everything
                if (autoLaunchTarget == -1) {
                    autoLaunching = false;
                    launchMotor.setPower(0);
                    flickServo.setPosition(flickDown);
                    waitingAfterFlick = false;
                } else {

                    // Rotate to the current ball if not aligned
                    if (index != autoLaunchTarget && !spinMotor.isBusy() && !waitingAfterFlick) {
                        rotateToIndex(autoLaunchTarget);
                    }

                    // Once aligned and not flicking - flick servo out
                    if (!spinMotor.isBusy() && !flicking && !waitingAfterFlick) {
                        flickServo.setPosition(flickUp);
                        flickStartTime = System.currentTimeMillis();
                        flicking = true;
                    }

                    // Retract servo after FLICK_TIME_MS
                    if (flicking && System.currentTimeMillis() - flickStartTime >= FLICK_TIME_MS) {
                        flickServo.setPosition(flickDown);
                        flicking = false;

                        // Clear the slot
                        slots[index] = Ball.EMPTY;

                        // Start post-flick delay
                        waitingAfterFlick = true;
                        flickEndTime = System.currentTimeMillis();
                    }

                    // Wait POST_FLICK_DELAY_MS before moving to next ball
                    if (waitingAfterFlick && System.currentTimeMillis() - flickEndTime >= POST_FLICK_DELAY_MS) {
                        autoLaunchTarget = findClosestLoaded();
                        waitingAfterFlick = false;
                    }
                }
            }

            // intake
            if (intakePressed) {
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
            }

            // go to balls
            if (aimGreenPressed) {
                aimClosest(Ball.GREEN);
            }

            if (aimPurplePressed) {
                aimClosest(Ball.PURPLE);
            }

            // shoot

            if (shootPressed && !autoLaunching) {
                flickServo.setPosition(flickUp);
            } else if (!autoLaunching) {
                flickServo.setPosition(flickDown);
            }

            if (shootPressed && !autoLaunching) {
                if (slots[index] != Ball.EMPTY) {
                    // INSERT TS launcher code
                    slots[index] = Ball.EMPTY;
                }
            }

            if (runLaunch) {
                launchMotor.setVelocity(6000*Math.PI/30, AngleUnit.RADIANS);
            } else {
                launchMotor.setPower(0);
            }

            // spindexer logic (COLOR-BASED DETECTION)

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
                }
            }

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
            telemetry.addLine("Slots:");
            for (int i = 0; i < 3; i++) {
                telemetry.addData("Slot " + i, slots[i]);
            }
            NormalizedRGBA c = intakeColor.getNormalizedColors();
            telemetry.addData("R", "%.2f", c.red);
            telemetry.addData("G", "%.2f", c.green);
            telemetry.addData("B", "%.2f", c.blue);
            telemetry.addData("Sum", "%.2f", c.red + c.green + c.blue);
            telemetry.update();

            NormalizedRGBA c2 = intakeColor2.getNormalizedColors();
            telemetry.addData("R", "%.2f", c2.red);
            telemetry.addData("G", "%.2f", c2.green);
            telemetry.addData("B", "%.2f", c2.blue);
            telemetry.addData("Sum", "%.2f", c2.red + c2.green + c2.blue);

            telemetry.addData("Launch Velocity", launchMotor.getVelocity());
            telemetry.addData("Launch Power", launchMotor.getPower());

            telemetry.update();
        }
    }

    // ROTATION

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

    // COLOR SENSOR (WIDE MARGIN + FLOOR REJECTION)

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

    private void enableLight(NormalizedColorSensor s) {
        if (s instanceof SwitchableLight) {
            ((SwitchableLight) s).enableLight(true);
        }
    }

    public void initialize() {
        spinMotor = hardwareMap.get(DcMotorEx.class, "spinMotor");
        intakeColor = hardwareMap.get(NormalizedColorSensor.class, "intakeColor");
        intakeColor2 = hardwareMap.get(NormalizedColorSensor.class, "intakeColor2");

        spinMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        spinMotor.setDirection(DcMotorEx.Direction.REVERSE);

        intakeColor.setGain(gain);
        enableLight(intakeColor);

        intakeColor2.setGain(gain);
        enableLight(intakeColor2);

        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        launchMotor.setDirection(DcMotor.Direction.REVERSE); // same as TeleOp_Flick_Launch
        launchMotor.setPower(0);

        hoodServo = hardwareMap.servo.get("hoodServo");
        flickServo = hardwareMap.servo.get("flickServo");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        drivetrain = new CustomMecanumDrive(hardwareMap);

    }

}