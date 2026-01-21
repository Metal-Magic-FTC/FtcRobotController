package org.firstinspires.ftc.teamcode.decode.teleOp.actual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.decode.teleOp.tests.CustomMecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@TeleOp(name="! TeleV3 - run this")
public class TeleV3 extends LinearOpMode {

    // ============================================================
    //                  HARDWARE + CONSTANTS
    // ============================================================

    private CustomMecanumDrive drivetrain;

    private DcMotor intakeMotor;
    private DcMotor launchMotor;
    private DcMotor spinMotor;

    private Servo pivotServo;
    private Servo flickServo;

    private NormalizedColorSensor backColor, leftColor, rightColor;

    private final int[] POSITIONS = {0, 246, 496};
    private final int[] INTAKE_POSITIONS = {-373, -132, 127}; // {352, -115, 142};

    private float gain = 20;

    private enum BallColor { PURPLE, GREEN, EMPTY, UNKNOWN }

    private BallColor[] balls = new BallColor[3];
    private int index = 0;
    private int currentTarget = 0;

    // ============================================================
    //                 HOOD + FLICK TIMING
    // ============================================================

    private boolean flicking = false;
    private long flickStartTime = 0;
    private final long HOOD_DELAY = 100;   // ms before flick after pivot down
    private final long FLICK_DURATION = 500; // ms flick stays up

    private long limitPressTime = 0;
    private boolean limitDelayTriggered = false;
    private final long LIMIT_DELAY = 500; // ms

    private double launchSpeed = 0.95;

    // ============================================================
    //                              INIT
    // ============================================================

    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();
        initializeBallArray();

        boolean limitPressed = false;
        boolean limitWasPressed = false;

        waitForStart();

        while (opModeIsActive()) {

            // ============================================================
            //                 GAMEPAD INPUT COLLECTION
            // ============================================================

            boolean limitSwitchNew = false; //limitPressed && !limitWasPressed;

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            boolean launcherButton = gamepad1.right_trigger > 0.3;
            boolean flickButton = gamepad1.dpad_left;
            boolean intakeEvent = gamepad1.dpad_down;
            boolean resetAndScan = gamepad1.left_trigger > 0.3;

            boolean gp2ResetIndex = false;
            boolean gp2ScanAll = gamepad2.left_bumper;
            boolean gp2MoveSpin = false;
            boolean gpMarkUnknown = gamepad1.left_bumper;
            boolean gpLaunchBall = gamepad2.y;
            boolean stopIntake = gamepad1.y;

            boolean gp2Pos1 = gamepad2.dpad_up;
            boolean gp2Pos2 = gamepad2.dpad_left;
            boolean gp2Pos3 = gamepad2.dpad_down;

            boolean gpToPurple = gamepad1.b;
            boolean gpToGreen = gamepad1.a;
            boolean gpToEmpty = gamepad1.right_bumper;

            boolean gpSetPurple = false;
            boolean gpSetGreen = false;

            boolean speedHigh = gamepad1.dpad_up;
            boolean speedLow = gamepad1.dpad_down;

            boolean gp2ResetSpin = gamepad2.a;
            double motorPowerReset = 0.2 * (gamepad2.right_trigger - gamepad2.left_trigger);

            if (gamepad2.right_bumper) {
                initializeBallArray();
            }

            if (gamepad2.y) {
                intakeMotor.setPower(-0.5);
            }

            if (gamepad2.x) {
                launchMotor.setPower(-0.1);
            }
            if (gamepad2.b) {
                launchMotor.setPower(0);
            }

            // ============================================================
            //                        SUBSYSTEM HANDLING
            // ============================================================

            if (speedLow) {
                launchSpeed = 0.95;
            }

            if (speedHigh) {
                launchSpeed = 1.00;
            }

            drivetrain.driveMecanum(strafe, drive, turn);

            handleIntake(stopIntake);
            handleLauncher(launcherButton);

            updateFlickPivot(flickButton);

            if (gp2ResetSpin) {

                spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                spinMotor.setPower(motorPowerReset);

            } else {

                handleBallCommands(
                        gp2ResetIndex, gpMarkUnknown, gp2ScanAll,
                        gpLaunchBall, gpToPurple, gpToGreen, gpToEmpty,
                        gpSetPurple, gpSetGreen,
                        intakeEvent, resetAndScan,
                        gp2Pos1, gp2Pos2, gp2Pos3
                );

            }

            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            // ============================================================
            //                           TELEMETRY
            // ============================================================
            telemetry.addData("0", balls[0].name());
            telemetry.addData("1", balls[1].name());
            telemetry.addData("2", balls[2].name());
            telemetry.update();

        }
    }

    // ============================================================
    //                       SUBSYSTEMS
    // ============================================================

    /** INTAKE **/
    private void handleIntake(boolean active) {
        intakeMotor.setPower(active ? 0.0 : 1);
    }

    /** LAUNCHER + HOOD **/
    private void handleLauncher(boolean active) {
        if (active) {
            launchMotor.setPower(1);
        } else {
            launchMotor.setPower(0);
            pivotServo.setPosition(0.6);   // hood idle
        }
    }

    /** Flick + Hood Non-blocking **/
    private void updateFlickPivot(boolean flickButton) {
        if (flickButton && !flicking) {
            // Start sequence: hood goes down first
            pivotServo.setPosition(0.735);
            flicking = true;
            flickStartTime = System.currentTimeMillis();
        }

        if (flicking) {
            long elapsed = System.currentTimeMillis() - flickStartTime;

            // Flick fires after HOOD_DELAY
            if (elapsed >= HOOD_DELAY && elapsed < HOOD_DELAY + FLICK_DURATION) {
                flickServo.setPosition(0.22);
            }
            // End flick after FLICK_DURATION
            else if (elapsed >= HOOD_DELAY + FLICK_DURATION) {
                flickServo.setPosition(0);
                balls[index] = BallColor.EMPTY;
                flicking = false;
                pivotServo.setPosition(0.6); // reset hood to idle
            }
        }
    }

    /** BALL COMMANDS **/
    private void handleBallCommands(
            boolean resetIdx,
            boolean markUnknown,
            boolean scanAll,
            boolean launchBall,
            boolean nextPurple,
            boolean nextGreen,
            boolean nextEmpty,
            boolean setPurple,
            boolean setGreen,
            boolean intakeEvent,
            boolean resetAndScan,
            boolean spin1,
            boolean spin2,
            boolean spin3
    ) {

        if (intakeEvent) balls[index] = BallColor.UNKNOWN;

        if (resetAndScan) {
            index = 0;
            goToIndex(index, POSITIONS);
            scanAllBalls();
        }

        if (resetIdx) {
            index = 0;
            goToIndex(index, POSITIONS);
        }

        if (markUnknown) balls[index] = BallColor.UNKNOWN;
        if (scanAll) scanAllBalls();

        if (launchBall) launchBallAt(index);

        if (nextPurple) {
            index = findClosestOf(BallColor.PURPLE);
            goToIndex(index, POSITIONS);
        }

        if (nextGreen) {
            index = findClosestOf(BallColor.GREEN);
            goToIndex(index, POSITIONS);
        }

        if (nextEmpty) {
            index = findClosestOf(BallColor.EMPTY);
            goToIndex(index, INTAKE_POSITIONS);
        }

        if (setPurple) balls[index] = BallColor.PURPLE;
        if (setGreen)  balls[index] = BallColor.GREEN;

        if (spin1) {
            goToIndex(0, POSITIONS);
        }

        if (spin2) {
            goToIndex(1, POSITIONS);
        }

        if (spin3) {
            goToIndex(2, POSITIONS);
        }
    }

    /** Launch ball + update array **/
    private void launchBallAt(int idx) {
        if (balls[idx] == BallColor.EMPTY) return;

        // Use the same flick + hood timing if desired
        pivotServo.setPosition(0.735);
        flickServo.setPosition(0.22);
        sleep(200);
        flickServo.setPosition(0);
        balls[idx] = BallColor.EMPTY;
        pivotServo.setPosition(0.6);
    }

    // ============================================================
    //                  SPINDEXER + COLOR HELPERS
    // ============================================================

    private void goToIndex(int newIndex, int[] table) {
        int target = table[newIndex];
        currentTarget = target;

        spinMotor.setTargetPosition(target);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(0.2);
    }

    /** Circular nearest search **/
    private int findClosestOf(BallColor target) {
        for (int offset = 0; offset < balls.length; offset++) {
            int right = (index + offset) % balls.length;
            int left  = (index - offset + balls.length) % balls.length;

            if (balls[right] == target) return right;
            if (balls[left] == target) return left;
        }
        return index;
    }

    private void scanAllBalls() {
        balls[0] = detectColor(backColor);
        balls[1] = detectColor(rightColor);
        balls[2] = detectColor(leftColor);
    }

    private BallColor detectColor(NormalizedColorSensor sensor) {
        NormalizedRGBA c = sensor.getNormalizedColors();
        float r = c.red, g = c.green, b = c.blue;
        float tol = 0.20f;

        if (b > r*(1+tol) && b > g*(1+tol)) return BallColor.PURPLE;
        if (g > r*(1+tol) && g > b*(1+tol)) return BallColor.GREEN;

        if (r > 0.01 || g > 0.01 || b > 0.01) return BallColor.EMPTY;
        return BallColor.EMPTY;
    }

    // ============================================================
    //                           INIT
    // ============================================================

    private void initializeBallArray() {
        balls[0] = BallColor.EMPTY;
        balls[1] = BallColor.EMPTY;
        balls[2] = BallColor.EMPTY;
    }

    private void initializeHardware() {
        initMotorAndServos();
        initColorSensors();
        drivetrain = new CustomMecanumDrive(hardwareMap);
    }

    private void initMotorAndServos() {
        pivotServo = hardwareMap.servo.get("launchServo");
        flickServo = hardwareMap.servo.get("flickServo");
        launchMotor = hardwareMap.dcMotor.get("launchMotor");

        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        spinMotor = hardwareMap.dcMotor.get("spinMotor");
        spinMotor.setTargetPosition(0);
        //spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void initColorSensors() {
        backColor  = hardwareMap.get(NormalizedColorSensor.class, "backColor");
        leftColor  = hardwareMap.get(NormalizedColorSensor.class, "leftColor");
        rightColor = hardwareMap.get(NormalizedColorSensor.class, "rightColor");

        backColor.setGain(gain);
        leftColor.setGain(gain);
        rightColor.setGain(gain);

        enableLight(backColor);
        enableLight(leftColor);
        enableLight(rightColor);
    }

    private void enableLight(NormalizedColorSensor s) {
        if (s instanceof SwitchableLight) ((SwitchableLight) s).enableLight(true);
    }
}