package org.firstinspires.ftc.teamcode.decode.teleOp.actual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.decode.teleOp.CustomMecanumDrive;

@TeleOp
public class TeleV1 extends LinearOpMode {

    private CustomMecanumDrive drivetrain;

    DcMotor intakeMotor = null;
    DcMotor launchMotor = null;
    DcMotor spinMotor = null;

    Servo pivotServo = null;

    Servo flickServo = null;

    int spinPosition;

    boolean leftWas = false;
    boolean leftIs = false;
    boolean rightWas = false;
    boolean rightIs = false;

    // 3 positions
    int[] POSITIONS = {0, 250, 500};
    int[] INTAKE_POSITIONS = {352, -200, 100};

    ballColors[] balls;

    int index = 0;
    int currentTarget = 0;

    public enum ballColors {
        PURPLE,
        GREEN,
        EMPTY
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize(); // initializing everything

        balls = new ballColors[3];
        balls[0] = ballColors.EMPTY;
        balls[1] = ballColors.EMPTY;
        balls[2] = ballColors.EMPTY;

        waitForStart(); // waiting until driver clicks play button

        spinPosition = 0;

        boolean spinControlIs = false;
        boolean spinControlWas = false;

        //pivotServo.setPosition(0.77);

        while (opModeIsActive()) { // duration of opMode

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            //drivetrain.driveMecanum(strafe, drive, turn); // creates mecanum drivetrain

            boolean intakeMotorControl = gamepad1.right_bumper;
            boolean launchControl = gamepad1.left_trigger >= 0.3;
            boolean launchControlReversed = gamepad1.right_trigger >= 0.3;

            boolean flickControl = gamepad1.a;

            // just for testing
            if (gamepad2.a) {
                balls[0] = ballColors.PURPLE;
                balls[1] = ballColors.GREEN;
                balls[2] = ballColors.PURPLE;
            }

            // Go to closest PURPLE
            if (gamepad2.b) {
                int targetIndex = findClosestColor(ballColors.PURPLE, index);
                index = targetIndex;
                currentTarget = POSITIONS[index];

                spinMotor.setTargetPosition(currentTarget);
                spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spinMotor.setPower(0.7);
            }

            // Go to closest GREEN
            if (gamepad2.x) {
                int targetIndex = findClosestColor(ballColors.GREEN, index);
                index = targetIndex;
                currentTarget = POSITIONS[index];

                spinMotor.setTargetPosition(currentTarget);
                spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spinMotor.setPower(0.7);
            }

            // Launch the ball at the current index when Y is pressed
            if (gamepad2.y) {

                // Only launch if ball is not EMPTY
                if (balls[index] != ballColors.EMPTY) {

                    // Flick the servo to shoot
                    flickServo.setPosition(0.6);
                    sleep(200);
                    flickServo.setPosition(1);

                    // Spin the launcher motor briefly
                    launchMotor.setPower(1);
                    sleep(300);
                    launchMotor.setPower(0);

                    // After launching, mark the slot empty
                    balls[index] = ballColors.EMPTY;
                }

            }

            // Move to closest EMPTY position for intake
            if (gamepad2.dpad_up) {
                int targetIndex = findClosestEmpty(index);
                index = targetIndex;
                currentTarget = INTAKE_POSITIONS[index];

                spinMotor.setTargetPosition(currentTarget);
                spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spinMotor.setPower(0.7);
            }

            // TESTING: Set color at current index
            if (gamepad2.dpad_right) {
                balls[index] = ballColors.PURPLE;
            }

            if (gamepad2.dpad_left) {
                balls[index] = ballColors.GREEN;
            }

            intakeMotor(intakeMotorControl);
            launch(0.34, launchControl, launchControlReversed, 1, flickControl);

            spinControlIs = gamepad1.dpad_right;

            spindexer(spinControlIs, spinControlWas);

            spinControlWas = gamepad1.dpad_right;

            telemetry.update();

            telemetry.addData("Servo pos:", pivotServo.getPosition());
            telemetry.addData("spindexer button position", spinPosition);
            telemetry.addData("spindexer encoder", spinMotor.getCurrentPosition());

            telemetry.addData("Index", index);
            telemetry.addData("Ball at Index", balls[index]);
            telemetry.update();


        }

    }

    public int findClosestEmpty(int currentIndex) {
        int n = balls.length;

        for (int offset = 0; offset < n; offset++) {
            int right = (currentIndex + offset) % n;
            int left  = (currentIndex - offset + n) % n;

            if (balls[right] == ballColors.EMPTY) return right;
            if (balls[left] == ballColors.EMPTY) return left;
        }

        return currentIndex; // no empty spot found
    }


    // Target index and position stored globally
    // int index = 0;
    // int[] POSITIONS = {0, 250, 500};
    // int currentTarget = 0;

    public int findClosestColor(ballColors targetColor, int currentIndex) {
        int n = balls.length;

        for (int offset = 0; offset < n; offset++) {
            int right = (currentIndex + offset) % n;
            int left  = (currentIndex - offset + n) % n;

            if (balls[right] == targetColor) return right;
            if (balls[left] == targetColor) return left;
        }

        return currentIndex; // no target color found
    }

    public void spindexer(boolean spinControlIs, boolean spinControlWas) {

        // Step 1: detect new press
        if (spinControlIs && !spinControlWas) {
            index = (index + 1) % POSITIONS.length;
            currentTarget = POSITIONS[index];

            spinMotor.setTargetPosition(currentTarget);
            spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spinMotor.setPower(0.7);  // must be > 0
        }

        // Step 2: stop automatically once reached
        if (!spinMotor.isBusy()) {
            spinMotor.setPower(0);
            spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addData("Spindexer Target", currentTarget);
        telemetry.addData("Spindexer Pos", spinMotor.getCurrentPosition());
        telemetry.addData("Busy?", spinMotor.isBusy());
    }

    public void launch(double servoPosition, boolean launchControl, boolean launchControlReversed, double speed, boolean flickControl) {

        //pivotServo.setPosition(servoPosition);

        if (flickControl) {
            flickServo.setPosition(0.6);
        } else {
            flickServo.setPosition(1);
        }

        leftIs = gamepad1.dpad_up;
        rightIs = gamepad1.dpad_down;

        if (!rightWas && rightIs) {
            //pivotServo.setPosition(Math.min(pivotServo.getPosition() + 0.01, 1));
        }

        if (!leftWas && leftIs) {
            //pivotServo.setPosition(Math.max(pivotServo.getPosition() - 0.01, 0));
        }

        leftWas = gamepad1.dpad_up;
        rightWas = gamepad1.dpad_down;

        if (launchControl) {
            launchMotor.setPower(speed);
        } else if (launchControlReversed) {
            launchMotor.setPower(-speed);
        } else {
            launchMotor.setPower(0);
        }

    }

    public void intakeMotor(boolean intakeMotorControl) {

        if (intakeMotorControl) {
            intakeMotor.setPower(0.67);
        } else {
            intakeMotor.setPower(0);
        }

    }

    public void runToPosition(DcMotor motor, int targetTicks, double power) {
        motor.setTargetPosition(targetTicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    public void initLauncher() {

        pivotServo = hardwareMap.servo.get("pivotServo");

        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");

        flickServo = hardwareMap.servo.get("flickServo");

    }

    public void initIntake() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        spinMotor = hardwareMap.get(DcMotor.class, "spinMotor");
        // Encoder setup
        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set direction (flip if needed)
        spinMotor.setDirection(DcMotor.Direction.REVERSE);

    }

    public void initialize() {

        initLauncher();
        initIntake();

        // fl - 0, fr - 1, bl - 2, br - 3
        drivetrain = new CustomMecanumDrive(hardwareMap);
    }

}
