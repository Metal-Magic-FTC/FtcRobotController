package org.firstinspires.ftc.teamcode.decode.teleOp.actual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.decode.teleOp.CustomMecanumDrive;

@TeleOp(name="!!!!! IM MID GOON SESSION DONT DISTURBME IM GOONING TO TeleV2 - Modular")
public class TeleV2 extends LinearOpMode {

    // -----------------------------
    // HARDWARE + GLOBAL VARS
    // -----------------------------

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
    final float[] hsvValues = new float[3];

    boolean spinControlWas = false;

    enum ballColors {
        PURPLE, GREEN, EMPTY, UNKNOWN
    }

    // -----------------------------
    // INIT
    // -----------------------------
    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();
        resetBallArray();

        waitForStart();

        while (opModeIsActive()) {

            // ============================================================
            //                      ALL CONTROLS AT TOP
            // ============================================================

            // Gamepad1
            double drive                 = -gamepad1.left_stick_y;
            double strafe                =  gamepad1.left_stick_x;
            double turn                  =  gamepad1.right_stick_x;

            boolean intakeButton         = gamepad1.left_trigger  > 0.167;
            boolean launchForward        = gamepad1.right_trigger  > 0.3;
            boolean launchReverse        = gamepad2.right_trigger > 0.3;
            boolean flickButton          = gamepad1.left_bumper;

            boolean gp2_resetIndex       = gamepad2.left_trigger > 0.3;          // move to pos 0
            boolean gp2_scanAll          = gamepad2.left_bumper;
            boolean gp2_markUnknown      = gamepad1.dpad_down || gamepad2.dpad_down;
            boolean gp2_launchBall       = gamepad1.left_bumper || gamepad2.y;
            boolean gp2_toNextPurple     = gamepad1.b || gamepad2.b;
            boolean gp2_toNextGreen      = gamepad1.a || gamepad2.x;
            boolean gp2_toNextEmpty      = gamepad1.right_bumper || gamepad2.dpad_up;

            boolean gp2_setPurple        = gamepad2.dpad_right;
            boolean gp2_setGreen         = gamepad2.dpad_left;

            // ============================================================
            //                 CALL SUBSYSTEM HANDLERS
            // ============================================================

           // driveMecanum(drive, strafe, turn);

            handleIntake(intakeButton);
            handleLauncher(launchForward, launchReverse, flickButton);

            handleBalls(
                    gp2_resetIndex, gp2_markUnknown, gp2_scanAll,
                    gp2_launchBall, gp2_toNextPurple, gp2_toNextGreen, gp2_toNextEmpty,
                    gp2_setPurple, gp2_setGreen
            );

            // ============================================================
            // TELEMETRY
            // ============================================================
            telemetry.addData("Index", index);
            telemetry.addData("Ball at Index", balls[index]);
            telemetry.addData("Spindexer Encoder", spinMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    // -----------------------------
    // SUBSYSTEM METHODS
    // -----------------------------

    /** DRIVETRAIN **/
    private void driveMecanum(double strafe, double drive, double turn) {
        drivetrain.driveMecanum(strafe, drive, turn);
    }

    /** INTAKE **/
    private void handleIntake(boolean intakeButton) {
        intakeMotor.setPower(intakeButton ? 0 : 0.67);
    }

    /** LAUNCHER **/
    private void handleLauncher(
            boolean forward, boolean reverse, boolean flickButton
    ) {

        if (flickButton)  flickServo.setPosition(0.22);
        else              flickServo.setPosition(0);

        if      (forward) launchMotor.setPower(1);
        else if (reverse) launchMotor.setPower(-1);
        else              launchMotor.setPower(0);
    }

    /** GAMEPAD 2 COMMAND GROUPS **/
    private void handleBalls(
            boolean resetIndex,
            boolean markUnknown,
            boolean scanAll,
            boolean launchBall,
            boolean toNextPurple,
            boolean toNextGreen,
            boolean toNextEmpty,
            boolean setPurple,
            boolean setGreen
    ) {

        if (resetIndex) {
            index = 0;
            moveSpindexer(index, POSITIONS);
        }

        if (markUnknown) {
            balls[index] = ballColors.UNKNOWN;
        }

        if (scanAll) {
            scanAllBalls();
        }

        if (launchBall) {
            launchBallAt(index);
        }

        if (toNextPurple) {
            index = findClosestColor(ballColors.PURPLE, index);
            moveSpindexer(index, POSITIONS);
        }

        if (toNextGreen) {
            index = findClosestColor(ballColors.GREEN, index);
            moveSpindexer(index, POSITIONS);
        }

        if (toNextEmpty) {
            index = findClosestEmpty(index);
            moveSpindexer(index, INTAKE_POSITIONS);
        }

        if (setPurple) balls[index] = ballColors.PURPLE;
        if (setGreen)  balls[index] = ballColors.GREEN;
    }

    /** Launch ball at specific index **/
    private void launchBallAt(int index) {
        if (balls[index] != ballColors.EMPTY) {
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
    // COLOR + SPIN HELPERS
    // -----------------------------

    private void moveSpindexer(int newIndex, int[] table) {
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

    /** Color detection **/
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
    // INIT HELPERS
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