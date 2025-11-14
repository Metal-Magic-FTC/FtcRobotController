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

    int spinPosition;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize(); // initializing everything

        waitForStart(); // waiting until driver clicks play button

        spinPosition = 0;

        boolean spinControlIs = false;
        boolean spinControlWas = false;

        while (opModeIsActive()) { // duration of opMode


            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            drivetrain.driveMecanum(strafe, drive, turn); // creates mecanum drivetrain

            boolean intakeMotorControl = gamepad1.right_bumper;
            boolean launchControl = gamepad1.left_trigger >= 0.3;
            boolean launchControlReversed = gamepad1.right_trigger >= 0.3;

            intakeMotor(intakeMotorControl);
            launch(0.34, launchControl, launchControlReversed, 1);

            spinControlIs = gamepad1.dpad_up;

            if (spinControlIs && !spinControlWas) {
                spinPosition ++;
                spinPosition %= 3;
                spinMotor.setPower(0);
            }

            spinMotor(spinPosition);

            spinControlWas = gamepad1.dpad_up;

            telemetry.update();

            telemetry.addData("spindexer button position", spinPosition);
            telemetry.addData("spindexer encoder", spinMotor.getCurrentPosition());

        }

    }

    public void launch(double servoPosition, boolean launchControl, boolean launchControlReversed, double speed) {

        pivotServo.setPosition(servoPosition);
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
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
        }

    }

    public void spinMotor(int position) {
        runToPosition(spinMotor, position * 250, 0.2);
    }

    public void runToPosition(DcMotor motor, int targetTicks, double power) {
        motor.setTargetPosition(targetTicks);
        motor.setPower(power); // always positive power for RUN_TO_POSITION
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void initLauncher() {

        pivotServo = hardwareMap.servo.get("pivotServo");

        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");

    }

    public void initIntake() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        spinMotor = hardwareMap.get(DcMotor.class, "spinMotor");
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        spinMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spinMotor.setTargetPosition(0);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void initialize() {

        initLauncher();
        initIntake();



        // fl - 0, fr - 1, bl - 2, br - 3
        drivetrain = new CustomMecanumDrive(hardwareMap);
    }

}
