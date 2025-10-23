package org.firstinspires.ftc.teamcode.decode.teleOp.actual;

import com.qualcomm.ftccommon.configuration.EditLynxUsbDeviceActivity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.decode.teleOp.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;

@TeleOp(name = "!v0 gang gang gang gang")
public class TeleV0 extends LinearOpMode {

    private CustomMecanumDrive drivetrain;

//    DcMotor leftLaunch = null;
//    DcMotor rightLaunch = null;

    DcMotor intakeMotor = null;

    Servo leftFlickServo = null;

    Servo middleFlickServo = null;

    //DcMotor pivotMotor = null;

    private double degrees = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize(); // initializing everything

        waitForStart(); // waiting until driver clicks play button

        leftFlickServo.setPosition(1);
        middleFlickServo.setPosition(1);

        while (opModeIsActive()) { // duration of opMode


            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            drivetrain.driveMecanum(strafe, drive, turn); // creates mecanum drivetrain

            boolean intakeControl = gamepad1.left_bumper;
            boolean reverseIntakeControl = gamepad1.right_bumper;
            double launchControl = gamepad1.right_trigger;
            double reverseLaunchControl = gamepad1.left_trigger;

            intakeControl(intakeControl, reverseIntakeControl);
            launchControl(launchControl, reverseLaunchControl);

            //degrees = pivotMotor.getCurrentPosition() * (90.0/135);

            //telemetry.addData("position", pivotMotor.getCurrentPosition());
            //telemetry.addData("target", pivotMotor.getTargetPosition());

            boolean leftFlickControl = gamepad1.b;
            boolean middleFlickControl = gamepad1.a;
            servoMovements(leftFlickControl, middleFlickControl);

            telemetry.addLine(degrees + "");
            telemetry.update();

        }

    }

    public void servoMovements(boolean leftFlickControl, boolean middleFlickControl) {

        if (leftFlickControl) {
            leftFlickServo.setPosition(0.4);
        } else {
            leftFlickServo.setPosition(1);
        }

        if (middleFlickControl) {
            middleFlickServo.setPosition(0.75);
        } else {
            middleFlickServo.setPosition(1);
        }

    }

    public void intakeControl(boolean intakeControl, boolean reverseIntakeControl) {

        if (intakeControl) {
            intakeMotor.setPower(1);
        } else if (reverseIntakeControl) {
            intakeMotor.setPower(-1);
        } else {
            intakeMotor.setPower(0);
        }
    }

    public void launchControl(double launchControl, double reverseLaunchControl) {

        //leftLaunch.setPower(launchControl - reverseLaunchControl);
        //rightLaunch.setPower(launchControl - reverseLaunchControl);

    }

    public void runToPosition(DcMotor motor, int targetTicks, double power) {
        motor.setTargetPosition(targetTicks);
        motor.setPower(power); // always positive power for RUN_TO_POSITION
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void initLauncher() {
//        leftLaunch = hardwareMap.get(DcMotor.class, "leftLaunch");
//        rightLaunch = hardwareMap.get(DcMotor.class, "rightLaunch");
//
//        leftLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        leftLaunch.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightLaunch.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFlickServo = hardwareMap.servo.get("leftFlickServo");
        middleFlickServo = hardwareMap.servo.get("middleFlickServo");

    }

    public void initIntake() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void initialize() {

        initLauncher();
        initIntake();

        //pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");

        //pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //pivotMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        // fl - 0, fr - 1, bl - 2, br - 3
        drivetrain = new CustomMecanumDrive(hardwareMap);
    }

}
