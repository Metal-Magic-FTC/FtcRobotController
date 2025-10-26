package org.firstinspires.ftc.teamcode.decode.teleOp.actual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.decode.teleOp.CustomMecanumDrive;

@TeleOp(name = "!v0 gang gang gang gang")
public class TeleV0 extends LinearOpMode {

    private CustomMecanumDrive drivetrain;

//    DcMotor leftLaunch = null;
//    DcMotor rightLaunch = null;

    DcMotor intakeMotor = null;

    Servo leftFlickServo = null;
    Servo rightFlickServo = null;

    Servo middleFlickServo = null;

    Servo leftGate = null;
    Servo rightGate = null;


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

            boolean intakeLeftControl = gamepad1.left_bumper;
            boolean intakeRightControl = gamepad1.right_bumper;
//            launchControl(launchControl, reverseLaunchControl);

            //degrees = pivotMotor.getCurrentPosition() * (90.0/135);

            //telemetry.addData("position", pivotMotor.getCurrentPosition());
            //telemetry.addData("target", pivotMotor.getTargetPosition());

            boolean leftFlickControl = gamepad1.x;
            boolean leftLaunch = gamepad1.dpad_left;
            boolean middleFlickControl = gamepad1.a;

            boolean rightFlickControl = gamepad1.b;
            boolean rightLaunch = gamepad1.dpad_right;
            boolean gatesClosedControl = gamepad1.dpad_up;
            double intakeGatesClosedControl = gamepad1.right_trigger;

            // controls "ball movement" servos
            servoMovements(leftFlickControl, leftLaunch, rightFlickControl, rightLaunch, middleFlickControl, gatesClosedControl, intakeRightControl, intakeLeftControl, intakeGatesClosedControl);

            intakeControl(intakeLeftControl || intakeRightControl || intakeGatesClosedControl >= 0.1);

            telemetry.addLine(degrees + "");
            telemetry.update();

        }

    }
    public void servoMovements(boolean leftFlickControl, boolean leftLaunch, boolean rightFlickControl, boolean rightLaunch, boolean middleFlickControl, boolean gatesClosedControl, boolean intakeRightControl, boolean intakeLeftControl, double intakeGatesClosedControl) {

        if (middleFlickControl) { // 4
            middleFlickServo.setPosition(0.75); // open
            rightFlickControl = true; // also resets right ball storage
            leftFlickControl = true; // also resets left ball storage
        } else {
            middleFlickServo.setPosition(1); // close
        }
        
        if (leftFlickControl) { // 2l
            leftGate.setPosition(1); // close
            leftFlickServo.setPosition(1); // open
        }

        if (leftLaunch) { // 3l
            leftGate.setPosition(0.7); // open
            leftFlickServo.setPosition(0.4); // launch
            rightGate.setPosition(0); // close
        }

        if (rightFlickControl) { // 2r
            rightGate.setPosition(0); // close
            rightFlickServo.setPosition(1); // open
        }

        if (rightLaunch) { // 3r
            rightGate.setPosition(0.3); // open
            rightFlickServo.setPosition(0.5); // launch
            leftGate.setPosition(1); // close
        }

        if (intakeRightControl) { // 1r
            leftGate.setPosition(1); // close
            rightGate.setPosition(0.3); // open
        }

        if (intakeLeftControl) { // 1l
            leftGate.setPosition(0.7); // open
            rightGate.setPosition(0); // close
        }



//        if (rightFlickControl) { // 2r
//            rightFlickServo.setPosition(0.4);
//        } else {
//            rightFlickServo.setPosition(1);
//        }
        
        if (gatesClosedControl || intakeGatesClosedControl >= 0.1) { // closes all gates

            leftGate.setPosition(1); // close
            leftFlickServo.setPosition(1); // open
            rightGate.setPosition(0); // close
            rightFlickServo.setPosition(1); // open

        }

    }

    public void intakeControl(boolean intakeControl) {

        if (intakeControl) {
            intakeMotor.setPower(1);
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
        rightFlickServo = hardwareMap.servo.get("rightFlickServo");
        leftGate = hardwareMap.servo.get("leftGate");
        rightGate = hardwareMap.servo.get("rightGate");
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
