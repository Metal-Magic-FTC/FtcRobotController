package org.firstinspires.ftc.teamcode.decode.teleOp.actual;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.decode.teleOp.CustomMecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@TeleOp(name="Drive, Flick, Launch")
//@Disabled
public class TeleOp_Flick_Launch extends LinearOpMode {

    Servo hoodServo;
    Servo flickServo;
    DcMotorEx launchMotor;

    DcMotor spinMotor;

    private CustomMecanumDrive drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {

        hoodServo = hardwareMap.servo.get("hoodServo");
        flickServo = hardwareMap.servo.get("flickServo");
        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        spinMotor = hardwareMap.get(DcMotor.class, "spinMotor");
        drivetrain = new CustomMecanumDrive(hardwareMap);

        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ;
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set direction (flip if needed)
        spinMotor.setDirection(DcMotor.Direction.REVERSE);

        launchMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        boolean bWasPressed = false;
        boolean bIsPressed = false;

        boolean leftWas = false;
        boolean leftIs = false;
        boolean rightWas = false;
        boolean rightIs = false;
        double power = 1;
        hoodServo.setPosition(0);
        flickServo.setPosition(0.90);

        while (opModeIsActive()) {
            //front distance (close to basket) - position: 0.75 and power: 0.7
            //back distance (peak of the large pyramid) - position 0.75 power: 0.9
            if (gamepad1.x) {
                launchMotor.setPower(0.7);
            }
            if (gamepad1.a) {
                launchMotor.setPower(0.9);
            }
            if (gamepad1.y) {
                launchMotor.setPower(1);
            }
            if (gamepad1.b) {
                launchMotor.setPower(0);
            }

            leftIs = gamepad1.left_bumper;
            rightIs = gamepad1.right_bumper;

            if (gamepad1.right_bumper) {
                hoodServo.setPosition((hoodServo.getPosition() + 0.001));
            }

            if (gamepad1.left_bumper) {
                hoodServo.setPosition((hoodServo.getPosition() - 0.001));
            }

            if (gamepad1.right_trigger>=0.4f) {
                flickServo.setPosition(0.67); // 0.6

            } else {
                flickServo.setPosition(0.9); // 1
            }

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            drivetrain.driveMecanum(strafe, drive, turn);

            leftWas = gamepad1.left_bumper;
            rightWas = gamepad1.right_bumper;

            spinMotor.setTargetPosition(0);
            spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spinMotor.setPower(0.2);

            telemetry.addData("Servo pos:", hoodServo.getPosition());
            telemetry.addData("Launch power:", power);
            telemetry.addData("Flick pos:", flickServo.getPosition());
            telemetry.addData("Launch power", launchMotor.getPower());
            telemetry.addData("Launch Velocity", launchMotor.getVelocity());
            telemetry.addData("RPM", (launchMotor.getVelocity(AngleUnit.DEGREES))/6);

            telemetry.update();

        }

    }

    private void driveMecanum(double strafe, double drive, double turn) {
        drivetrain.driveMecanum(strafe, drive, turn);
    }

}
