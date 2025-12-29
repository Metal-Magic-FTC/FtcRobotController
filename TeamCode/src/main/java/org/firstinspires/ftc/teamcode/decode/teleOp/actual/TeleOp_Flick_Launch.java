package org.firstinspires.ftc.teamcode.decode.teleOp.actual;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.decode.teleOp.CustomMecanumDrive;

@TeleOp(name="Drive, Flick, Launch")
//@Disabled
public class TeleOp_Flick_Launch extends LinearOpMode {

    Servo hoodServo;
    Servo flickServo;
    DcMotor launchMotor;

    private CustomMecanumDrive drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {

        hoodServo = hardwareMap.servo.get("hoodServo");
        flickServo = hardwareMap.servo.get("flickServo");
        launchMotor = hardwareMap.dcMotor.get("launchMotor");
        drivetrain = new CustomMecanumDrive(hardwareMap);

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

            telemetry.addData("Servo pos:", hoodServo.getPosition());
            telemetry.addData("Launch power:", power);
            telemetry.addData("Flick pos:", flickServo.getPosition());

            telemetry.update();

        }

    }

    private void driveMecanum(double strafe, double drive, double turn) {
        drivetrain.driveMecanum(strafe, drive, turn);
    }

}
