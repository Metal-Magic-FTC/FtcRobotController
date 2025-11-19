package org.firstinspires.ftc.teamcode.decode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="!FLICK HER servoTest")
public class FlickServoTest extends LinearOpMode {

    Servo testServo;
    DcMotor launchMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        testServo = hardwareMap.servo.get("flickServo");
        launchMotor = hardwareMap.dcMotor.get("launchMotor");
        waitForStart();

        boolean bWasPressed = false;
        boolean bIsPressed = false;

        boolean leftWas = false;
        boolean leftIs = false;
        boolean rightWas = false;
        boolean rightIs = false;
        double power = 1;
        testServo.setPosition(0.6);

        while (opModeIsActive()) {
            //front
            if (gamepad1.x) {
                launchMotor.setPower(0.7);
            }
            if (gamepad1.a) {
                launchMotor.setPower(0.8);
            }
            if (gamepad1.y) {
                launchMotor.setPower(0.9);
            }
            if (gamepad1.b) {
                launchMotor.setPower(0);
            }

            leftIs = gamepad1.left_bumper;
            rightIs = gamepad1.right_bumper;

            if (gamepad1.right_bumper) {
                testServo.setPosition((testServo.getPosition() + 0.001));

            }

            if (gamepad1.left_bumper) {
                testServo.setPosition((testServo.getPosition() - 0.001));
            }

            leftWas = gamepad1.left_bumper;
            rightWas = gamepad1.right_bumper;

            telemetry.addData("Servo pos:", testServo.getPosition());
            telemetry.addData("Motor power:", power);

            telemetry.update();

        }

    }


}
