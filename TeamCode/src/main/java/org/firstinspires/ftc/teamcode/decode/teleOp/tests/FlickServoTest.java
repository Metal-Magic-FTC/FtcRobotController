package org.firstinspires.ftc.teamcode.decode.teleOp.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="!FLICK ER servoTest")
@Disabled
public class FlickServoTest extends LinearOpMode {

    Servo testServo;
    Servo flickServo;
    DcMotor launchMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        testServo = hardwareMap.servo.get("launchServo");
        flickServo = hardwareMap.servo.get("flickServo");
        launchMotor = hardwareMap.dcMotor.get("launchMotor");
        waitForStart();

        boolean bWasPressed = false;
        boolean bIsPressed = false;

        boolean leftWas = false;
        boolean leftIs = false;
        boolean rightWas = false;
        boolean rightIs = false;
        double power = 1;
        testServo.setPosition(0.70);
        flickServo.setPosition(0);

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
                testServo.setPosition((testServo.getPosition() + 0.001));

            }

            if (gamepad1.left_bumper) {
                testServo.setPosition((testServo.getPosition() - 0.001));
            }
            if (gamepad1.right_trigger>=0.4f) {
                flickServo.setPosition(0.22); // 0.6

            } else {
                flickServo.setPosition(0); // 1
            }

            leftWas = gamepad1.left_bumper;
            rightWas = gamepad1.right_bumper;

            telemetry.addData("Servo pos:", testServo.getPosition());
            telemetry.addData("Motor power:", power);
            telemetry.addData("Flick pos:", flickServo.getPosition());

            telemetry.update();

        }

    }


}
