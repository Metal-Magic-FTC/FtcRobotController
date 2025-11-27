package org.firstinspires.ftc.teamcode.decode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="!FlickHerServoTestOnlyPleaseDontAddAnyThingElseButFlickServo")
public class FlickHerServoTestOnlyPleaseDontAddAnyThingElseButFlickServo extends LinearOpMode {

    Servo flickServo;

    @Override
    public void runOpMode() throws InterruptedException {

        flickServo = hardwareMap.servo.get("flickServo");
        waitForStart();

        boolean bWasPressed = false;
        boolean bIsPressed = false;

        boolean leftWas = false;
        boolean leftIs = false;
        boolean rightWas = false;
        boolean rightIs = false;
        double power = 1;
        flickServo.setPosition(1);

        while (opModeIsActive()) {
            //front distance (close to basket) - position: 0.75 and power: 0.7
            //back distance (peak of the large pyramid) - position 0.75 power: 0.9

            leftIs = gamepad1.left_bumper;
            rightIs = gamepad1.right_bumper;


            if (leftIs && !leftWas) {
                flickServo.setPosition(flickServo.getPosition() - 0.01);
            }

            if (rightIs && !rightWas) {
                flickServo.setPosition(flickServo.getPosition() + 0.01);
            }

            if (gamepad1.right_trigger>=0.4f) {
                flickServo.setPosition(0.6);

            } else {
                flickServo.setPosition(1);
            }

            leftWas = gamepad1.left_bumper;
            rightWas = gamepad1.right_bumper;

            telemetry.addData("Motor power:", power);
            telemetry.addData("Flick pos:", flickServo.getPosition());

            telemetry.update();

        }

    }


}
