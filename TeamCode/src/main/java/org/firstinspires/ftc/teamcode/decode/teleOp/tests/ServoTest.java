package org.firstinspires.ftc.teamcode.decode.teleOp.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp(name="!servoTest")
public class ServoTest extends LinearOpMode {

    Servo testServo;

    @Override
    public void runOpMode() throws InterruptedException {

        testServo = hardwareMap.servo.get("pivotServo");

        waitForStart();

        boolean bWasPressed = false;
        boolean bIsPressed = false;

        boolean leftWas = false;
        boolean leftIs = false;
        boolean rightWas = false;
        boolean rightIs = false;

        while (opModeIsActive()) {

            if (gamepad1.a) {
                testServo.setPosition(0);
            }

            if (gamepad1.b) {
                testServo.setPosition(1);
            }

            leftIs = gamepad1.left_bumper;
            rightIs = gamepad1.right_bumper;

            if (!rightWas && rightIs) {
                testServo.setPosition(Math.min(testServo.getPosition() + 0.01, 1));
            }

            if (!leftWas && leftIs) {
                testServo.setPosition(Math.max(testServo.getPosition() - 0.01, 0));
            }

            leftWas = gamepad1.left_bumper;
            rightWas = gamepad1.right_bumper;

            telemetry.addData("Servo pos:", testServo.getPosition());
            telemetry.update();

        }

    }


}
