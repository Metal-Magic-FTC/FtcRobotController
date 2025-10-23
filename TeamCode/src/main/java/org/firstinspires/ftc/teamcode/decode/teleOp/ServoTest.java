package org.firstinspires.ftc.teamcode.decode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="!servoTest")
public class ServoTest extends LinearOpMode {

    Servo testServo;

    @Override
    public void runOpMode() throws InterruptedException {

        testServo = hardwareMap.servo.get("pivotServo");
        testServo.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                testServo.setPosition(0);
            }

            if (gamepad1.b) {
                testServo.setPosition(1);
            }

            if (gamepad1.right_bumper) {
                testServo.setPosition(Math.min(testServo.getPosition() + 0.01, 1));
            }

            if (gamepad1.left_bumper) {
                testServo.setPosition(Math.max(testServo.getPosition() - 0.01, 0));
            }

            if (gamepad1.x) {
                testServo.setPosition(0.78);
            } else {
                testServo.setPosition(1);
            }
            telemetry.addData("Servo pos:", testServo.getPosition());
            telemetry.update();

        }

    }


}
