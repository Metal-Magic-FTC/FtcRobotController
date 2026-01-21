package org.firstinspires.ftc.teamcode.decode.teleOp.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp(name="!!gate test")
public class GateTest extends LinearOpMode {

    Servo leftGate = null;
    Servo rightGate = null;
    Servo rightFlickServo = null;

    @Override
    public void runOpMode() throws InterruptedException {

        leftGate = hardwareMap.servo.get("leftGate");
        rightGate = hardwareMap.servo.get("rightGate");
        rightFlickServo = hardwareMap.servo.get("rightFlickServo");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                leftGate.setPosition(0.7);
            }

            if (gamepad1.right_bumper) {
                leftGate.setPosition(1);
            }

            if (gamepad1.left_trigger >= 0.3) {
                rightGate.setPosition(0);
            }

            if (gamepad1.right_trigger >= 0.3) {
                rightGate.setPosition(0.3);
            }

            if (gamepad1.a) {
                rightFlickServo.setPosition(0.5);
            }

            if (gamepad1.b) {
                rightFlickServo.setPosition(1);
            }

        }

    }

}
