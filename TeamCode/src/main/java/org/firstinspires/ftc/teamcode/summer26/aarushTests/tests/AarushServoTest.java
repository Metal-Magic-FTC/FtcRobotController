package org.firstinspires.ftc.teamcode.summer26.aarushTests.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="AarushTESTGUTHA")
public class AarushServoTest extends OpMode {

    Servo leftServo;
    Servo rightServo;

    double position = 0.5;

    @Override
    public void init() {
        leftServo = hardwareMap.get(Servo.class, "Servo1");
        rightServo = hardwareMap.get(Servo.class, "Servo2");

        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            position += 0.01;
        }

        if (gamepad1.b) {
            position -= 0.01;
        }

        position = Math.max(0, Math.min(1, position));

        leftServo.setPosition(position);
        rightServo.setPosition(1 - position);

        telemetry.addData("Position", position);
        telemetry.update();

    }
}