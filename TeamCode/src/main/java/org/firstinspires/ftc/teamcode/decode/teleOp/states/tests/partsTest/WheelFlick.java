package org.firstinspires.ftc.teamcode.decode.teleOp.states.tests.partsTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class WheelFlick extends OpMode {

    Servo flickServo;

    DcMotorEx flickMotor;

    @Override
    public void init() {

        flickServo = hardwareMap.servo.get("flickServo");
        flickMotor = hardwareMap.get(DcMotorEx.class, "flickMotor");

    }

    @Override
    public void loop() {

    }

}
