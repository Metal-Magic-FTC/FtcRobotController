package org.firstinspires.ftc.teamcode.decode.teleOp.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp(name="!!!!!!!TEST")
public class pushTest extends OpMode {

    private CRServo crServo;
    private double amount;

    @Override
    public void init() {
        crServo = hardwareMap.get(CRServo.class, "pivotServo");
        crServo.setPower(0);
        amount = 0.1;
        telemetry.addData("Status","Initialized");
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            amount += 0.01;
        }
        if (gamepad1.y) {
            amount -= 0.01;
        }
        if (gamepad1.a) {
            crServo.setPower(amount);
        } else if (gamepad1.b) {
            crServo.setPower(-amount);
        } else {
            crServo.setPower(0);
        }
        telemetry.addData("Amount", amount);
        telemetry.addData("CrServo power", crServo.getPower());
    }
}
