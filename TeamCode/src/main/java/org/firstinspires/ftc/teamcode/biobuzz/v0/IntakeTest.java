package org.firstinspires.ftc.teamcode.biobuzz.v0;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="v0IntakeTest")
public class IntakeTest extends OpMode {

    private DcMotor intakeMotor;
    private double amnt;
    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setPower(0);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {

        if (gamepad1.x) {
            intakeMotor.setPower(amnt);
        } else {
            intakeMotor.setPower(0);
        }

        if (gamepad1.y) {
            intakeMotor.setPower(-amnt);
        } else {
            intakeMotor.setPower(0);
        }

        if (gamepad1.a && !gamepad1.aWasPressed()) {
            amnt += 0.05;
        }
        if (gamepad1.b && !gamepad1.bWasPressed()) {
            amnt -= 0.05;
        }

        telemetry.addData("amnt: ", amnt);
        telemetry.addData("power: ", intakeMotor.getPower());

    }
}
