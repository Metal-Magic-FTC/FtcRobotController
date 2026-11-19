package org.firstinspires.ftc.teamcode.biobuzz.comp1.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


@TeleOp
public class Transfer extends OpMode {

    public DcMotor intake = null;
    public DcMotor transfer = null;
    double power = 0;
    double sign = 1;

    @Override
    public void init() {
        intake = hardwareMap.dcMotor.get("intakeMotor");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transfer = hardwareMap.dcMotor.get("transferMotor");
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger >= 0.3F) {
            intake.setPower(power);
            transfer.setPower(sign*power);
        }
        else if (gamepad1.left_trigger >= 0.3F) {
            intake.setPower(-power);
            transfer.setPower(-sign*power);
        }
        else {
            intake.setPower(0);
            transfer.setPower(0);
        }

        if (gamepad1.aWasPressed()) {
            power = 1;
        }
        if (gamepad1.bWasPressed()) {
            power = 0;
        }
        if (gamepad1.dpadUpWasPressed()) {
            power += 0.1;
        }
        if (gamepad1.dpadDownWasPressed()) {
            power -= 0.1;
        }
        power = Range.clip(power, 0, 1);
        if (gamepad1.xWasPressed()) {
            sign *= -1;
        }

        telemetry.addData("motor speed", power);
        telemetry.addData("sign", sign);

        telemetry.update();
    }
}
