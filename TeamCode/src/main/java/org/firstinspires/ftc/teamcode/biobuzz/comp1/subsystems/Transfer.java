package org.firstinspires.ftc.teamcode.biobuzz.comp1.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


@TeleOp
public class Transfer extends OpMode {

    public DcMotor transfer = null;
    double power = 0;
    double incr = 0.1;

    @Override
    public void init() {
        // Get the motor from the hardware map BEFORE configuring it
        transfer = hardwareMap.dcMotor.get("test");
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger >= 0.3F) {
            transfer.setPower(power);
        }
        else if (gamepad1.left_trigger >= 0.3F) {
            transfer.setPower(-power);
        }
        else {
            transfer.setPower(0);
        }

        // xWasPressed() is already edge-triggered: true once per press
        if (gamepad1.aWasPressed()) {
            power += incr;
        }
        if (gamepad1.bWasPressed()) {
            power -= incr;
        }
        power = Range.clip(power, 0, 1);

        if (gamepad1.xWasPressed()) {
            incr -= 0.01;
        }
        if (gamepad1.yWasPressed()) {
            incr += 0.01;
        }
        incr = Range.clip(incr, 0.01, 1);

        telemetry.addData("motor pos", transfer.getCurrentPosition());
        telemetry.addData("motor speed", power);
        telemetry.addData("increment", incr);

        telemetry.update();
    }
}
