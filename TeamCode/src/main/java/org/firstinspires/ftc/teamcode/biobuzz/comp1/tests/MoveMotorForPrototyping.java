package org.firstinspires.ftc.teamcode.biobuzz.comp1.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Test A/B")
public class MoveMotorForPrototyping extends LinearOpMode {

    private DcMotor motor;

    @Override
    public void runOpMode() {
        // "motor" must match the name you gave this motor in your robot configuration
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready - press Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                motor.setPower(1.0);      // forward
            } else if (gamepad1.b) {
                motor.setPower(-1.0);     // reverse
            } else {
                motor.setPower(0.0);      // stop
            }

            telemetry.addData("A pressed", gamepad1.a);
            telemetry.addData("B pressed", gamepad1.b);
            telemetry.addData("Motor Power", motor.getPower());
            telemetry.update();
        }
    }
}