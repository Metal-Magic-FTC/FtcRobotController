package org.firstinspires.ftc.teamcode.decode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp(name = "!!!SpindexerTest")
public class SpindexerTest extends LinearOpMode {
    private DcMotor spinMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        spinMotor = hardwareMap.get(DcMotor.class, "spinMotor");

        // Encoder setup
        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set direction (flip if needed)
        spinMotor.setDirection(DcMotor.Direction.FORWARD);

        // 3 positions
        int[] positions = {0, 250, 500};
        int index = 0;

        boolean aWasPressed = false;

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Op mode active");
            boolean aPressed = gamepad1.a;

            if (aPressed && !aWasPressed) {
                telemetry.addData("A pressed: ", gamepad1.a);
                telemetry.update();
                // Advance index
                index = (index + 1) % positions.length;
                int target = positions[index];

                // ----- MANUAL RUN_USING_ENCODER MOVEMENT -----
                if (target > spinMotor.getCurrentPosition()) {
                    // Move forward
                    spinMotor.setPower(-0.2);
                    while (opModeIsActive() && spinMotor.getCurrentPosition() < target) {
                        telemetry.addData("Moving → Target", target);
                        telemetry.addData("Position", spinMotor.getCurrentPosition());
                        telemetry.update();
                    }
                } else {
                    // Move backward
                    spinMotor.setPower(0.2);
                    while (opModeIsActive() && spinMotor.getCurrentPosition() > target) {
                        telemetry.addData("Moving ← Target", target);
                        telemetry.addData("Position", spinMotor.getCurrentPosition());
                        telemetry.update();
                    }
                }

                // Stop motor once target reached
                spinMotor.setPower(0);
            } else {
                telemetry.addLine("Nothing pressed");
                telemetry.update();
            }

            aWasPressed = aPressed;

            telemetry.addData("Target Index", index);
            telemetry.addData("Current Position", spinMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}