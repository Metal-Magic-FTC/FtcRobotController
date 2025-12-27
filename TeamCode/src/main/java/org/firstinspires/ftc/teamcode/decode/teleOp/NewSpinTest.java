package org.firstinspires.ftc.teamcode.decode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name = "!!!SpindexerTest")
public class NewSpinTest extends LinearOpMode {

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
        int[] positions = {0, 1500, 3000};
        int index = 0;

        boolean aWasPressed = false;

        waitForStart();

        while (opModeIsActive()) {

            boolean aPressed = gamepad1.a;

            if (aPressed && !aWasPressed) {
                int target = spinMotor.getCurrentPosition()+30;
                while (opModeIsActive() && spinMotor.getCurrentPosition()<target) {
                    spinMotor.setPower(0.02);
                }
//                // Advance index
//                index = (index + 1) % positions.length;
//                int target = positions[index];
//
//                // ----- MANUAL RUN_USING_ENCODER MOVEMENT -----
//                if (target > spinMotor.getCurrentPosition()) {
//                    // Move forward
//                    spinMotor.setPower(-0.02);
//                    while (opModeIsActive() && spinMotor.getCurrentPosition() < target) {
//                        telemetry.addData("Moving → Target", target);
//                        telemetry.addData("Position", spinMotor.getCurrentPosition());
//                        telemetry.update();
//                    }
//                } else {
//                    // Move backward
//                    spinMotor.setPower(0.02);
//                    while (opModeIsActive() && spinMotor.getCurrentPosition() > target) {
//                        telemetry.addData("Moving ← Target", target);
//                        telemetry.addData("Position", spinMotor.getCurrentPosition());
//                        telemetry.update();
//                    }
//                }
//
//                // Stop motor once target reached
//                spinMotor.setPower(0);
            }

            aWasPressed = aPressed;

            telemetry.addData("Target Index", index);
            telemetry.addData("Current Position", spinMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}