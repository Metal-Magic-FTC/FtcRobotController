package org.firstinspires.ftc.teamcode.decode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "!!!SpindexerTest")
public class NewSpinTest extends LinearOpMode {

    private DcMotor spinMotor;
    int[] positions = {0,9,18};
    double[] shooting = {4.5,13.5,22.5};
    int index = 0;

    boolean rightTriggerPressed = false;
    boolean rightTriggerWasPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {

        spinMotor = hardwareMap.get(DcMotor.class, "spinMotor");

        // Encoder setup
        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ;
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set direction (flip if needed)
        spinMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                spinMotor.setTargetPosition(250);
                spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spinMotor.setPower(0.2);
            }
            if (gamepad1.b) {
                spinMotor.setTargetPosition(500);
                spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spinMotor.setPower(0.2);
            }
            if (gamepad1.x) {
                spinMotor.setTargetPosition(0);
                spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spinMotor.setPower(0.2);
            }
//            if (rightTriggerPressed && !rightTriggerWasPressed) {
//                spinMotor.setTargetPosition(spinMotor.getCurrentPosition()+9);
//                spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                spinMotor.setPower(0.05);
//            }

//            rightTriggerWasPressed = gamepad1.right_bumper;

            telemetry.addData("Current Position", spinMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}