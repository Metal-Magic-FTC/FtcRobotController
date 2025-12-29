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

            rightTriggerPressed = gamepad1.right_bumper;

            if (rightTriggerPressed) {
                spinMotor.setTargetPosition(positions[iterIndex(index)]);
                spinMotor.setPower(0.02);
            } else if (gamepad1.left_trigger>=0.01f){
                spinMotor.setTargetPosition(positions[iterIndex(index)]);
                spinMotor.setPower(-0.02);
            } else {
                spinMotor.setPower(0);
            }


            if (gamepad1.aWasReleased())
                spinMotor.setTargetPosition(9);
            if (gamepad1.bWasReleased())
                spinMotor.setTargetPosition(18);
            if (gamepad1.cWasReleased())
                spinMotor.setTargetPosition(27);
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
    private int iterIndex(int index) {
        if (index==2) {
            index=0;
            return index;
        } else {
            index+=1;
            return index;
        }
    }
//    private BallColor detectColor(NormalizedColorSensor sensor) {
//        NormalizedRGBA c = sensor.getNormalizedColors();
//        float r = c.red, g = c.green, b = c.blue;
//        float tol = 0.20f;
//
//        if (b > r*(1+tol) && b > g*(1+tol)) return BallColor.PURPLE;
//        if (g > r*(1+tol) && g > b*(1+tol)) return BallColor.GREEN;
//
//        if (r > 0.01 || g > 0.01 || b > 0.01) return BallColor.EMPTY;
//        return BallColor.EMPTY;
//    }
}