package org.firstinspires.ftc.teamcode.decode.teleOp.states.tests.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name="!!!!!Tune the spindexer PIDF")
public class SpindexerTuningTest extends OpMode {

    public DcMotorEx spinMotor;
    int current = 0;
    int[] positions = {125,375,625};

    double P = 0;
    double F = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1;


    @Override
    public void init() {

        spinMotor = hardwareMap.get(DcMotorEx.class, "spinMotor");
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinMotor.setDirection(DcMotor.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        spinMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addLine("initialized");

    }

    @Override
    public void loop() {

        // get all gamepad commands
        // set target velocity
        // update telemetry

//        if (gamepad1.yWasPressed()) {
//            if (curTargetVelocity == highVelocity) {
//                curTargetVelocity = lowVelocity;
//            } else {
//                curTargetVelocity = highVelocity;
//            }
//        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        // set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        spinMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        if (gamepad1.a) {
            current = iterate(current);
            spinMotor.setTargetPosition(positions[current]);
            spinMotor.setPower(0.38);
        }

        double curVelocity = spinMotor.getVelocity();

        //double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target position", positions[current]);
        telemetry.addData("Current velocity", curVelocity);
        //telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("-----------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);

    }
    private int iterate (int currentPos) {
        if (currentPos==2) {
            return 0;
        } else {
            return currentPos++;
        }
    }
}
