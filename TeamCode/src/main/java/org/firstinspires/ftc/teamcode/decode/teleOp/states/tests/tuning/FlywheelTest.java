package org.firstinspires.ftc.teamcode.decode.teleOp.states.tests.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;


import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

import kotlin.math.UMathKt;

@TeleOp(name="Tune the flywheel PIDF")
public class FlywheelTest extends OpMode {

    public DcMotorEx launchMotor;

    public double velocity = 2000;
    public double highVelocity = 2000;
    public double lowVelocity = 900;
    private Follower follower;


    double curTargetVelocity = highVelocity;

    double P = 0;
    double F = 0;
    double X = 0;
    double Y = 0;
    double distance = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1;


    @Override
    public void init() {

        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setDirection(DcMotor.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addLine("initialized");
        follower = Constants.createFollower(hardwareMap);

        // ✅ Known start
        follower.setPose(new Pose(87, 9, Math.toRadians(90)));
    }

    @Override
    public void loop() {

        // get all gamepad commands
        // set target velocity
        // update telemetry
        X = follower.getPose().getX();
        Y = follower.getPose().getY();
        distance = Math.pow(Math.pow(Math.abs(129-X),2)+Math.pow(Math.abs(132-Y),2), 0.5);

        if (gamepad1.yWasPressed()) {
            velocity+=100;
        }

        if (gamepad1.bWasPressed()) {
            velocity-=100;
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
        //PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        //launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        launchMotor.setVelocity(velocity);

        double curVelocity = launchMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Distance: ", distance);

        telemetry.addData("Target velocity", velocity);
//        telemetry.addData("Current velocity", curVelocity);
//        telemetry.addData("Error", "%.2f", error);
//        telemetry.addLine("-----------------------------");
//        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
//        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
//        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);

    }

}
