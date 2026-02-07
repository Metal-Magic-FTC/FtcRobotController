package org.firstinspires.ftc.teamcode.decode.teleOp.statesTele;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.teleOp.tests.CustomMecanumDrive;

@TeleOp(name="!!!!! Auto Align")

public class AutoAlign extends LinearOpMode {

    private CustomMecanumDrive drivetrain;
    private Follower follower;

    private static double TARGET_X = 150;
    private static double TARGET_Y = 137;

    public static Pose startPose = new Pose(
            109,
            130,
            Math.toRadians(180)
    );

    double targetHeading;
    PIDFController controller;
    boolean headingLock;


    @Override
    public void runOpMode() throws InterruptedException {
        initialize(); // initializing everything

        waitForStart(); // waiting until driver clicks play button

        while (opModeIsActive()) {
            follower.update();
            headingLock = gamepad1.x;

            controller.setCoefficients(follower.constants.coefficientsHeadingPIDF);
            controller.updateError(getHeadingError());

            if (headingLock)
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, controller.run(), true);
            else
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

            telemetry.update();
        }
    }

    public double getHeadingError() {
        if (follower.getCurrentPath() == null) {
            return 0;
        }

        double headingError = MathFunctions.getTurnDirection(follower.getPose().getHeading(), targetHeading) * MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), targetHeading);
        return headingError;
    }

    public void initialize() {

        // fl - 0, fr - 1, bl - 2, br - 3
        drivetrain = new CustomMecanumDrive(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose == null ? new Pose() : startPose);
        follower.update();

        targetHeading = Math.toRadians(180); // Radians
        controller = new PIDFController(follower.constants.coefficientsHeadingPIDF);
        headingLock = false;

        follower.startTeleopDrive();
    }

}
