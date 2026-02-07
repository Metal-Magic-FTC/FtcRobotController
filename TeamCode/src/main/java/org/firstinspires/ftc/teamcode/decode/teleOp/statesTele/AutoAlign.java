package org.firstinspires.ftc.teamcode.decode.teleOp.statesTele;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.teleOp.states.tests.limelightV2.FusedPose;
import org.firstinspires.ftc.teamcode.decode.teleOp.tests.CustomMecanumDrive;

@TeleOp(name="!!!!! Auto Align")

public class AutoAlign extends LinearOpMode {

    private CustomMecanumDrive drivetrain;
    private Follower follower;
    FusedPose fusedPose;

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
            fusedPose.update();
            Pose limelightPose = fusedPose.getRobotPose(true); // CONVERTED pose
            if (limelightPose != null) {
                follower.setPose(limelightPose);
            }
            updateGoalHeading();

            headingLock = gamepad1.x;

            controller.setCoefficients(follower.constants.coefficientsHeadingPIDF);
            double v = getHeadingError();
            controller.updateError(v);

            double otherV = 0;
            
            if (headingLock) {
                otherV = controller.run();
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, otherV, true);
            } else
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

            telemetry.addData("poseX", follower.getPose().getX());
            telemetry.addData("headingLock", headingLock);
            telemetry.addData("error", v);
            telemetry.addData("otherV", otherV);
            telemetry.update();
        }
    }

    public double getHeadingError() {
        double headingError = MathFunctions.getTurnDirection(follower.getPose().getHeading(), targetHeading) * MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), targetHeading);
        return headingError;
    }

    public void updateGoalHeading() {
        Pose robotPos = follower.getPose();
        double angleToTarget = MathFunctions.normalizeAngle(Math.atan2(
                TARGET_Y - robotPos.getY(),
                TARGET_X - robotPos.getX()
        ));
        targetHeading = angleToTarget;
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

        fusedPose = new FusedPose(hardwareMap, startPose);
    }

}
