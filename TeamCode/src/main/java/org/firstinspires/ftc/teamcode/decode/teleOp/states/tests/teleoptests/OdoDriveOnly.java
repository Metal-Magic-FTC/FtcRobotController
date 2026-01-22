package org.firstinspires.ftc.teamcode.decode.teleOp.states.tests.teleoptests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.teleOp.tests.CustomMecanumDrive;

@TeleOp(name = "OdoDriveOnly")
public class OdoDriveOnly extends LinearOpMode {

    // ---- TARGET POSE YOU GAVE ----
    public static final Pose SHOOT_POSE = new Pose(
            95,
            103,
            Math.toRadians(45)
    );

    // ---- SET THIS TO YOUR REAL START POSE ----
    public static final Pose START_POSE = new Pose(
            116,
            128,
            Math.toRadians(225)
    );

    private Follower follower;
    private CustomMecanumDrive drivetrain;

    private boolean prevA = false;
    private boolean prevB = false;

    private boolean autoRunning = false;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        drivetrain = new CustomMecanumDrive(hardwareMap);

        follower.setPose(START_POSE);

        waitForStart();

        while (opModeIsActive()) {

            follower.update();

            // ---- manual drive ----
            double drive  = -gamepad1.left_stick_y;
            double strafe =  gamepad1.left_stick_x;
            double turn   =  gamepad1.right_stick_x;

            boolean aPressed = gamepad1.a && !prevA;
            boolean bPressed = gamepad1.b && !prevB;

            prevA = gamepad1.a;
            prevB = gamepad1.b;

            // Driver override cancels auto
            boolean driverOverride =
                    Math.abs(drive) > 0.05 ||
                            Math.abs(strafe) > 0.05 ||
                            Math.abs(turn) > 0.05;

            if (autoRunning && driverOverride) {
                follower.breakFollowing();
                autoRunning = false;
            }

            // ---- A: FACE START (TURN ONLY) ----
            if (aPressed) {
                Pose current = follower.getPose();

                double dx = START_POSE.getX() - current.getX();
                double dy = START_POSE.getY() - current.getY();
                double targetHeading = Math.atan2(dy, dx);

                // Turn in place: same X/Y, only heading changes
                Pose turnTarget = new Pose(current.getX(), current.getY(), targetHeading);

                PathChain turnChain = new PathBuilder(follower)
                        .addPath(new com.pedropathing.geometry.BezierLine(current, turnTarget))
                        .setLinearHeadingInterpolation(current.getHeading(), targetHeading)
                        .build();

                follower.followPath(turnChain);
                autoRunning = true;
            }

            // ---- B: GO TO SHOOT POSE ----
            if (bPressed) {
                Pose current = follower.getPose();

                PathChain shootChain = new PathBuilder(follower)
                        .addPath(new com.pedropathing.geometry.BezierLine(current, SHOOT_POSE))
                        .setLinearHeadingInterpolation(current.getHeading(), SHOOT_POSE.getHeading())
                        .build();

                follower.followPath(shootChain);
                autoRunning = true;
            }

            // ---- MANUAL DRIVE ONLY IF NOT AUTO ----
            if (!autoRunning) {
                drivetrain.driveMecanum(strafe, drive, turn);
            }

            // ---- auto finished? ----
            if (autoRunning && !follower.isBusy()) {
                autoRunning = false;
            }

            // ---- telemetry ----
            Pose p = follower.getPose();
            telemetry.addData("Mode", autoRunning ? "AUTO" : "MANUAL");
            telemetry.addData("X", "%.2f", p.getX());
            telemetry.addData("Y", "%.2f", p.getY());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(p.getHeading()));
            telemetry.update();
        }
    }
}