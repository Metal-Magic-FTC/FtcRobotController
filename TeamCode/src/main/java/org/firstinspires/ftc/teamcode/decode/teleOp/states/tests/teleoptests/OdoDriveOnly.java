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

    // ---- TUNING (change if needed) ----
    private static final double POS_TOLERANCE = 2.0;                 // inches
    private static final double HEADING_TOLERANCE = Math.toRadians(3); // radians

    private Follower follower;
    private CustomMecanumDrive drivetrain;

    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevY = false;

    private boolean autoRunning = false;

    // Track what our auto target is so we can end early when close
    private Pose autoTargetPose = null;

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
            boolean yPressed = gamepad1.y && !prevY;

            prevA = gamepad1.a;
            prevB = gamepad1.b;
            prevY = gamepad1.y;

            // ---- Y: FORCE MANUAL MODE ----
            if (yPressed) {
                follower.breakFollowing();
                autoRunning = false;
                autoTargetPose = null;
            }

            // Driver override cancels auto (sticks)
            boolean driverOverride =
                    Math.abs(drive) > 0.05 ||
                            Math.abs(strafe) > 0.05 ||
                            Math.abs(turn) > 0.05;

            if (autoRunning && driverOverride) {
                follower.breakFollowing();
                autoRunning = false;
                autoTargetPose = null;
            }

            // ---- A: FACE START (TURN ONLY) ----
            if (aPressed) {
                Pose current = follower.getPose();

                double dx = START_POSE.getX() - current.getX();
                double dy = START_POSE.getY() - current.getY();
                double targetHeading = Math.atan2(dy, dx);

                // Hold current position, change only heading
                Pose turnOnlyTarget = new Pose(current.getX(), current.getY(), targetHeading);

                follower.holdPoint(turnOnlyTarget);

                autoRunning = true;
                autoTargetPose = turnOnlyTarget;
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
                autoTargetPose = SHOOT_POSE;
            }

            // ---- AUTO END CONDITIONS ----
            if (autoRunning) {
                Pose current = follower.getPose();

                boolean finished = !follower.isBusy();
                boolean closeEnough = (autoTargetPose != null) && isCloseToTarget(current, autoTargetPose);

                if (finished || closeEnough) {
                    follower.breakFollowing();
                    autoRunning = false;
                    autoTargetPose = null;
                }
            }

            // ---- MANUAL DRIVE ONLY IF NOT AUTO ----
            if (!autoRunning) {
                drivetrain.driveMecanum(strafe, drive, turn);
            }

            // ---- telemetry ----
            Pose p = follower.getPose();
            telemetry.addData("Mode", autoRunning ? "AUTO" : "MANUAL");
            telemetry.addData("X", "%.2f", p.getX());
            telemetry.addData("Y", "%.2f", p.getY());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(p.getHeading()));
            telemetry.addData("Target", autoTargetPose == null ? "none" :
                    String.format("(%.1f, %.1f, %.1f°)",
                            autoTargetPose.getX(),
                            autoTargetPose.getY(),
                            Math.toDegrees(autoTargetPose.getHeading())));
            telemetry.update();
        }
    }

    // --- helper methods ---
    private boolean isCloseToTarget(Pose current, Pose target) {
        double dx = target.getX() - current.getX();
        double dy = target.getY() - current.getY();
        double dist = Math.hypot(dx, dy);

        double headingError = angleWrap(target.getHeading() - current.getHeading());

        return dist <= POS_TOLERANCE && Math.abs(headingError) <= HEADING_TOLERANCE;
    }

    private double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2.0 * Math.PI;
        while (radians < -Math.PI) radians += 2.0 * Math.PI;
        return radians;
    }
}