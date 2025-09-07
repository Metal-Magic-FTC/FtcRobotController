package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Triangle2", group = "PedroPathing")
public class Triangle2 extends OpMode {

    /** Create a Follower instance from Constants */
    private Follower follower;

    /** Define the triangle vertices as Poses */
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));        // Start position
    private final Pose secondPose = new Pose(24, -24, Math.toRadians(90));   // Move forward-right
    private final Pose thirdPose = new Pose(24, 24, Math.toRadians(45));     // Move forward-left

    /** PathChain representing the entire triangle */
    private PathChain trianglePath;

    @Override
    public void init() {
        // Initialize follower
        follower = Constants.createFollower(hardwareMap);

        telemetry.addLine("Triangle2 Initialized. Waiting for start...");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Keep updating follower before start to get current position
        follower.update();

        telemetry.addLine("Triangle2 Ready");
        telemetry.addLine("Ensure space to move in a triangular pattern.");
        telemetry.update();
    }

    @Override
    public void start() {
        // Set the robot's starting pose
        follower.setStartingPose(startPose);

        // Build the triangle path
        trianglePath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, secondPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), secondPose.getHeading())
                .addPath(new BezierLine(secondPose, thirdPose))
                .setLinearHeadingInterpolation(secondPose.getHeading(), thirdPose.getHeading())
                .addPath(new BezierLine(thirdPose, startPose))
                .setLinearHeadingInterpolation(thirdPose.getHeading(), startPose.getHeading())
                .build();

        // Start following the path
        follower.followPath(trianglePath);
    }

    @Override
    public void loop() {
        // Continuously update follower
        follower.update();

        // If finished, restart the triangle loop
        if (follower.atParametricEnd()) {
            follower.followPath(trianglePath, true);
        }

        // Telemetry for debugging
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Following Path", !follower.atParametricEnd());
        telemetry.update();
    }
}