package org.firstinspires.ftc.teamcode.limeLight.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "lShape", group = "PedroPathing")
public class LShapePath extends OpMode {

    /** Create a Follower instance from Constants */
    private Follower follower;

    /** Define the triangle vertices as Poses */
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));        // Start position
    private final Pose secondPose = new Pose(24, 0, Math.toRadians(90));   // Move forward
    private final Pose thirdPose = new Pose(24, -24, Math.toRadians(45));     // Move forward-right

    /** PathChain representing the entire triangle */
    private PathChain lPath;

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */

    @Override
    public void init() {
        // Initialize follower
        follower = Constants.createFollower(hardwareMap);
        telemetry.addLine("Triangle2 Initialized. Waiting for start...");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        follower.update();

        telemetry.addLine("Triangle2 Ready");
        telemetry.addLine("Ensure space to move in a triangular pattern.");
        telemetry.update();
    }

    /** Creates the PathChain for the "triangle".*/
    @Override
    public void start() {
        follower.setStartingPose(startPose);

        // Build the triangle path
        lPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, secondPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), secondPose.getHeading())
                .addPath(new BezierLine(secondPose, thirdPose))
                .setLinearHeadingInterpolation(secondPose.getHeading(), thirdPose.getHeading())
                .addPath(new BezierLine(thirdPose, secondPose))
                .setLinearHeadingInterpolation(thirdPose.getHeading(), secondPose.getHeading())
                .addPath(new BezierLine(secondPose, startPose))
                .setLinearHeadingInterpolation(secondPose.getHeading(), startPose.getHeading())
                .build();


        // Start following the path
        follower.followPath(lPath);
    }

    @Override
    public void loop() {
        // Continuously update follower
        follower.update();

        // If finished, restart the triangle loop
        if (follower.atParametricEnd()) {
            follower.followPath(lPath, true);
        }

        // Telemetry for debugging
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Following Path", !follower.atParametricEnd());
        telemetry.update();
    }
}