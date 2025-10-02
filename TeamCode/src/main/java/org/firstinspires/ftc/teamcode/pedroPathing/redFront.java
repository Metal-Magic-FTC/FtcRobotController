package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "redFront", group = "autosV1")
public class redFront extends OpMode {

    /** Create a Follower instance from Constants */
    private Follower follower;

    /** Define the path vertices as Poses */
    private final Pose startPose = new Pose(82, 8, Math.toRadians(270));        // Start position
    private final Pose preShoot = new Pose(86, 19, Math.toRadians(245));   // Move forward-right
    private final Pose row1Pre = new Pose(110, 35, Math.toRadians(0));     // Move forward-left
    private final Pose row1Post = new Pose(130, 35, Math.toRadians(0));     // Move forward-left
    private final Pose row2Pre = new Pose(110, 60, Math.toRadians(0));
    private final Pose row2Post = new Pose(130, 60, Math.toRadians(0));

    /** PathChain representing the entire triangle */
    private PathChain defaultAutoPath;

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
        defaultAutoPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, preShoot))
                .setLinearHeadingInterpolation(startPose.getHeading(), preShoot.getHeading())
                .addPath(new BezierLine(preShoot, row1Pre))
                .setLinearHeadingInterpolation(preShoot.getHeading(), row1Pre.getHeading())
                .addPath(new BezierLine(row1Pre, row1Post))
                .setLinearHeadingInterpolation(row1Pre.getHeading(), row1Post.getHeading())
                .addPath(new BezierLine(row1Post, preShoot))
                .setLinearHeadingInterpolation(row1Post.getHeading(), preShoot.getHeading())
                .addPath(new BezierLine(preShoot, row2Pre))
                .setLinearHeadingInterpolation(preShoot.getHeading(), row2Pre.getHeading())
                .addPath(new BezierLine(row2Pre, row2Post))
                .setLinearHeadingInterpolation(row2Pre.getHeading(), row2Post.getHeading())
                .addPath(new BezierLine(row2Post, preShoot))
                .setLinearHeadingInterpolation(row2Post.getHeading(), preShoot.getHeading())
                .build();


        // Start following the path
        follower.followPath(defaultAutoPath);
    }

    @Override
    public void loop() {
        // Continuously update follower
        follower.update();

        // If finished, restart the triangle loop
        if (follower.atParametricEnd()) {
            follower.followPath(defaultAutoPath, true);
        }

        // Telemetry for debugging
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Following Path", !follower.atParametricEnd());
        telemetry.update();
    }
}