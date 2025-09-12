package org.firstinspires.ftc.teamcode.decode.teleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "AutoV0", group = "PedroPathing")
public class AutoV0 extends OpMode {

    /** Create a Follower instance from Constants */
    private Follower follower;

    /** Define the triangle vertices as Poses */
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));        // Start position
    private final Pose secondPose = new Pose(24, -30, Math.toRadians(90));   // Move forward-right
    private final Pose thirdPose = new Pose(24, -20, Math.toRadians(90));     // Move forward-left
    private final Pose fourthPose = new Pose(0, -30, Math.toRadians(135));

    /** PathChain representing the entire triangle */
    private PathChain autoPath;

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
        autoPath = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, secondPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), secondPose.getHeading())
                .addPath(new BezierLine(secondPose, thirdPose))
                .setLinearHeadingInterpolation(secondPose.getHeading(), thirdPose.getHeading())
                .addPath(new BezierCurve(thirdPose, fourthPose))
                .setLinearHeadingInterpolation(thirdPose.getHeading(), fourthPose.getHeading())
                .addPath(new BezierLine(fourthPose, startPose))
                .setLinearHeadingInterpolation(fourthPose.getHeading(), startPose.getHeading())
                .build();



//        follower.followPath(autoPath);

        // Start following the path
        follower.followPath(autoPath);
    }

    @Override
    public void loop() {
        // Continuously update follower
        follower.update();

        // If finished, restart the triangle loop
        if (follower.atParametricEnd()) {
//            follower.followPath(autoPath, true);
        }

        // Telemetry for debugging
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Following Path", !follower.atParametricEnd());
        telemetry.update();
    }
}