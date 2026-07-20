package org.firstinspires.ftc.teamcode.decode.pedroPathing.statesAutos.twelveClose.red;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;

public class GeneratedPathsRed12BallCloseV3 {

    private final Follower follower;

    // ---------------- START POSE ----------------

    //public static final Pose START_POSE = new Pose(118.157, 128.629, Math.toRadians(45));
    public static final Pose START_POSE =
            new Pose(108+10, 130.2926713735558-8, Math.toRadians(180));

    public GeneratedPathsRed12BallCloseV3(Follower follower) {
        this.follower = follower;
    }

    // ---------------- PATHS ----------------

    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                START_POSE,
                                //new Pose(91.371, 102.034, Math.toRadians(45))
                                new Pose(90, 14, Math.toRadians(35))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(90),
                        Math.toRadians(35)
                )
                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(90, 14, Math.toRadians(35)),
                                //new Pose(111.86034912718205, 80.07980049875314),
                                new Pose(96, 34, Math.toRadians(0))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(90),
                        Math.toRadians(35)
                )
                .build();
    }

    public PathChain intake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(96, 34, Math.toRadians(0)),
                                new Pose(128, 34, Math.toRadians(0))
                        )
                )
                .setLinearHeadingInterpolation(0, 0)
                .build();
    }

    public PathChain wallBump() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(90, 14, Math.toRadians(35)),
                                //new Pose(111.86034912718205, 80.07980049875314),
                                new Pose(129, 77, Math.toRadians(90))
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
//                .setLinearHeadingInterpolation(
//                        Math.toRadians(0),
//                        Math.toRadians(90)
//                )
//                // rotate in place
//                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }



    public PathChain leave() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(96, 94, Math.toRadians(30)),
                                new Pose(117.3496136071887, 82.19084724005137, Math.toRadians(90))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(30),
                        Math.toRadians(90)
                )
                .build();
    }
}