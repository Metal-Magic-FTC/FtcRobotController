package org.firstinspires.ftc.teamcode.decode.pedroPathing.statesAutos.twelveClose.red;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;

public class GeneratedPathsRed12BallClose {

    private final Follower follower;

    // ---------------- START POSE ----------------

    //public static final Pose START_POSE = new Pose(118.157, 128.629, Math.toRadians(45));
    public static final Pose START_POSE =
            new Pose(108+10, 130.2926713735558-8, Math.toRadians(180));

    public GeneratedPathsRed12BallClose(Follower follower) {
        this.follower = follower;
    }

    // ---------------- PATHS ----------------

    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                START_POSE,
                                //new Pose(91.371, 102.034, Math.toRadians(45))
                                new Pose(96, 94, Math.toRadians(41))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(41)
                )
                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(96, 94, Math.toRadians(45)),
                                new Pose(84.88846763536056, 84.28908937098198),
                                new Pose(103.2, 79, Math.toRadians(0))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(41),
                        Math.toRadians(0)
                )
                .build();
    }

    public PathChain intake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(103.2, 79, Math.toRadians(0)),
                                new Pose(128, 79, Math.toRadians(0))
                        )
                )
                .setLinearHeadingInterpolation(0, 0)
                .build();
    }

    public PathChain gate() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(128, 79, Math.toRadians(0)),
                                //new Pose(111.86034912718205, 80.07980049875314),
                                new Pose(129, 76, Math.toRadians(90))
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

    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(127, 78, Math.toRadians(90)),
                                new Pose(96, 94, Math.toRadians(43))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(90),
                        Math.toRadians(38)
                )
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(96, 94, Math.toRadians(43)),
                                new Pose(68.8707865168539, 60.123595505617985),
                                // new Pose(89.75910487372168, 65.29946200112505),
                                new Pose(104.2, 55, Math.toRadians(0))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(38),
                        Math.toRadians(0)
                )
                .build();
    }

    public PathChain intake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(104.2, 55, Math.toRadians(0)),
                                new Pose(128, 55, Math.toRadians(0))
                        )
                )
                .setLinearHeadingInterpolation(0, 0)
                .build();
    }

    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(128, 55, Math.toRadians(0)),
                                new Pose(109.05707981278215, 62.47068594135381),
                                new Pose(96, 94, Math.toRadians(43))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(39)
                )
                .build();
    }

    public PathChain toIntake3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(96, 94, Math.toRadians(39)),
                                new Pose(68.58988764044946, 35.7),
                                // new Pose(83.56293000245203, 52.706418485237485),
                                new Pose(104.2, 30, Math.toRadians(0))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(39),
                        Math.toRadians(0)
                )
                .build();
    }

    public PathChain intake3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(104.2, 30, Math.toRadians(0)),
                                new Pose(130, 30, Math.toRadians(0))
                        )
                )
                .setLinearHeadingInterpolation(0, 0)
                .build();
    }

    public PathChain shoot4() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(130, 30, Math.toRadians(0)),
                                new Pose(96, 94, Math.toRadians(39))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(39)
                )
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
                        Math.toRadians(39),
                        Math.toRadians(90)
                )
                .build();
    }
}