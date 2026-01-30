package org.firstinspires.ftc.teamcode.decode.pedroPathing.statesAutos.twelve.blue;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;

public class GeneratedPathsBlue12BallClose {

    private final Follower follower;

    public static final Pose START_POSE =
            new Pose(36, 130.2926713735558, Math.toRadians(0));

    public GeneratedPathsBlue12BallClose(Follower follower) {
        this.follower = follower;
    }

    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                START_POSE,
                                new Pose(36.73977295870534, 104.66047300630309, Math.toRadians(137))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(137)
                )
                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(36.73977295870534, 104.66047300630309, Math.toRadians(137)),
                                new Pose(40.8, 79.809, Math.toRadians(180))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(137),
                        Math.toRadians(180)
                )
                .build();
    }

    public PathChain intake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(40.8, 79.809, Math.toRadians(180)),
                                new Pose(18, 79, Math.toRadians(180))
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public PathChain gate() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(18, 79, Math.toRadians(180)),
                                new Pose(16, 78, Math.toRadians(90))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(90)
                )
                .build();
    }

    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(17, 78, Math.toRadians(90)),
                                new Pose(36.73977295870534, 104.66047300630309, Math.toRadians(139))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(90),
                        Math.toRadians(139)
                )
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(36.73977295870534, 104.66047300630309, Math.toRadians(139)),
                                new Pose(75.1292134831461, 60.123595505617985),
                                new Pose(39.8, 55, Math.toRadians(180))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(139),
                        Math.toRadians(180)
                )
                .build();
    }

    public PathChain intake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(39.8, 55, Math.toRadians(180)),
                                new Pose(18, 55, Math.toRadians(180))
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(18, 55, Math.toRadians(180)),
                                new Pose(34.9429201872, 62.47068594135381),
                                new Pose(36.73977295870534, 104.66047300630309, Math.toRadians(137))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(137)
                )
                .build();
    }

    public PathChain toIntake3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(36.73977295870534, 104.66047300630309, Math.toRadians(137)),
                                new Pose(75.41011235955054, 35.7),
                                new Pose(39.8, 30, Math.toRadians(180))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(137),
                        Math.toRadians(180)
                )
                .build();
    }

    public PathChain intake3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(39.8, 30, Math.toRadians(180)),
                                new Pose(18, 30, Math.toRadians(180))
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public PathChain shoot4() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(18, 30, Math.toRadians(180)),
                                new Pose(43.57123363286264, 112.57058536585366, Math.toRadians(140))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(137)
                )
                .build();
    }

    public PathChain leave() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(43.57123363286264, 112.57058536585366, Math.toRadians(140)),
                                new Pose(26.6503863928113, 82.19084724005137, Math.toRadians(90))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(137),
                        Math.toRadians(90)
                )
                .build();
    }
}
