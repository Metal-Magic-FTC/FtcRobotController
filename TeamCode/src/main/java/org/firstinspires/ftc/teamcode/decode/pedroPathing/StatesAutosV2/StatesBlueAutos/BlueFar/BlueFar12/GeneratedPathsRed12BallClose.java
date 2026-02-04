package org.firstinspires.ftc.teamcode.decode.pedroPathing.StatesAutosV2.StatesBlueAutos.BlueFar.BlueFar12;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class GeneratedPathsRed12BallClose {

    private final Follower follower;

    // ---------------- START POSE ----------------

    //public static final Pose START_POSE = new Pose(118.157, 128.629, Math.toRadians(45));
    public static final Pose START_POSE =
            new Pose(56, 8, Math.toRadians(90));

    Pose blueStart = START_POSE.mirror();

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
                                new Pose(58, 24, Math.toRadians(125))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(90),
                        Math.toRadians(125)
                )
                .build();
    }



    public PathChain intake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(58, 24, Math.toRadians(125)),
                                new Pose(42, 36, Math.toRadians(180))
                        )
                )
                .setLinearHeadingInterpolation(125, 180)
                .build();
    }

    public PathChain intakeball1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(42, 36, Math.toRadians(180)),
                                new Pose(28, 36, Math.toRadians(180))
                        )
                )
                .setLinearHeadingInterpolation(125, 180)
                .build();
    }

    public PathChain intakeball2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(28, 36, Math.toRadians(180)),
                                new Pose(22, 36, Math.toRadians(180))
                        )
                )
                .setLinearHeadingInterpolation(125, 180)
                .build();
    }

    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(127, 78, Math.toRadians(90)).mirror(),
                                new Pose(102.7714489864318, 101.06945056241283, Math.toRadians(43))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(90),
                        Math.toRadians(43)
                )
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(102.7714489864318, 101.06945056241283, Math.toRadians(43)),
                                new Pose(68.8707865168539, 60.123595505617985),
                                // new Pose(89.75910487372168, 65.29946200112505),
                                new Pose(104.2, 55, Math.toRadians(0))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(43),
                        Math.toRadians(0)
                )
                .build();
    }

    public PathChain intake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(104.2, 55, Math.toRadians(0)),
                                new Pose(126, 55, Math.toRadians(0))
                        )
                )
                .setLinearHeadingInterpolation(0, 0)
                .build();
    }

    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(126, 55, Math.toRadians(0)),
                                new Pose(109.05707981278215, 62.47068594135381),
                                new Pose(107.26022704129466, 104.66047300630309, Math.toRadians(43))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(45)
                )
                .build();
    }

    public PathChain toIntake3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(107.26022704129466, 104.66047300630309, Math.toRadians(43)),
                                new Pose(68.58988764044946, 35.7),
                                // new Pose(83.56293000245203, 52.706418485237485),
                                new Pose(104.2, 30, Math.toRadians(0))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(45),
                        Math.toRadians(0)
                )
                .build();
    }

    public PathChain intake3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(104.2, 30, Math.toRadians(0)),
                                new Pose(128, 30, Math.toRadians(0))
                        )
                )
                .setLinearHeadingInterpolation(0, 0)
                .build();
    }

    public PathChain shoot4() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(128, 30, Math.toRadians(0)),
                                new Pose(107.26022704129466, 104.66047300630309, Math.toRadians(45))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(45)
                )
                .build();
    }

    public PathChain leave() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(100.42876636713736, 112.57058536585366, Math.toRadians(30)),
                                new Pose(117.3496136071887, 82.19084724005137, Math.toRadians(90))
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(45),
                        Math.toRadians(90)
                )
                .build();
    }
}