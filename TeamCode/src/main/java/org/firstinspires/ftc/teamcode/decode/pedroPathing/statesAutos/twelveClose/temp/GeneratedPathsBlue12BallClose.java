package org.firstinspires.ftc.teamcode.decode.pedroPathing.statesAutos.twelveClose.temp;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;

public class GeneratedPathsBlue12BallClose {

    private final Follower follower;

    // ---------------- START POSE ----------------

    //public static final Pose START_POSE = new Pose(118.157, 128.629, Math.toRadians(45));
    public static final Pose START_POSE =
            new Pose(108+10, 130.2926713735558-8, Math.toRadians(180)).mirror();

    public GeneratedPathsBlue12BallClose(Follower follower) {
        this.follower = follower;
    }

    // ---------------- PATHS ----------------

    public PathChain scan() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                START_POSE,
                                //new Pose(91.371, 102.034, Math.toRadians(45))
                                new Pose(107, 106, Math.toRadians(135)).mirror()
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(180-180),
                        Math.toRadians(180-135)
                )
                .build();
    }

    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(107, 106, Math.toRadians(135)).mirror(),
                                //new Pose(91.371, 102.034, Math.toRadians(45))
                                new Pose(96, 94, Math.toRadians(35)).mirror()
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(180-135),
                        Math.toRadians(180-35)
                )
                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(96, 94, Math.toRadians(35)).mirror(),
                                new Pose(84.88846763536056, 84.28908937098198).mirror(),
                                new Pose(103.2, 75.5, Math.toRadians(0)).mirror()
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(180-35),
                        Math.toRadians(180-0)
                )
                .build();
    }

    public PathChain intake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(103.2, 75.5, Math.toRadians(0)).mirror(),
                                new Pose(127, 75.5, Math.toRadians(0)).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-0))
                .build();
    }

    public PathChain gate() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(127, 75.5, Math.toRadians(0)).mirror(),
                                //new Pose(111.86034912718205, 80.07980049875314),
                                new Pose(130, 73, Math.toRadians(90)).mirror()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-90))
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
                                new Pose(127, 75, Math.toRadians(90)).mirror(),
                                new Pose(105, 91, Math.toRadians(30)).mirror()
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(180-90),
                        Math.toRadians(180-30)
                )
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(105, 91, Math.toRadians(30)).mirror(),
                                new Pose(68.8707865168539, 60.123595505617985).mirror(),
                                // new Pose(89.75910487372168, 65.29946200112505),
                                new Pose(104.2, 52, Math.toRadians(0)).mirror()
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(180-30),
                        Math.toRadians(180-0)
                )
                .build();
    }

    public PathChain intake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(104.2, 52, Math.toRadians(0)).mirror(),
                                new Pose(128, 52, Math.toRadians(0)).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-0))
                .build();
    }

    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(128, 52, Math.toRadians(0)).mirror(),
                                new Pose(109.05707981278215, 62.47068594135381).mirror(),
                                new Pose(96, 94, Math.toRadians(39)).mirror()
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(180-0),
                        Math.toRadians(180-39)
                )
                .build();
    }

    public PathChain toIntake3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(96, 94, Math.toRadians(39)).mirror(),
                                new Pose(68.58988764044946, 35.7).mirror(),
                                // new Pose(83.56293000245203, 52.706418485237485),
                                new Pose(104.2, 30, Math.toRadians(0)).mirror()
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(180-39),
                        Math.toRadians(180-0)
                )
                .build();
    }

    public PathChain intake3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(104.2, 30, Math.toRadians(0)).mirror(),
                                new Pose(130, 30, Math.toRadians(0)).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-0))
                .build();
    }

    public PathChain shoot4() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(130, 30, Math.toRadians(0)).mirror(),
                                new Pose(96, 94, Math.toRadians(30)).mirror()
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(180-0),
                        Math.toRadians(180-30)
                )
                .build();
    }

    public PathChain leave() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(96, 94, Math.toRadians(30)).mirror(),
                                new Pose(117.3496136071887, 82.19084724005137, Math.toRadians(90)).mirror()
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(180-30),
                        Math.toRadians(180-90)
                )
                .build();
    }
}