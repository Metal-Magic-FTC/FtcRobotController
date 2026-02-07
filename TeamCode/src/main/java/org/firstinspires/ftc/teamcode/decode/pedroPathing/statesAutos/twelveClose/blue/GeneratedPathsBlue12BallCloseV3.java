package org.firstinspires.ftc.teamcode.decode.pedroPathing.statesAutos.twelveClose.blue;

import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.pedropathing.follower.Follower;

public class GeneratedPathsBlue12BallCloseV3 {

    private final Follower follower;

    // Mirror across Y-axis:
    // X -> 144 - X
    // Heading -> PI - heading

    public static final Pose START_POSE =
            new Pose(144 - (108 + 10), 130.2926713735558 - 8, Math.toRadians(0));

    public GeneratedPathsBlue12BallCloseV3(Follower follower) {
        this.follower = follower;
    }

    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        START_POSE,
                        new Pose(144 - 96, 94, Math.toRadians(145))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(145)
                )
                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        new Pose(144 - 96, 94, Math.toRadians(145)),
                        new Pose(144 - 84.888, 84.289),
                        new Pose(144 - 103.2, 79, Math.toRadians(180))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(145),
                        Math.toRadians(180)
                )
                .build();
    }

    public PathChain intake1() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(144 - 103.2, 79, Math.toRadians(180)),
                        new Pose(144 - 128, 79, Math.toRadians(180))
                ))
                .build();
    }

    public PathChain gate() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        new Pose(144 - 128, 79, Math.toRadians(180)),
                        new Pose(144 - 129, 76, Math.toRadians(90))
                ))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }

    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(144 - 127, 78, Math.toRadians(90)),
                        new Pose(144 - 105, 91, Math.toRadians(150))
                ))
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        new Pose(144 - 105, 91, Math.toRadians(150)),
                        new Pose(144 - 68.87, 60.12),
                        new Pose(144 - 104.2, 55, Math.toRadians(180))
                ))
                .build();
    }

    public PathChain intake2() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(144 - 104.2, 55, Math.toRadians(180)),
                        new Pose(144 - 128, 55, Math.toRadians(180))
                ))
                .build();
    }

    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        new Pose(144 - 128, 55, Math.toRadians(180)),
                        new Pose(144 - 109.05, 62.47),
                        new Pose(144 - 96, 94, Math.toRadians(141))
                ))
                .build();
    }

    public PathChain toIntake3() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        new Pose(144 - 96, 94, Math.toRadians(141)),
                        new Pose(144 - 68.59, 35.7),
                        new Pose(144 - 104.2, 30, Math.toRadians(180))
                ))
                .build();
    }

    public PathChain intake3() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(144 - 104.2, 30, Math.toRadians(180)),
                        new Pose(144 - 130, 30, Math.toRadians(180))
                ))
                .build();
    }

    public PathChain shoot4() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(144 - 130, 30, Math.toRadians(180)),
                        new Pose(144 - 96, 94, Math.toRadians(150))
                ))
                .build();
    }

    public PathChain leave() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(144 - 96, 94, Math.toRadians(150)),
                        new Pose(144 - 117.35, 82.19, Math.toRadians(90))
                ))
                .build();
    }
}