package org.firstinspires.ftc.teamcode.decode.pedroPathing.StatesAutosV2.StatesRedAutos.RedClose.RedClose12;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class GeneratedPathsRed12BallCloseV2 {

    private final Follower follower;

    // ---------------- START POSE ----------------
    public static final Pose START_POSE =
            new Pose(113.8154613466334, 135.5561097256858, Math.toRadians(180));

    public GeneratedPathsRed12BallCloseV2(Follower follower) {
        this.follower = follower;
    }

    // ---------------- PATHS ----------------

    // Path 1
    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        START_POSE,
                        new Pose(96.75810473815463, 84, Math.toRadians(0))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(0)
                )
                .build();
    }

    // Path 2
    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(96.75810473815463, 84, Math.toRadians(0)),
                        new Pose(102.8, 84, Math.toRadians(0))
                ))
                .setLinearHeadingInterpolation(0, 0)
                .build();
    }

    // Path 3
    public PathChain intake1() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(102.8, 84, Math.toRadians(0)),
                        new Pose(135, 84, Math.toRadians(0))
                ))
                .setLinearHeadingInterpolation(0, 0)
                .build();
    }

    // Path 4
    public PathChain gate() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(135, 84, Math.toRadians(0)),
                        new Pose(128.24688279301748, 70.42394014962593, Math.toRadians(-90))
                ))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();
    }

    public PathChain intake1ToShoot2() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        new Pose(129, 84, Math.toRadians(0)),
                        new Pose(116.75810473815463, 84, Math.toRadians(0))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(0)
                )
                .build();
    }

    // Path 5
    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(128.24688279301748, 70.42394014962593, Math.toRadians(0)),
                        new Pose(102.75810473815463, 84, Math.toRadians(0))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(0)
                )
                .build();
    }

    // Path 6
    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        new Pose(96.75810473815463, 84, Math.toRadians(0)),
                        new Pose(87.69675810473817, 59.87032418952615),
                        new Pose(100.8, 59.6, Math.toRadians(0))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(0)
                )
                .build();
    }

    // Path 7
    public PathChain intake2() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(100.8, 59.6, Math.toRadians(0)),
                        new Pose(135, 59.6, Math.toRadians(0))
                ))
                .setLinearHeadingInterpolation(0, 0)
                .build();
    }

    // Path 8
    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        new Pose(135, 59.6, Math.toRadians(0)),
                        new Pose(96.33553615960099, 59.10174563591022),
                        new Pose(96.75810473815463, 84, Math.toRadians(0))
                ))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(0)
                )
                .build();
    }

    // Path 9
    public PathChain toIntake3() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        new Pose(96.75810473815463, 84, Math.toRadians(0)),
                        new Pose(83.57481296758105, 35.693266832917686),
                        new Pose(102.8, 35.5, Math.toRadians(0))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(0)
                )
                .build();
    }

    // Path 10
    public PathChain intake3() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(102.8, 35.5, Math.toRadians(0)),
                        new Pose(125.1, 35.5, Math.toRadians(0))
                ))
                .setLinearHeadingInterpolation(0, 0)
                .build();
    }

    // Path 11
    public PathChain shoot4() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        new Pose(125.1, 35.5, Math.toRadians(0)),
                        new Pose(96.64226932668332, 34.40835411471322),
                        new Pose(96.75810473815463, 84, Math.toRadians(0))
                ))
                //.setTangentHeadingInterpolation()
                //.setLinearHeadingInterpolation(0, -90)
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(0)
                )
                .setReversed()
                .build();
    }

    // Path 12
    public PathChain leave() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(96.75810473815463, 84, Math.toRadians(-90)),
                        new Pose(96.75810473815463, 72.44389027431421, Math.toRadians(-90))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(-90)
                )
                .build();
    }
}
