package org.firstinspires.ftc.teamcode.decode.pedroPathing.StatesAutosV2.StatesBlueAutos.BlueClose.BlueClose12;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class GeneratedPathsBlue12BallCloseV2 {

    private final Follower follower;

    // ---------------- START POSE ----------------
    public static final Pose START_POSE =
            new Pose(30.1845386533666, 135.5561097256858, Math.toRadians(0));

    public GeneratedPathsBlue12BallCloseV2(Follower follower) {
        this.follower = follower;
    }

    // ---------------- PATHS ----------------

    // Path 1
    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        START_POSE,
                        new Pose(47.24189526184537, 84, Math.toRadians(180))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(180)
                )
                .build();
    }

    // Path 2
    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(47.24189526184537, 84, Math.toRadians(180)),
                        new Pose(41.2, 84, Math.toRadians(180))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(180)
                )
                .build();
    }

    // Path 3
    public PathChain intake1() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(41.2, 84, Math.toRadians(180)),
                        new Pose(9, 84, Math.toRadians(180))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(180)
                )
                .build();
    }

    // Path 4
    public PathChain gate() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(9, 84, Math.toRadians(180)),
                        new Pose(15.75311720698252, 70.42394014962593, Math.toRadians(90))
                ))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }

    public PathChain intake1ToShoot2() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        new Pose(15, 84, Math.toRadians(180)),
                        new Pose(23.24189526184537, 84, Math.toRadians(180))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(180)
                )
                .build();
    }

    // Path 5
    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(15.75311720698252, 70.42394014962593, Math.toRadians(180)),
                        new Pose(41.24189526184537, 84, Math.toRadians(180))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(180)
                )
                .build();
    }

    // Path 6
    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        new Pose(47.24189526184537, 84, Math.toRadians(180)),
                        new Pose(56.30324189526183, 59.87032418952615),
                        new Pose(43.2, 59.6, Math.toRadians(180))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(180)
                )
                .build();
    }

    // Path 7
    public PathChain intake2() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(43.2, 59.6, Math.toRadians(180)),
                        new Pose(9, 59.6, Math.toRadians(180))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(180)
                )
                .build();
    }

    // Path 8
    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        new Pose(9, 59.6, Math.toRadians(180)),
                        new Pose(47.66446384039901, 59.10174563591022),
                        new Pose(47.24189526184537, 84, Math.toRadians(180))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(180)
                )
                .build();
    }

    // Path 9
    public PathChain toIntake3() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        new Pose(47.24189526184537, 84, Math.toRadians(180)),
                        new Pose(60.42518703241895, 35.693266832917686),
                        new Pose(41.2, 35.5, Math.toRadians(180))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(180)
                )
                .build();
    }

    // Path 10
    public PathChain intake3() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(41.2, 35.5, Math.toRadians(180)),
                        new Pose(18.9, 35.5, Math.toRadians(180))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(180)
                )
                .build();
    }

    // Path 11
    public PathChain shoot4() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        new Pose(18.9, 35.5, Math.toRadians(180)),
                        new Pose(47.35773067331668, 34.40835411471322),
                        new Pose(47.24189526184537, 84, Math.toRadians(180))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(180)
                )
                .setReversed()
                .build();
    }

    // Path 12
    public PathChain leave() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(47.24189526184537, 84, Math.toRadians(90)),
                        new Pose(47.24189526184537, 72.44389027431421, Math.toRadians(90))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(90)
                )
                .build();
    }
}
