package org.firstinspires.ftc.teamcode.decode.pedroPathing.autonomousv1.redfront;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;

public class GeneratedPathsRedFront {

    private final Follower follower;

    // Central definition of the starting pose
    public static final Pose START_POSE = new Pose(
            86,  // X
            9.196261682242984, // Y
            Math.toRadians(90) // Heading
    );

    public GeneratedPathsRedFront(Follower follower) {
        this.follower = follower;
    }

    public PathChain move () {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(START_POSE, new Pose(86, 25))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();
    }


    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(START_POSE, new Pose(84, 13))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(71))
                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(84, 13), new Pose(91, 34))
                )
                .setLinearHeadingInterpolation(Math.toRadians(71), Math.toRadians(0))
                .build();
    }

    public PathChain intakeball1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(91, 34), new Pose(103, 34))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public PathChain intakeball2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(103, 34), new Pose(109, 34))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public PathChain intakeball3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(109, 34), new Pose(120, 34))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(120, 34), new Pose(84, 13))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68))
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(84, 13), new Pose(101.353, 60.112))
                )
                .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(0))
                .build();
    }

    public PathChain intakeball4() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(101.353, 60.112), new Pose(104, 60.112))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public PathChain intakeball5() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(104, 60.112), new Pose(109, 60.112))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public PathChain intakeball6() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(109, 60.112), new Pose(120, 60.112))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }


    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(130.991, 60.112), new Pose(84, 13))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68))
                .build();
    }
}
