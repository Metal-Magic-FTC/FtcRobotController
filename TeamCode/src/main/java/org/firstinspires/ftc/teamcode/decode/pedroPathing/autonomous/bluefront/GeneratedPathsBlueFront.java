package org.firstinspires.ftc.teamcode.decode.pedroPathing.autonomous.bluefront;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;

public class GeneratedPathsBlueFront {

    private final Follower follower;

    // Central definition of the starting pose
    public static final Pose START_POSE = new Pose(
            55.85046728971962,  // X
            9.196261682242984, // Y
            Math.toRadians(90) // Heading
    );

    public GeneratedPathsBlueFront(Follower follower) {
        this.follower = follower;
    }

    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(START_POSE, new Pose(55.85046728971962, 13))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(99))
                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(55.85046728971962, 13), new Pose(42.647, 38))
                )
                .setLinearHeadingInterpolation(Math.toRadians(99), Math.toRadians(180))
                .build();
    }

    public PathChain intakeball1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(42.647, 38), new Pose(40, 38))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }
    public PathChain intakeball2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(40, 38), new Pose(35, 38))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public PathChain intakeball3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(35, 38), new Pose(24, 38))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(24, 38), new Pose(55.85046728971962, 13))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(105))
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(55.85046728971962, 13), new Pose(144-101.353, 60.112))
                )
                .setLinearHeadingInterpolation(Math.toRadians(99), Math.toRadians(180))
                .build();
    }

    public PathChain intakeball4() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(144-101.353, 59.400), new Pose(144-106.5, 60.112))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public PathChain intakeball5() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(144-106.5, 59.400), new Pose(144-112, 60.112))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }
    public PathChain intakeball6() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(144-112, 59.400), new Pose(144-120, 60.112))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }


    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(144-130.991, 60.112), new Pose(55.85046728971962, 13))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(99))
                .build();
    }
}
