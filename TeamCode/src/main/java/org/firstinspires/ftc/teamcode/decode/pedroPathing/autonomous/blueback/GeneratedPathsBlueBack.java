package org.firstinspires.ftc.teamcode.decode.pedroPathing.autonomous.blueback;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;

public class GeneratedPathsBlueBack {

    private final Follower follower;

    // Central definition of the starting pose
    public static final Pose START_POSE = new Pose(
            144-116.6988847583643,  // X
            128.83271375464685, // Y
            Math.toRadians(180-225) // Heading
    );

    public GeneratedPathsBlueBack(Follower follower) {
        this.follower = follower;
    }
    public PathChain scan() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(START_POSE, new Pose(54, 101))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-225), Math.toRadians(60))
                .build();
    }

    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(54,101), new Pose(59, 81))
                )
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(128))
                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(59, 81), new Pose(144-101.353, 83))
                )
                .setLinearHeadingInterpolation(Math.toRadians(128), Math.toRadians(180-0))
                .build();
    }

    public PathChain intakeball1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(144-101.353, 83), new Pose(144-104, 83))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-0))
                .build();
    }

    public PathChain intakeball2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(144-104, 83), new Pose(144-109, 83))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-0))
                .build();
    }

    public PathChain intakeball3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(144-109, 83), new Pose(144-120, 83))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-0))
                .build();
    }



    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(144-121.900, 83), new Pose(59, 81))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(136.67))
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(144-90, 100), new Pose(144-101.353, 59.400))
                )
                .setLinearHeadingInterpolation(Math.toRadians(136.67), Math.toRadians(180-0))
                .build();
    }

    public PathChain intakeball4() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(144-101.353, 59.400), new Pose(144-106.5, 59.400))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-0))
                .build();
    }
    public PathChain intakeball5() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(144-106.5, 59.400), new Pose(144-112, 59.400))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-0))
                .build();
    }
    public PathChain intakeball6() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(144-112, 59.400), new Pose(144-120, 59.400))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-0))
                .build();
    }

    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(144-123, 59.400), new Pose(59, 81))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-45))
                .build();
    }
}
