package org.firstinspires.ftc.teamcode.decode.pedroPathing.autonomous.redmiddle;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;

public class GeneratedPathsRedMiddle {

    private final Follower follower;

    // Central definition of the starting pose
    public static final Pose START_POSE = new Pose(
            88,  // X
            136, // Y
            Math.toRadians(270) // Heading
    );

    public GeneratedPathsRedMiddle(Follower follower) {
        this.follower = follower;
    }

    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(START_POSE, new Pose(101.353, 113.844))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(225))
                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(101.353, 113.844), new Pose(101.353, 83.300))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                .build();
    }

    public PathChain intake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(101.353, 83.300), new Pose(123, 83.300))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(121.900, 83.300), new Pose(101.353, 113.844))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(101.353, 113.844), new Pose(101.353, 59.400))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                .build();
    }

    public PathChain intake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(101.353, 59.400), new Pose(123, 59.400))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(121.900, 59.400), new Pose(101.353, 113.844))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                .build();
    }
}
