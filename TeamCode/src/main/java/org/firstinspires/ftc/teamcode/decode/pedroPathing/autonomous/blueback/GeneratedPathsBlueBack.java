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
            25.562130177514792,  // X
            128.83271375464685, // Y
            Math.toRadians(315) // Heading
    );

    public GeneratedPathsBlueBack(Follower follower) {
        this.follower = follower;
    }

    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(START_POSE, new Pose(42, 113.844))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(42, 113.844), new Pose(42, 83.300))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();
    }

    public PathChain intake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(42, 83.300), new Pose(22, 83.300))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(22, 83.300), new Pose(42, 113.844))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(42, 113.844), new Pose(42, 59.400))
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                .build();
    }

    public PathChain intake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(42, 59.400), new Pose(22, 59.400))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(22, 59.400), new Pose(42, 113.844))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
    }
}
