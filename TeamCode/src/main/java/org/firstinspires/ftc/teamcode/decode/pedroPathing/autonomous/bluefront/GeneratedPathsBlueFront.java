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
            64.1495327103,  // X
            9.196261682242984, // Y
            Math.toRadians(90) // Heading
    );

    public GeneratedPathsBlueFront(Follower follower) {
        this.follower = follower;
    }

    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(START_POSE, new Pose(58, 13))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120))
                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(58, 13), new Pose(42, 35.664))
                )
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                .build();
    }

    public PathChain intake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(42, 35.664), new Pose(22, 35.664))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(22, 35.664), new Pose(58, 13))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(75.589, 21.084), new Pose(42, 59.400))
                )
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(180))
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
                        new BezierLine(new Pose(130.991, 60.112), new Pose(58, 13))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                .build();
    }
}
