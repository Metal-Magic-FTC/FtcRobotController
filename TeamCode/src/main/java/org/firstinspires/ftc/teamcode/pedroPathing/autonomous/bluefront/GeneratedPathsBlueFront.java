package org.firstinspires.ftc.teamcode.pedroPathing.autonomous.bluefront;

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
                        new BezierLine(START_POSE, new Pose(84.22304832713755, 13.204460966542756))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60))
                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(75.589, 21.084), new Pose(101.353, 35.664))
                )
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0))
                .build();
    }

    public PathChain intake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(101.353, 35.664), new Pose(123, 35.664))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(131.664, 35.664), new Pose(84.22304832713755, 13.204460966542756))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60))
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(75.589, 21.084), new Pose(101.353, 60.112))
                )
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0))
                .build();
    }

    public PathChain intake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(101.353, 59.400), new Pose(123, 60.112))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(130.991, 60.112), new Pose(84.22304832713755, 13.204460966542756))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60))
                .build();
    }
}
