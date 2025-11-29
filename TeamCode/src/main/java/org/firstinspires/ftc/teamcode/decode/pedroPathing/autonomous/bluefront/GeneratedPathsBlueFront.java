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
            79.85046728971962,  // X
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

    public PathChain intakeball1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(101.353, 35.664), new Pose(106.5, 35.664))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public PathChain intakeball2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(106.5, 35.664), new Pose(112, 35.664))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public PathChain intakeball3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(112, 35.664), new Pose(120, 35.664))
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

    public PathChain intakeball4() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(101.353, 59.400), new Pose(106.5, 60.112))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public PathChain intakeball5() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(106.5, 59.400), new Pose(112, 60.112))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public PathChain intakeball6() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(112, 59.400), new Pose(120, 60.112))
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
