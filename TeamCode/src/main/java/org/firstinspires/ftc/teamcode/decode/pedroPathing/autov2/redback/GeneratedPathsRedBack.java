package org.firstinspires.ftc.teamcode.decode.pedroPathing.autov2.redback;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class GeneratedPathsRedBack {

    private final Follower follower;

    // Central definition of the starting pose
    public static final Pose START_POSE = new Pose(
            87,  // X
            135, // Y
            Math.toRadians(270) // Heading
    );

    public static final Pose SCAN_POSE = new Pose(
            100,
            100,
            Math.toRadians(105)
    );

    public static final Pose SHOOT_POSE = new Pose(
            90,
            97,
            Math.toRadians(48)
    );


    public GeneratedPathsRedBack(Follower follower) {
        this.follower = follower;
    }
    public PathChain scan(){
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(START_POSE, SCAN_POSE)
                )
                .setLinearHeadingInterpolation(START_POSE.getHeading(), Math.toRadians(105))
                .build();
    }

    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(SCAN_POSE, SHOOT_POSE)
                )
                .setLinearHeadingInterpolation(Math.toRadians(105), Math.toRadians(48))
                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(SHOOT_POSE, new Pose(95, 84))
                )
                .setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(0))
                .build();
    }

    public PathChain intakeball1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(95, 84), new Pose(100, 84))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public PathChain intakeball2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(100, 84), new Pose(105, 84))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public PathChain intakeball3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(105, 84), new Pose(110, 84))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }



    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(114, 84), SHOOT_POSE)
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(48))
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(SHOOT_POSE, new Pose(95, 59.400))
                )
                .setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(0))
                .build();
    }

    public PathChain intakeball4() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(95, 59.400), new Pose(100, 59.400))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public PathChain intakeball5() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(100, 59.400), new Pose(105, 59.400))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public PathChain intakeball6() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(105, 59.400), new Pose(110, 59.400))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(114, 59.400), SHOOT_POSE)
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(48))
                .build();
    }
}
