package org.firstinspires.ftc.teamcode.decode.pedroPathing.autov2.redback;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class GeneratedPathsRedBackV5 {

    private final Follower follower;

    // Central definition of the starting pose
//    public static final Pose START_POSE = new Pose(
//            92,  // X
//            135, // Y
//            Math.toRadians(270) // Heading
//    );

    public static final Pose START_POSE = new Pose(
            116,  // X
            128, // Y
            Math.toRadians(225) // Heading
    );

    public static final Pose SCAN_POSE = new Pose(
            102,
            120,
            Math.toRadians(107)
    );

    public static final Pose SHOOT_POSE = new Pose(
            95,
            103,
            Math.toRadians(42)
    );

    public GeneratedPathsRedBackV5(Follower follower) {
        this.follower = follower;
    }
    public PathChain scan(){
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(START_POSE, SCAN_POSE)
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(96))
                .build();
    }

    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(SCAN_POSE, SHOOT_POSE)
                )
                .setLinearHeadingInterpolation(Math.toRadians(96), Math.toRadians(52))
                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(SHOOT_POSE, new Pose(87, 89))
                )
                .setLinearHeadingInterpolation(Math.toRadians(52), Math.toRadians(0))
                .build();
    }

    public PathChain intakeball1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(87, 89), new Pose(94, 89))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public PathChain intakeball2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(94, 89), new Pose(99, 89))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public PathChain intakeball3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(99, 89), new Pose(107, 89))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }



    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(107, 89), SHOOT_POSE) // x used to be 116
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(41))
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(93, 94), new Pose(101, 59))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();
    }

    public PathChain intakeball4() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(101, 59), new Pose(110,59))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public PathChain intakeball5() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(110, 59), new Pose(114, 59))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public PathChain intakeball6() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(114, 59), new Pose(127, 59))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(127, 59), SHOOT_POSE)
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(41))
                .build();
    }
}