package org.firstinspires.ftc.teamcode.decode.pedroPathing.autov2.blueback;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;

public class GeneratedPathsBlueBackV4 {

    private final Follower follower;

    // Central definition of the starting pose
//    public static final Pose START_POSE = new Pose(
//            58,  // X
//            135, // Y
//            Math.toRadians(270) // Heading
//    );

    public static final Pose START_POSE = new Pose(
            22,  // X
            128.83, // Y
            Math.toRadians(-45) // Heading
    );

    public static final Pose SCAN_POSE = new Pose(
            40,
            120,
            Math.toRadians(55)
    );

    public static final Pose SHOOT_POSE = new Pose(
            36,
            100,
            Math.toRadians(138)
    );

    public GeneratedPathsBlueBackV4(Follower follower) {
        this.follower = follower;
    }
    public PathChain scan(){
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(START_POSE, SCAN_POSE)
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(55))
                .build();
    }

    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(SCAN_POSE, SHOOT_POSE)
                )
                .setLinearHeadingInterpolation(Math.toRadians(55), Math.toRadians(138))
                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(SHOOT_POSE, new Pose(47, 73))
                )
                .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(180))
                .build();
    }

    public PathChain intakeball1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(47, 73), new Pose(41, 73))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public PathChain intakeball2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(41, 73), new Pose(36, 73))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public PathChain intakeball3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(36, 73), new Pose(25, 73))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }



    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(30, 73), SHOOT_POSE)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(SHOOT_POSE, new Pose(48.3, 59.400))
                )
                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                .build();
    }

    public PathChain intakeball4() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(91, 59.400), new Pose(106.5, 59.400))
                )
                .setLinearHeadingInterpolation(Math.toRadians(139.67), Math.toRadians(180))
                .build();
    }
    public PathChain intakeball5() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(106.5, 59.400), new Pose(112, 59.400))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public PathChain intakeball6() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(144-112, 59.400), new Pose(144-120, 59.400))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(123, 59.400), SHOOT_POSE)
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(41))
                .build();
    }
}