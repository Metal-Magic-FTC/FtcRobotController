package org.firstinspires.ftc.teamcode.decode.pedroPathing.NihalsGayAutos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class NihalsGeneratedPathsBlueV1 {

    private final Follower follower;

    // Central definition of the starting pose
//    public static final Pose START_POSE = new Pose(
//            92,  // X
//            135, // Y
//            Math.toRadians(270) // Heading
//    );

    public static final Pose START_POSE = new Pose(
            56,  // X
            8, // Y
            Math.toRadians(90) // Heading
    );

//    public static final Pose SCAN_POSE = new Pose(
//            102,
//            120,
//            Math.toRadians(107)
//    );

    public static final Pose SHOOT_POSE = new Pose(
            60,
            75,
            Math.toRadians(135)
    );

    public NihalsGeneratedPathsBlueV1(Follower follower) {
        this.follower = follower;
    }
    public PathChain shoot(){
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(START_POSE, SHOOT_POSE)
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();
    }
//
//    public PathChain shoot() {
//        return new PathBuilder(follower)
//                .addPath(
//                        new BezierLine(, SHOOT_POSE)
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(96), Math.toRadians(52))
//                .build();
//    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(SHOOT_POSE, new Pose(50, 35))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();
    }

    public PathChain intakeball1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(50, 35), new Pose(44, 35))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public PathChain intakeball2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(44, 35), new Pose(38, 35))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public PathChain intakeball3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(38, 35), new Pose(32, 35))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }


    public PathChain midPointShoot() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(30, 35), new Pose(59, 40))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
    }
    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(59, 54), SHOOT_POSE) // x used to be 116
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine( SHOOT_POSE, new Pose(46, 58))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();
    }

    public PathChain intakeball4() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(46, 58), new Pose(40,58))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }
    public PathChain intakeball5() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(40, 58), new Pose(34, 58))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }
    public PathChain intakeball6() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(34, 58), new Pose(30, 58))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(30, 58), SHOOT_POSE)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
    }

    public PathChain returnToStart() {
        return new PathBuilder(follower)
                .addPath(
                     new BezierLine(SHOOT_POSE, new Pose(56, 12))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();
    }

    public PathChain slide() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(START_POSE, new Pose(37, 8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();
    }
}