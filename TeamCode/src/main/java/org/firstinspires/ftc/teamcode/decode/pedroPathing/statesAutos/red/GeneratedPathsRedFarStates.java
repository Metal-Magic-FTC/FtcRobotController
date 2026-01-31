package org.firstinspires.ftc.teamcode.decode.pedroPathing.statesAutos.red;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class GeneratedPathsRedFarStates {

    private final Follower follower;

    // Central definition of the starting pose
//    public static final Pose START_POSE = new Pose(
//            92,  // X
//            135, // Y
//            Math.toRadians(270) // Heading
//    );

    public static final Pose START_POSE = new Pose(
            87,  // X
            8, // Y
            Math.toRadians(90) // Heading
    );

    public static final Pose SCAN_POSE = new Pose(
            87,
            81,
            Math.toRadians(90)
    );

    public static final Pose SHOOT_POSE = new Pose(
            83,
            81,
            Math.toRadians(30)
    );

    public GeneratedPathsRedFarStates(Follower follower) {
        this.follower = follower;
    }

    public PathChain scan(){
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(START_POSE, SCAN_POSE)
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();
    }

    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(SCAN_POSE, SHOOT_POSE)
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(30))
                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(SHOOT_POSE, new Pose(100, 30))
                )
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                .build();
    }

    public PathChain intakeball1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(100, 30), new Pose(122, 30))
                )
                .setTangentHeadingInterpolation()
                .build();
    }

//    public PathChain intakeball2() {
//        return new PathBuilder(follower)
//                .addPath(
//                        new BezierLine(new Pose(94, 89), new Pose(99, 89))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .build();
//    }
//
//    public PathChain intakeball3() {
//        return new PathBuilder(follower)
//                .addPath(
//                        new BezierLine(new Pose(99, 83), new Pose(119, 83))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .build();
//    }



    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(122, 29), SHOOT_POSE) // x used to be 116
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(SHOOT_POSE, new Pose(100,54))
                )
                .setTangentHeadingInterpolation()
                .build();
    }

    public PathChain intakeball4() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(100, 54), new Pose(122, 54))
                )
                .setTangentHeadingInterpolation()
                .build();
    }
//    public PathChain intakeball5() {
//        return new PathBuilder(follower)
//                .addPath(
//                        new BezierLine(new Pose(106.5, 59.400), new Pose(112, 59.400))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .build();
//    }
//    public PathChain intakeball6() {
//        return new PathBuilder(follower)
//                .addPath(
//                        new BezierLine(new Pose(91, 59.400), new Pose(127, 59.400))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .build();
//    }

    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(122, 54), SHOOT_POSE)
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                .build();
    }
    public PathChain leave() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(SHOOT_POSE, new Pose(83,55))
                )
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(30))
                .build();
    }
}
