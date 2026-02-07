package org.firstinspires.ftc.teamcode.decode.pedroPathing.StatesAutosV2.StatesBlueAutos.BlueFar;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class GeneratedPathsBlueFarStates {

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
//            58.5,
//            15,
//            Math.toRadians(80)
//    );

    public static final Pose SHOOT_POSE = new Pose(
            59,
            12,
            Math.toRadians(90)
    );

    public GeneratedPathsBlueFarStates(Follower follower) {
        this.follower = follower;
    }

//    public PathChain scan(){
//        return new PathBuilder(follower)
//                .addPath(
//                        new BezierLine(START_POSE, SCAN_POSE)
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(80))
//                .build();
//    }

    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(START_POSE, SHOOT_POSE)
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(SHOOT_POSE, new Pose(41, 35))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();
    }

    public PathChain intakeball1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(41, 35), new Pose(23, 35))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
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
                        new BezierLine(new Pose(23, 35), SHOOT_POSE) // x used to be 116
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(SHOOT_POSE, new Pose(42,60))
                )
                .setTangentHeadingInterpolation()
                .build();
    }

    public PathChain intakeball4() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(42, 60), new Pose(25, 60))
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
                        new BezierLine(new Pose(25, 60), SHOOT_POSE)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();
    }

    public PathChain toIntake3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(SHOOT_POSE, new Pose(42,60))
                )
                .setTangentHeadingInterpolation()
                .build();
    }





    public PathChain leave() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(SHOOT_POSE, new Pose(59,33))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();
    }
}
