package org.firstinspires.ftc.teamcode.decode.pedroPathing.StatesAutosV2.StatesRedAutos.RedFar;

import com.pedropathing.follower.Follower;
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
            0, // Y
            Math.toRadians(90) // Heading
    );

    public static final Pose SHOOT_POSE = new Pose(
            86,
            7,
            Math.toRadians(63)
    );

    public GeneratedPathsRedFarStates(Follower follower) {
        this.follower = follower;
    }

    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(START_POSE, SHOOT_POSE)
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(61))
                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(SHOOT_POSE, new Pose(101, 26))
                )
                .setLinearHeadingInterpolation(Math.toRadians(59), 0)
                .build();
    }

    public PathChain intakeball1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(101, 26), new Pose(125, 23))
                )
                .setLinearHeadingInterpolation(0, 0)
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
                        new BezierLine(new Pose(138, 26), SHOOT_POSE) // x used to be 116
                )
                .setLinearHeadingInterpolation(0,Math.toRadians(67))
                .build();
    }

    public PathChain leaveNew() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(SHOOT_POSE, new Pose(110, 3))
                )
                .setLinearHeadingInterpolation(Math.toRadians(67), Math.toRadians(90))
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(SHOOT_POSE,
                                 new Pose(101,53))
                )
                .setLinearHeadingInterpolation(Math.toRadians(58), 0)
                .build();
    }

    public PathChain intakeball4() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(101, 53), new Pose(125, 53))
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
                        new BezierLine(new Pose(125, 53), SHOOT_POSE)
                )
                .setLinearHeadingInterpolation(0,Math.toRadians(58))
                .build();
    }
    public PathChain leave() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(SHOOT_POSE, new Pose(89,28))
                )
                .setTangentHeadingInterpolation()
                .build();
    }
}
