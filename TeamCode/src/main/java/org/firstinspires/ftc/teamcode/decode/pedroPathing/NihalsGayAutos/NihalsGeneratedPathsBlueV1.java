package org.firstinspires.ftc.teamcode.decode.pedroPathing.NihalsGayAutos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class NihalsGeneratedPathsBlueV1 {

    private final Follower follower;

    public static final Pose START_POSE = new Pose(
            56,  // X
            8, // Y
            Math.toRadians(90) // Heading
    );


    public NihalsGeneratedPathsBlueV1(Follower follower) {
        this.follower = follower;
    }
    public PathChain shoot(){
        return new PathBuilder(follower).addPath(
        new BezierLine(
                new Pose(56, 8),

                new Pose(60, 75)
        )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(145))

                .build();
    }

//    public PathChain shoot() {
//        return new PathBuilder(follower).addPath(
//                        new BezierLine(
//                                new Pose(54.330, 128.440),
//
//                                new Pose(42.813, 101.934)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(135))
//
//                .build();
//    }

    public PathChain toIntake1() {
        return new PathBuilder(follower).addPath(
                        new BezierLine(
                                new Pose(60, 75),

                                new Pose(46, 35)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))

                .build();
    }

    public PathChain intakeball1() {
        return new PathBuilder(follower).addPath(
                        new BezierLine(
                                new Pose(46, 35),

                                new Pose(19, 35)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }


    public PathChain shoot2() {
        return new PathBuilder(follower).addPath(
                        new BezierLine(
                                new Pose(19, 35),

                                new Pose(60, 75)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))

                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower).addPath(
                        new BezierLine(
                                new Pose(60, 75),

                                new Pose(46, 60)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))

                .build();
    }

    public PathChain intakeball2() {
        return new PathBuilder(follower).addPath(
                        new BezierLine(
                                new Pose(46, 60),

                                new Pose(19, 60)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }


    public PathChain shoot3() {
        return new PathBuilder(follower).addPath(
                        new BezierLine(
                                new Pose(19, 60),

                                new Pose(60, 75)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))

                .build();
    }
}