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
            116.6988847583643,  // X
            128.83271375464685, // Y
            Math.toRadians(225) // Heading
    );

    public GeneratedPathsRedBack(Follower follower) {
        this.follower = follower;
    }
    public PathChain scan(){
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(START_POSE, new Pose(100, 100))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(105))
                .build();
    }

    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(100,100), new Pose(93, 94))
                )
                .setLinearHeadingInterpolation(Math.toRadians(105), Math.toRadians(46.3067))
                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(93, 94), new Pose(101.353, 81))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();
    }

    public PathChain intakeball1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(101.353, 81), new Pose(104, 81))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public PathChain intakeball2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(104, 81), new Pose(109, 81))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public PathChain intakeball3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(109, 81), new Pose(120, 81))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }



    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(121.900, 81), new Pose(93, 94))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(41.67))
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(93, 94), new Pose(101.353, 59.400))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();
    }

    public PathChain intakeball4() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(101.353, 59.400), new Pose(106.5, 59.400))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
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
                        new BezierLine(new Pose(112, 59.400), new Pose(120, 59.400))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(123, 59.400), new Pose(86, 87))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();
    }
}
