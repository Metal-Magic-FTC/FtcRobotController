package org.firstinspires.ftc.teamcode.decode.pedroPathing.autov2.blueback;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class GeneratedPathsBlueBack {

    private final Follower follower;

    // Central definition of the starting pose
    public static final Pose START_POSE = new Pose(
            57,  // X
            135, // Y
            Math.toRadians(270) // Heading
    );

    public GeneratedPathsBlueBack(Follower follower) {
        this.follower = follower;
    }

    public PathChain scan() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(START_POSE, new Pose(48, 116))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(90))
                .build();
    }

    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(48, 116), new Pose(40, 110))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(133))
                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(40, 110), new Pose(48.3, 82))
                )
                .setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(180))
                .build();
    }



    public PathChain intakeball3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(48.3, 82), new Pose(20, 82))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }


    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(20, 82), new Pose(40, 110))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(133))
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(40, 110), new Pose(48.3, 110))
                )
                .setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(180))
                .build();
    }


    public PathChain intakeball6() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(144 - 112, 59.400), new Pose(144 - 120, 59.400))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180 - 0), Math.toRadians(180 - 0))
                .build();
    }

    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(144 - 123, 59.400), new Pose(40, 110))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180 - 0), Math.toRadians(180 - 45))
                .build();
    }

    public PathChain toIntake3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(59, 81), new Pose(42, 35.5))
                )
                .setLinearHeadingInterpolation(Math.toRadians(136.67), Math.toRadians(180 - 0))
                .build();
    }
    public PathChain intakeball7() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(42, 35.5), new Pose(36, 35.5))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180 - 0), Math.toRadians(180 - 0))
                .build();
    }


public PathChain intakeball8() {
    return new PathBuilder(follower)
            .addPath(
                    new BezierLine(new Pose(36, 35.5), new Pose(30, 35.5))
            )
            .setLinearHeadingInterpolation(Math.toRadians(180 - 0), Math.toRadians(180 - 0))
            .build();
}
    public PathChain intakeball9() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(30, 35.5), new Pose(22, 35.5))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180 - 0), Math.toRadians(180 - 0))
                .build();
    }
    public PathChain shoot4() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(22, 35.5), new Pose(59, 81))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180 - 0), Math.toRadians(180 - 45))
                .build();
    }

}