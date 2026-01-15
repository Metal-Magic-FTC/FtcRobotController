package org.firstinspires.ftc.teamcode.decode.pedroPathing.autov2.blueback;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;

public class GeneratedPathsBlueBackV5 {

    private final Follower follower;
    

    public GeneratedPathsBlueBackV5(Follower follower) {
        this.follower = follower;
    }
    public PathChain scan(){
        return new PathBuilder(follower).addPath(
        new BezierLine(
                new Pose(24.352, 129.319),

                new Pose(54.330, 128.440)
        )
        ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(45))

                .build();
    }

    public PathChain shoot() {
        return new PathBuilder(follower).addPath(
                        new BezierLine(
                                new Pose(54.330, 128.440),

                                new Pose(42.813, 101.934)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(135))

                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower).addPath(
                        new BezierLine(
                                new Pose(42.813, 101.934),

                                new Pose(41.121, 84.132)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                .build();
    }

    public PathChain intakeball1() {
        return new PathBuilder(follower).addPath(
                        new BezierLine(
                                new Pose(41.121, 84.132),

                                new Pose(18.978, 83.901)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }


    public PathChain shoot2() {
        return new PathBuilder(follower).addPath(
                        new BezierLine(
                                new Pose(18.978, 59.802),

                                new Pose(42.813, 101.934)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower).addPath(
                        new BezierLine(
                                new Pose(42.813, 101.934),

                                new Pose(41.121, 59.791)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                .build();
    }

    public PathChain intakeball2() {
        return new PathBuilder(follower).addPath(
                        new BezierLine(
                                new Pose(41.121, 59.791),

                                new Pose(18.978, 59.802)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }


    public PathChain shoot3() {
        return new PathBuilder(follower).addPath(
                        new BezierLine(
                                new Pose(54.330, 128.440),

                                new Pose(42.813, 101.934)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(135))

                .build();
    }
}