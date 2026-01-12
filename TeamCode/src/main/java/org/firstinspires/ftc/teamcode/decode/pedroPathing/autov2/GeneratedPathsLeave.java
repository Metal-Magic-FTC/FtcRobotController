package org.firstinspires.ftc.teamcode.decode.pedroPathing.autov2;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;

public class GeneratedPathsLeave {

    private final Follower follower;

    public static final Pose START_POSE = new Pose(
            0,  // X
            0, // Y
            Math.toRadians(90) // Heading
    );

    public static final Pose LEAVE_POSE = new Pose(
            0,
            24,
            Math.toRadians(90)
    );

    public GeneratedPathsLeave(Follower follower) {
        this.follower = follower;
    }

    public PathChain exit(){
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(START_POSE, LEAVE_POSE)
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();
    }

}