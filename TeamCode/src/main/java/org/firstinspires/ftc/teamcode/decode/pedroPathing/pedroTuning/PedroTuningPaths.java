package org.firstinspires.ftc.teamcode.decode.pedroPathing.pedroTuning;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class PedroTuningPaths {

    private final Follower follower;

    // ---------------- POSES ----------------

    // Where the robot is placed at the start of the test
    public static final Pose START_POSE = new Pose(
            103.880,            // X
            135.920,            // Y
            Math.toRadians(90)  // Heading
    );

    // End of path1 / start of path2
    public static final Pose MID_POSE = new Pose(
            86.535,
            108.632,
            Math.toRadians(180)
    );

    // End of path2
    public static final Pose END_POSE = new Pose(
            99.207,
            59.568,
            Math.toRadians(0)
    );

    public PedroTuningPaths(Follower follower) {
        this.follower = follower;
    }

    // ---------------- PATHS ----------------

    /** START_POSE -> MID_POSE, turning from 90 deg to 180 deg. */
    public PathChain path1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(START_POSE, MID_POSE)
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();
    }

    /** MID_POSE -> END_POSE, turning from 180 deg to 0 deg. */
    public PathChain path2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(MID_POSE, END_POSE)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
    }

    /** Both segments in a single chain, if you want to run them without stopping in between. */
    public PathChain full() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(START_POSE, MID_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .addPath(new BezierLine(MID_POSE, END_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
    }
}
