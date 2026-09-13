package org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.v1;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

/**
 * Auto V1 paths, generated from the Pedro visualizer file my_path2.pp (6 paths, in sequence order).
 */
public class PathsV1 {

    /**
     * The .pp marks paths 2-6 with "reverse": true on their linear heading. In Pedro that is
     * HeadingInterpolator.reversedLinear, which turns from start to end heading the LONG way around
     * (e.g. 180 -> 250 spins -290 degrees instead of +70). Set false to take the shortest turn instead.
     */
    public static final boolean FOLLOW_VISUALIZER_REVERSE = true;

    public static final Pose START_POSE = new Pose(30.725, 6.851, Math.toRadians(90));

    public static final Pose POSE_1 = new Pose(8.131, 7.234, Math.toRadians(180));
    public static final Pose POSE_2 = new Pose(40.662, 113.526, Math.toRadians(250));
    public static final Pose POSE_3 = new Pose(9.963, 47.279, Math.toRadians(180));
    public static final Pose POSE_4 = new Pose(45.894, 11.389, Math.toRadians(90));
    public static final Pose POSE_5 = new Pose(58.659, 108.740, Math.toRadians(90));
    public static final Pose POSE_6 = new Pose(14.049, 93.826, Math.toRadians(0));

    private final Follower follower;

    public PathsV1(Follower follower) {
        this.follower = follower;
    }

    public PathChain path1() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(START_POSE, POSE_1))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public PathChain path2() {
        return heading(new PathBuilder(follower)
                .addPath(new BezierLine(POSE_1, POSE_2)), 180, 250, true)
                .build();
    }

    public PathChain path3() {
        return heading(new PathBuilder(follower)
                .addPath(new BezierCurve(
                        POSE_2,
                        new Pose(27.746, 43.724),
                        POSE_3)), 250, 180, true)
                .build();
    }

    public PathChain path4() {
        return heading(new PathBuilder(follower)
                .addPath(new BezierLine(POSE_3, POSE_4)), 180, 90, true)
                .build();
    }

    public PathChain path5() {
        return heading(new PathBuilder(follower)
                .addPath(new BezierCurve(
                        POSE_4,
                        new Pose(59.279, 8.025),
                        POSE_5)), 90, 90, true)
                .build();
    }

    public PathChain path6() {
        return heading(new PathBuilder(follower)
                .addPath(new BezierLine(POSE_5, POSE_6)), 90, 0, true)
                .build();
    }

    private static PathBuilder heading(PathBuilder builder, double startDeg, double endDeg, boolean reversedInVisualizer) {
        double start = Math.toRadians(startDeg);
        double end = Math.toRadians(endDeg);
        if (reversedInVisualizer && FOLLOW_VISUALIZER_REVERSE) {
            return builder.setHeadingInterpolation(HeadingInterpolator.reversedLinear(start, end));
        }
        return builder.setLinearHeadingInterpolation(start, end);
    }
}
