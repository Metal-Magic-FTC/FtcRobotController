package org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.v1;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

/**
 * Auto V1 paths, generated from the Pedro visualizer file my_path2.pp (6 paths, in sequence order).
 */
public class PathsV1 {

    /**
     * The .pp marks paths 2-6 with "reverse": true on their linear heading, which turns from start to end heading
     * the LONG way around (e.g. 180 -> 250 spins -290 degrees instead of +70). In Pedro 3 that is longLinear.
     * Set false to take the shortest turn instead.
     */
    public static final boolean FOLLOW_VISUALIZER_REVERSE = true;

    private static final PoseFactory P = PoseFactory.degrees();

    public static final Pose START_POSE = P.of(30.725, 6.851, 90);

    public static final Pose POSE_1 = P.of(8.131, 7.234, 180);
    public static final Pose POSE_2 = P.of(40.662, 113.526, 250);
    public static final Pose POSE_3 = P.of(9.963, 47.279, 180);
    public static final Pose POSE_4 = P.of(45.894, 11.389, 90);
    public static final Pose POSE_5 = P.of(58.659, 108.740, 90);
    public static final Pose POSE_6 = P.of(14.049, 93.826, 0);

    public Path path1() {
        return line(START_POSE, POSE_1).linear(Math.toRadians(180), Math.toRadians(180));
    }

    public Path path2() {
        return heading(line(POSE_1, POSE_2), 180, 250);
    }

    public Path path3() {
        return heading(curve(POSE_2, P.of(27.746, 43.724, 0), POSE_3), 250, 180);
    }

    public Path path4() {
        return heading(line(POSE_3, POSE_4), 180, 90);
    }

    public Path path5() {
        return heading(curve(POSE_4, P.of(59.279, 8.025, 0), POSE_5), 90, 90);
    }

    public Path path6() {
        return heading(line(POSE_5, POSE_6), 90, 0);
    }

    /** Linear heading that matches the visualizer's "reverse" flag (long way round) when enabled. */
    private static Path heading(Path path, double startDeg, double endDeg) {
        double start = Math.toRadians(startDeg);
        double end = Math.toRadians(endDeg);
        return FOLLOW_VISUALIZER_REVERSE ? path.heading(Interpolator.longLinear(start, end)) : path.linear(start, end);
    }
}
