package org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.v5;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Auto V5 paths, from the Pedro visualizer file "yeehw.pp (1).json" (5 paths).
 * Headings use the shortest turn between each path's start and end heading.
 */
public class PathsV5 {

    private static final PoseFactory P = PoseFactory.degrees();

    /** Start: far wall, facing the audience (path 1 starts at 270 deg). */
    public static final Pose start = P.of(29.013237173950344, 133.65237908759272, 270);

    // Path 1: start -> far wall FLOWER
    private final Pose p1Control = P.of(35.58471479286674, 120.107262345086, 0);
    private final Pose p1End = P.of(46.44228759178749, 127.18567656553711, 89);

    // Path 2: far wall FLOWER -> red wall FLOWER
    private final Pose p2Start = P.of(46.44228759178749, 127.18567656553711, 90);
    private final Pose p2Control1 = P.of(21.792584664260925, 104.98035748057424, 0);
    private final Pose p2Control2 = P.of(30.42738765590081, 43.80518360705265, 0);
    private final Pose p2End = P.of(15.41323181299063, 46.64537359200992, 180);

    // Path 3: red wall FLOWER -> shooting spot
    private final Pose p3Control = P.of(31.753651193615646, 49.04438180394235, 0);
    private final Pose p3End = P.of(43.55600787593538, 14.755595961245977, 180);

    // Path 4: shooting spot -> corner (intake the corner balls on the way)
    private final Pose p4Control1 = P.of(36.58479195973446, 37.14051959916381, 0);
    private final Pose p4Control2 = P.of(2.9968411001487762, 25.146116705573554, 0);
    private final Pose p4End = P.of(5.706834024559205, 10.892003897358878, 270);

    // Path 5: corner -> park
    private final Pose p5Control = P.of(42.08944374983573, 69.46583013418234, 0);
    private final Pose p5End = P.of(13.400270105717619, 96.15005452351028, 125);

    /** Start -> far wall FLOWER. */
    public Path path1() {
        return Paths.curve(start, p1Control, p1End).linear(start, p1End);
    }

    /** Far wall FLOWER -> red wall FLOWER. */
    public Path path2() {
        return Paths.curve(p2Start, p2Control1, p2Control2, p2End).linear(p2Start, p2End);
    }

    /** Red wall FLOWER -> shooting spot (heading stays 180). */
    public Path path3() {
        return Paths.curve(p2End, p3Control, p3End).constant(p3End);
    }

    /** Shooting spot -> corner. */
    public Path path4() {
        return Paths.curve(p3End, p4Control1, p4Control2, p4End).linear(p3End, p4End);
    }

    /** Corner -> park. */
    public Path path5() {
        return Paths.curve(p4End, p5Control, p5End).linear(p4End, p5End);
    }
}
