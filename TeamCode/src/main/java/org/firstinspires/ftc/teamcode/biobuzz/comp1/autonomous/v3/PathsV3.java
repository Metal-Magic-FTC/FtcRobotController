package org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.v3;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Auto V3 paths, exported from the Pedro visualizer (class renamed from Paths, start pose made public).
 */
public class PathsV3 {

    private static final PoseFactory poseFactory = PoseFactory.degrees();

    public static final Pose start = poseFactory.of(30.75, 132.75, 270);
    private final Pose path1 = poseFactory.of(46.4423, 127.1857, 89);
    private final Pose path1Control1 = poseFactory.of(35.5847, 120.1073, 0);
    private final Pose point2Start = poseFactory.of(46.4423, 127.1857, 90);
    private final Pose point2 = poseFactory.of(15.4132, 46.6454, 180);
    private final Pose point2Control1 = poseFactory.of(21.7926, 104.9804, 0);
    private final Pose point2Control2 = poseFactory.of(30.4274, 43.8052, 0);
    private final Pose point3Start = poseFactory.of(15.4132, 46.6454, 180);
    private final Pose point3 = poseFactory.of(32.0591, 9.7088, -175);
    private final Pose point4 = poseFactory.of(8.2307, 7.7588, -175.3217);
    private final Pose point5Start = poseFactory.of(8.2307, 7.7588, -175);
    private final Pose point5 = poseFactory.of(9.6023, 103.0424, 125);
    private final Pose point5Control1 = poseFactory.of(42.0894, 69.4658, 0);

    /** Start -> far wall FLOWER. */
    public Path path1() {
        return curve(start, path1Control1, path1).linear(start, path1);
    }

    /** Far wall FLOWER -> red wall FLOWER. */
    public Path path2() {
        return curve(point2Start, point2Control1, point2Control2, point2).linear(point2Start, point2);
    }

    /** Red wall FLOWER -> shooting position. */
    public Path path3() {
        return line(point3Start, point3).linear(point3Start, point3);
    }

    /** Shooting position -> corner (red GARDEN). */
    public Path path4() {
        return line(point3, point4).tangent();
    }

    /** Corner -> park. */
    public Path path5() {
        return curve(point5Start, point5Control1, point5).linear(point5Start, point5);
    }
}
