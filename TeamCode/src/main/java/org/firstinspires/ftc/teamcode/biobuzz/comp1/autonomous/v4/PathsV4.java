package org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.v4;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Auto V4 paths (new robot), exported from the Pedro visualizer (class renamed from Paths, start pose made public).
 */
public class PathsV4 {

    private static final PoseFactory poseFactory = PoseFactory.degrees();

    public static final Pose start = poseFactory.of(29.0132, 133.6524, 270);
    private final Pose path1 = poseFactory.of(46.4423, 127.1857, 89);
    private final Pose path1Control1 = poseFactory.of(35.5847, 120.1073, 0);
    private final Pose point2Start = poseFactory.of(46.4423, 127.1857, 90);
    private final Pose point2 = poseFactory.of(15.4132, 46.6454, 180);
    private final Pose point2Control1 = poseFactory.of(21.7926, 104.9804, 0);
    private final Pose point2Control2 = poseFactory.of(30.4274, 43.8052, 0);
    private final Pose point3Start = poseFactory.of(15.4132, 46.6454, 180);
    private final Pose point3 = poseFactory.of(5.7068, 10.892, 270);
    private final Pose point4Start = poseFactory.of(5.7068, 10.892, 270);
    private final Pose point4 = poseFactory.of(9.6023, 103.0424, 125);
    private final Pose point4Control1 = poseFactory.of(42.0894, 69.4658, 0);

    /** Start -> far wall FLOWER. */
    public Path path1() {
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }

    /** Far wall FLOWER -> red wall FLOWER. */
    public Path path2() {
        return Paths.curve(point2Start, point2Control1, point2Control2, point2).linear(point2Start, point2);
    }

    /** Red wall FLOWER -> corner. */
    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    /** Corner -> park. */
    public Path path4() {
        return Paths.curve(point4Start, point4Control1, point4).linear(point4Start, point4);
    }
}
