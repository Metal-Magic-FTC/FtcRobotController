package org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.downSupport;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Down Support paths, exported from the Pedro visualizer (Down_Support.pp.json + the park path that was only added
 * to the Java export). Class renamed from Paths, start pose made public, paths named by what they do.
 */
public class PathsDownSupport {

    private static final PoseFactory poseFactory = PoseFactory.degrees();

    public static final Pose start = poseFactory.of(30.9914, 132.236, 270);
    private final Pose farShoot = poseFactory.of(55.8387, 30.8369, 89);
    private final Pose farShootControl1 = poseFactory.of(0, 22.4534, 0);
    private final Pose farShootControl2 = poseFactory.of(31.7644, 36.2636, 0);
    private final Pose corner = poseFactory.of(9.9829, 12.4771, 270);
    private final Pose cornerControl1 = poseFactory.of(21.0163, 39.5269, 0);
    private final Pose homeShoot = poseFactory.of(31.0204, 132.2088, 270);
    private final Pose homeShootControl1 = poseFactory.of(28.6592, 71.0389, 0);
    private final Pose flower = poseFactory.of(47.2828, 128.678, 89);
    private final Pose flowerControl1 = poseFactory.of(36.3242, 108.1996, 0);
    private final Pose park = poseFactory.of(9.6094, 106.2377, 0);

    /** Path 1: across the field to shoot the preloads. */
    public Path toFarShoot() {
        return curve(start, farShootControl1, farShootControl2, farShoot).linear(start, farShoot);
    }

    /** Path 2: into the corner to pick up more pollen. */
    public Path toCorner() {
        return curve(farShoot, cornerControl1, corner).linear(farShoot, corner);
    }

    /** Path 3: back to the starting side to shoot. */
    public Path toHomeShoot() {
        return curve(corner, homeShootControl1, homeShoot).linear(corner, homeShoot);
    }

    /** Path 4: to the scoring-side FLOWER. */
    public Path toFlower() {
        return curve(homeShoot, flowerControl1, flower).linear(homeShoot, flower);
    }

    /** Path 5: park (Java export only, not in the .pp file). */
    public Path toPark() {
        return line(flower, park).linear(flower, park);
    }
}
