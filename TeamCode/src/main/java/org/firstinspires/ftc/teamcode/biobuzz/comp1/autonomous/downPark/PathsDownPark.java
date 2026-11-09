package org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.downPark;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Paths from the Pedro visualizer file Down_Park.pp.json.
 *
 * START HEADING: the file's start point says 90 deg, but Path 1 begins at 270 deg (facing the audience / the red
 * HIVE). We start at 270 so the Limelight faces the HIVE while waiting for its tags, and the robot does not spin
 * 180 deg the moment Path 1 starts.
 */
public class PathsDownPark {

    private static final PoseFactory poseFactory = PoseFactory.degrees();

    public static final Pose start = poseFactory.of(31.1653, 132.2628, 270);
    private final Pose flowerControl = poseFactory.of(33.5944, 107.2594, 0);
    private final Pose flower = poseFactory.of(46.3806, 129.2469, 89);
    private final Pose park = poseFactory.of(10.8101, 105.1973, 0);

    /** Path 1: curve to the scoring-side FLOWER, turning from 270 to 89 deg. */
    public Path toFlower() {
        return curve(start, flowerControl, flower).linear(start, flower);
    }

    /** Path 2: straight to the park spot, turning from 89 to 0 deg. */
    public Path toPark() {
        return line(flower, park).linear(flower, park);
    }
}
