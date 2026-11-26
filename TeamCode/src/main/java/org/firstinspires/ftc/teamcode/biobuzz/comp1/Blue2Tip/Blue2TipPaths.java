package org.firstinspires.ftc.teamcode.biobuzz.comp1.Blue2Tip;

import com.pedropathing.follower.Follower;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * BLUE 2-tip paths: a mirror of Red2TipPaths across the field center line (x = 72).
 *   blue x = 144 - red x,   y unchanged,   blue heading = 180 - red heading.
 * If you retune a red pose, mirror it the same way here.
 */
public class Blue2TipPaths {

    private static final PoseFactory poseFactory = PoseFactory.degrees();

    // ===================== PATH SPEEDS (fraction of top speed, 1.0 = full) =====================
    public static final double FAST      = 1.0;  // normal drive legs
    public static final double SWEEP     = 0.45; // garden sweep
    public static final double CREEP     = 0.35; // last few inches into a flower
    public static final double SHORT_HOP = 0.7;  // launch spot <-> far-wall flower

    // ===================== POSES =====================
    // Starts on the AUDIENCE wall: cell A is the up cell with the 3 nectar.            red: (58, 9, 90)
    private final Pose start = poseFactory.of(86, 9, 90);

    // Garden sweep along the audience wall toward the blue-wall corner               red: (26, 10, 180) / (10, 10, 180)
    private final Pose gardenStart = poseFactory.of(118, 10, 0);
    private final Pose gardenEnd = poseFactory.of(134, 10, 0);

    // Cell B launch spot: all three cell B volleys. Tune this first.                  red: (58, 127, 270)
    private final Pose launchB = poseFactory.of(86, 127, 270);

    // Blue-wall FLOWER                                                                red: (28, 48, 180) / (15.4132, 46.6454, 180)
    private final Pose blueWallFlowerApproach = poseFactory.of(116, 48, 0);
    private final Pose blueWallFlower = poseFactory.of(128.5868, 46.6454, 0);

    // Far-wall FLOWER                                                                 red: (46.4423, 127.1857, 90)
    private final Pose farWallFlower = poseFactory.of(97.5577, 127.1857, 90);

    // Park in the loading zone                                                        red: (9.6023, 103.0424, 125)
    private final Pose park = poseFactory.of(134.3977, 103.0424, 55);

    // Curve control points: keep the robot in the lane along the blue wall, clear of the hive and the tip 1 spill
    private final Pose gardenToBControl1 = poseFactory.of(120, 30, 180);
    private final Pose gardenToBControl2 = poseFactory.of(122, 122, 180);
    private final Pose laneControlRear = poseFactory.of(118, 118, 180);
    private final Pose laneControlAudience = poseFactory.of(118, 70, 180);

    /** Tells the follower where the robot starts. Call once during init. */
    public void setStartPose(Follower follower) {
        follower.setPose(start);
    }

    /** Start -> garden. */
    public Path toGarden() {
        return Paths.line(start, gardenStart).linear(start, gardenStart);
    }

    /** Sweep the garden along the audience wall (intake on). */
    public Path gardenSweep() {
        return Paths.line(gardenStart, gardenEnd).linear(gardenStart, gardenEnd);
    }

    /** Garden -> lane -> cell B launch spot. */
    public Path gardenToLaunchB() {
        return Paths.curve(gardenEnd, gardenToBControl1, gardenToBControl2, launchB).linear(gardenEnd, launchB);
    }

    /** Launch spot -> blue-wall FLOWER approach. */
    public Path launchBToBlueWallFlower() {
        return Paths.curve(launchB, laneControlRear, laneControlAudience, blueWallFlowerApproach).linear(launchB, blueWallFlowerApproach);
    }

    /** Creep into the blue-wall FLOWER (intake on). */
    public Path blueWallFlowerCreep() {
        return Paths.line(blueWallFlowerApproach, blueWallFlower).linear(blueWallFlowerApproach, blueWallFlower);
    }

    /** Blue-wall FLOWER -> cell B launch spot. */
    public Path blueWallFlowerToLaunchB() {
        return Paths.curve(blueWallFlower, laneControlAudience, laneControlRear, launchB).linear(blueWallFlower, launchB);
    }

    /** Launch spot -> far-wall FLOWER (turns around in place, intake on). */
    public Path launchBToFarWallFlower() {
        return Paths.line(launchB, farWallFlower).linear(launchB, farWallFlower);
    }

    /** Far-wall FLOWER -> cell B launch spot. */
    public Path farWallFlowerToLaunchB() {
        return Paths.line(farWallFlower, launchB).linear(farWallFlower, launchB);
    }

    /** Launch spot -> park. */
    public Path parkPath() {
        return Paths.line(launchB, park).linear(launchB, park);
    }

    /** Emergency park: straight from wherever the robot is right now (the whole route stays right of the HIVE). */
    public Path parkFromHere(Follower follower) {
        Pose current = follower.pose();
        return Paths.line(current, park).linear(current, park);
    }

    /** Far-wall FLOWER -> park (used when the HIVE already tipped, so no return to the launch spot). */
    public Path farWallFlowerToPark() {
        return Paths.line(farWallFlower, park).linear(farWallFlower, park);
    }

    /** RESCUE (tip 1 failed): launch spot -> back to the start spot through the lane (a straight line would hit the HIVE). */
    public Path rescueToStart() {
        return Paths.curve(launchB, laneControlRear, laneControlAudience, start).linear(launchB, start);
    }

    /** RESCUE: start spot -> park through the lane. */
    public Path rescueToPark() {
        return Paths.curve(start, laneControlAudience, laneControlRear, park).linear(start, park);
    }
}
