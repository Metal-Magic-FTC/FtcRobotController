package org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.aud2tip;

import com.pedropathing.follower.Follower;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;


public class RedAud2TipPaths {

    private static final PoseFactory poseFactory = PoseFactory.degrees();

    // ===================== PATH SPEEDS (fraction of top speed, 1.0 = full) =====================
    public static final double FAST      = 1.0;  // normal drive legs
    public static final double SWEEP     = 0.45; // garden sweep
    public static final double CREEP     = 0.35; // last few inches into a flower
    public static final double SHORT_HOP = 0.7;  // launch spot <-> far-wall flower

    // ===================== POSES =====================
    // Starts on the AUDIENCE wall (not the rear wall like V4): cell A is the up cell with the 3 nectar.
    private final Pose start = poseFactory.of(58, 9, 90);

    // Garden sweep along the audience wall toward the red-wall corner
    private final Pose gardenStart = poseFactory.of(26, 10, 180);
    private final Pose gardenEnd = poseFactory.of(10, 10, 180);

    // Cell B launch spot: all three cell B volleys (31 in from cell B). Tune this first.
    private final Pose launchB = poseFactory.of(58, 127, 270);

    // Red-wall FLOWER (pose from PathsV4)
    private final Pose redWallFlowerApproach = poseFactory.of(28, 48, 180);
    private final Pose redWallFlower = poseFactory.of(15.4132, 46.6454, 180);

    // Far-wall FLOWER (pose from PathsV4)
    private final Pose farWallFlower = poseFactory.of(46.4423, 127.1857, 90);

    // Park in the loading zone (pose from PathsV4)
    private final Pose park = poseFactory.of(9.6023, 103.0424, 125);

    // Curve control points: keep the robot in the lane along the red wall, clear of the hive and the tip 1 spill
    private final Pose gardenToBControl1 = poseFactory.of(24, 30, 0);
    private final Pose gardenToBControl2 = poseFactory.of(22, 122, 0);
    private final Pose laneControlRear = poseFactory.of(26, 118, 0);
    private final Pose laneControlAudience = poseFactory.of(26, 70, 0);

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

    /** Launch spot -> red-wall FLOWER approach. */
    public Path launchBToRedWallFlower() {
        return Paths.curve(launchB, laneControlRear, laneControlAudience, redWallFlowerApproach).linear(launchB, redWallFlowerApproach);
    }

    /** Creep into the red-wall FLOWER (intake on). */
    public Path redWallFlowerCreep() {
        return Paths.line(redWallFlowerApproach, redWallFlower).linear(redWallFlowerApproach, redWallFlower);
    }

    /** Red-wall FLOWER -> cell B launch spot. */
    public Path redWallFlowerToLaunchB() {
        return Paths.curve(redWallFlower, laneControlAudience, laneControlRear, launchB).linear(redWallFlower, launchB);
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

    /** Emergency park: straight from wherever the robot is right now (the whole route stays left of the HIVE). */
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