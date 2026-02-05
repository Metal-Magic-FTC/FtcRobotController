package org.firstinspires.ftc.teamcode.decode.pedroPathing.StatesAutosV2.StatesBlueAutos.BlueClose;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class BlueClosePaths {

    private final Follower follower;

    // ---------------- START POSE ----------------
    public final Pose START = np(113.8154613466334, 129.5561097256858, 90);

    public final Pose SHOOT = np(1, 1, 0);
    public final Pose SHOOTC = np(1, 1, 0);
    public final Pose SROW1 = np(1, 1, 0);
    public final Pose EROW1 = np(1, 1, 0);
    public final Pose SROW2 = np(1, 1, 0);
    public final Pose SROW2C = np(1, 1, 0);
    public final Pose EROW2 = np(1, 1, 0);
    public final Pose SROW3 = np(1, 1, 0);
    public final Pose SROW3C = np(1, 1, 0);
    public final Pose EROW3 = np(1, 1, 0);
    public final Pose GATE = np(1, 1, 0);

    public BlueClosePaths (Follower follower) {
        this.follower = follower;
    }

    // ---------------- PATHS ----------------

   // Path 1
    public PathChain shoot() {
        return to1L(SHOOT).build();
    }

    // Path 2
    public PathChain toIntake1() {
        return to1L(SROW1).build();
    }

    // Path 3
    public PathChain intake1() {
        return to1L(EROW1).build();
    }

    // Path 4
    public PathChain gate() {
        return to1(GATE).setConstantHeadingInterpolation(90).build();
    }

    // Path 5
    public PathChain shoot2() {
        return to1L(SHOOT).build();
    }

    // Path 6
    public PathChain toIntake2() {
        return to2(SHOOT, SROW2C, SROW2).build();
    }

    // Path 7
    public PathChain intake2() {
        return to1L(EROW2).build();
    }

    // Path 8
    public PathChain shoot3() {
        return to2T(EROW2, SHOOTC, SHOOT).build();
    }

    // Path 9
    public PathChain toIntake3() {
        return to2L(SHOOT, SROW3C, SROW3).build();
    }

    // Path 10
    public PathChain intake3() {
        return to1L(EROW3).build();
    }

    // Path 11
    public PathChain shoot4() {
        return to2T(EROW3, SHOOTC, SHOOT).build();
    }

    // Path 12
    public PathChain leave() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(96.75810473815463, 84, Math.toRadians(-90)),
                        new Pose(96.75810473815463, 72.44389027431421, Math.toRadians(-90))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(-90),
                        Math.toRadians(-90)
                )
                .build();
    }

    /**
     * This method is used to quickly create a degree-based pose using 3 values
     *
     * @param x The x value in inches (0-144)
     * @param y The y value in inches (0-144)
     * @param h The heading value in degrees (0-360)
     */
    public Pose np(double x, double y, double h) {
        return new Pose(x, y, Math.toRadians(h));
    }

    /**
     * This method is used to quickly create a radian-based pose using 3 values
     *
     * @param x The x value in inches (0-144)
     * @param y The y value in inches (0-144)
     * @param h The heading value in radians (0-6.283)
     */

    public Pose npR(double x, double y, double h) {
        return new Pose(x, y, h);
    }

    /**
     * This method is used to quickly create PathBuilder using a single destination pose value and no specific heading
     *
     * @param pose The immediate destination
     */

    public PathBuilder to1 (Pose pose) {
        return new PathBuilder(follower)
                .addPaths(new BezierLine(follower::getPose,pose));
    }

    /**
     * This method is used to quickly create PathBuilder using a single destination pose value and linear heading
     *
     * @param pose The immediate destination
    */

    public PathBuilder to1L (Pose pose) {
        return new PathBuilder(follower)
                .addPaths(new BezierLine(follower::getPose,pose))
                .setLinearHeadingInterpolation(follower.getHeading(), pose.getHeading());
    }

    /**
     * This method is used to quickly create PathBuilder using a single destination pose value and tangential heading
     *
     * @param pose The immediate destination
     */
    public PathBuilder to1T (Pose pose) {
        return new PathBuilder(follower)
                .addPaths(new BezierLine(follower::getPose,pose))
                .setTangentHeadingInterpolation();
    }

    /**
     * This method is used to quickly create PathBuilder using initial, control, and final poses with no specific heading
     *
     * @param poses Includes ALL poses including start, control, and destination with linear heading
     */

    public PathBuilder to2 (Pose... poses) {
        return new PathBuilder(follower)
                .addPaths(new BezierCurve(poses))
                .setLinearHeadingInterpolation(poses[0].getHeading(), poses[poses.length-1].getHeading());
    }

    /**
     * This method is used to quickly create PathBuilder using initial, control, and final poses with linear heading
     *
     * @param poses Includes ALL poses including start, control, and destination with linear heading
     */

    public PathBuilder to2L (Pose... poses) {
        return new PathBuilder(follower)
                .addPaths(new BezierCurve(poses))
                .setLinearHeadingInterpolation(poses[0].getHeading(), poses[poses.length-1].getHeading());
    }

    /**
     * This method is used to quickly create PathBuilder using initial, control, and final poses with tangential heading
     *
     * @param poses Includes ALL poses including start, control, and destination with tangential heading
     */

    public PathBuilder to2T (Pose... poses) {
        return new PathBuilder(follower)
                .addPaths(new BezierCurve(poses))
                .setTangentHeadingInterpolation();
    }

}
