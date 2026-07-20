package org.firstinspires.ftc.teamcode.decode.pedroPathing.statesAutos.twelveFar.blue;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;

/**
 * Far-side 12-ball auto.
 */
public class GeneratedPathsBlue12BallFarV1 {

    private final Follower follower;

    // ---------------- START POSE ----------------
    // Blue Far starts near the far wall (positive Y), facing toward field center
    // X=118 is near the right wall, Y=135 is near far wall
    public static final Pose START_POSE =
            new Pose(118, 135, Math.toRadians(180)).mirror();

    public GeneratedPathsBlue12BallFarV1(Follower follower) {
        this.follower = follower;
    }

    // ---------------- PATHS ----------------

    public PathChain scan() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        START_POSE,
                        new Pose(107, 110, Math.toRadians(125)).mirror()))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),  // start heading (mirrored)
                        Math.toRadians(55))   // end heading (180-125=55, mirrored)
                .build();
    }

    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(107, 110, Math.toRadians(125)).mirror(),
                        new Pose(96, 94, Math.toRadians(39.5)).mirror()))
                .setLinearHeadingInterpolation(
                        Math.toRadians(55),   // 180-125
                        Math.toRadians(140.5)) // 180-39.5
                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        new Pose(96, 94, Math.toRadians(39.5)).mirror(),
                        new Pose(84.888, 84.289).mirror(),
                        new Pose(103.2, 75.5, Math.toRadians(0)).mirror()))
                .setLinearHeadingInterpolation(
                        Math.toRadians(140.5), // 180-39.5
                        Math.toRadians(180))   // 180-0
                .build();
    }

    public PathChain intake1() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(103.2, 75.5, Math.toRadians(0)).mirror(),
                        new Pose(127, 75.5, Math.toRadians(0)).mirror()))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public PathChain gate() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        new Pose(127, 75.5, Math.toRadians(0)).mirror(),
                        new Pose(130, 73, Math.toRadians(90)).mirror()))
                .setConstantHeadingInterpolation(Math.toRadians(90)) // 180-90
                .build();
    }

    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(127, 75, Math.toRadians(90)).mirror(),
                        new Pose(105, 91, Math.toRadians(30)).mirror()))
                .setLinearHeadingInterpolation(
                        Math.toRadians(90),   // 180-90
                        Math.toRadians(150))  // 180-30
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        new Pose(105, 91, Math.toRadians(30)).mirror(),
                        new Pose(68.871, 60.124).mirror(),
                        new Pose(104.2, 52, Math.toRadians(0)).mirror()))
                .setLinearHeadingInterpolation(
                        Math.toRadians(150), // 180-30
                        Math.toRadians(180)) // 180-0
                .build();
    }

    public PathChain intake2() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(104.2, 52, Math.toRadians(0)).mirror(),
                        new Pose(128, 52, Math.toRadians(0)).mirror()))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        new Pose(128, 52, Math.toRadians(0)).mirror(),
                        new Pose(109.057, 62.471).mirror(),
                        new Pose(96, 94, Math.toRadians(39)).mirror()))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180), // 180-0
                        Math.toRadians(141)) // 180-39
                .build();
    }

    public PathChain toIntake3() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        new Pose(96, 94, Math.toRadians(39)).mirror(),
                        new Pose(68.590, 35.7).mirror(),
                        new Pose(102.6, 30, Math.toRadians(0)).mirror()))
                .setLinearHeadingInterpolation(
                        Math.toRadians(141), // 180-39
                        Math.toRadians(180)) // 180-0
                .build();
    }

    public PathChain intake3() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(102.6, 30, Math.toRadians(0)).mirror(),
                        new Pose(130, 30, Math.toRadians(0)).mirror()))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public PathChain shoot4() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(130, 30, Math.toRadians(0)).mirror(),
                        new Pose(96, 94, Math.toRadians(30)).mirror()))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180), // 180-0
                        Math.toRadians(150)) // 180-30
                .build();
    }


    public PathChain leave() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(96, 94, Math.toRadians(30)).mirror(),
                        new Pose(117.350, 82.191, Math.toRadians(90)).mirror()))
                .setLinearHeadingInterpolation(
                        Math.toRadians(150), // 180-30
                        Math.toRadians(90))  // 180-90
                .build();
    }
}