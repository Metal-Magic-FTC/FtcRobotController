package org.firstinspires.ftc.teamcode.decode.AarushImprovements.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

/**
 * Generated paths for the Blue close 12-ball sorted autonomous. Built by
 * mirroring the Red set across the X-axis (the {@link #blue(Pose)} helper
 * handles the flip).
 */
public class BlueClose12Paths {

    private final Follower follower;

    /**
     * In the Pedropathing field coordinate system, the Blue alliance is the
     * Red alliance mirrored across the X-axis. This helper does that flip.
     */
    private static Pose blue(Pose red) {
        return new Pose(red.getX(), -red.getY(), -red.getHeading());
    }

    private static final Pose RED_START =
            new Pose(118, 122.2926713735558, Math.toRadians(180));
    public static final Pose START_POSE = blue(RED_START);

    public BlueClose12Paths(Follower follower) {
        this.follower = follower;
    }

    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        blue(new Pose(118, 122.2926713735558, Math.toRadians(180))),
                        blue(new Pose(96, 94, Math.toRadians(35)))))
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-35))
                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        blue(new Pose(96, 94, Math.toRadians(35))),
                        blue(new Pose(84.88846763536056, 84.28908937098198, 0)),
                        blue(new Pose(103.2, 79, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(-35), Math.toRadians(0))
                .build();
    }

    public PathChain intake1() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        blue(new Pose(103.2, 79, Math.toRadians(0))),
                        blue(new Pose(128, 79, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(0, 0)
                .build();
    }

    public PathChain gate() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        blue(new Pose(128, 79, Math.toRadians(0))),
                        blue(new Pose(129, 76, Math.toRadians(90)))))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();
    }

    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        blue(new Pose(127, 78, Math.toRadians(90))),
                        blue(new Pose(96, 94, Math.toRadians(30)))))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-30))
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        blue(new Pose(96, 94, Math.toRadians(33))),
                        blue(new Pose(68.8707865168539, 60.123595505617985, 0)),
                        blue(new Pose(104.2, 55, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(0))
                .build();
    }

    public PathChain intake2() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        blue(new Pose(104.2, 55, Math.toRadians(0))),
                        blue(new Pose(128, 55, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(0, 0)
                .build();
    }

    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        blue(new Pose(128, 55, Math.toRadians(0))),
                        blue(new Pose(109.05707981278215, 62.47068594135381, 0)),
                        blue(new Pose(96, 94, Math.toRadians(39)))))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-39))
                .build();
    }

    public PathChain toIntake3() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        blue(new Pose(96, 94, Math.toRadians(39))),
                        blue(new Pose(68.58988764044946, 35.7, 0)),
                        blue(new Pose(104.2, 30, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(-39), Math.toRadians(0))
                .build();
    }

    public PathChain intake3() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        blue(new Pose(104.2, 30, Math.toRadians(0))),
                        blue(new Pose(130, 30, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(0, 0)
                .build();
    }

    public PathChain shoot4() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        blue(new Pose(130, 30, Math.toRadians(0))),
                        blue(new Pose(96, 94, Math.toRadians(30)))))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))
                .build();
    }

    public PathChain leave() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        blue(new Pose(96, 94, Math.toRadians(30))),
                        blue(new Pose(117.3496136071887, 82.19084724005137, Math.toRadians(90)))))
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-90))
                .build();
    }
}
