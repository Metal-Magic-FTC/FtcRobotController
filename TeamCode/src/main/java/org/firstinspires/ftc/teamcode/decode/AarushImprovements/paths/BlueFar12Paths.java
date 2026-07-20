package org.firstinspires.ftc.teamcode.decode.AarushImprovements.paths;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;

/**
 * Far-side 12-ball autonomous paths for Blue alliance (AarushImprovements copy).
 * 
 * <p>Blue Far starts near the far wall (opposite the Blue alliance station).
 * Coordinates are mirrored from Red across the X-axis (field center Y=72).</p>
 */
public class BlueFar12Paths {

    private final Follower follower;

    /** Blue far start pose: near far wall, facing field center */
    public static final Pose START_POSE =
            new Pose(118, 135, Math.toRadians(180)).mirror();

    private static Pose blue(Pose red) {
        return new Pose(red.getX(), -red.getY(), -red.getHeading());
    }

    public BlueFar12Paths(Follower follower) {
        this.follower = follower;
    }

    public PathChain scan() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        blue(new Pose(118, 135, Math.toRadians(180))),
                        blue(new Pose(107, 110, Math.toRadians(125)))))
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),  // mirrored 180
                        Math.toRadians(55))
                .build();
    }

    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        blue(new Pose(107, 110, Math.toRadians(125))),
                        blue(new Pose(96, 94, Math.toRadians(39.5)))))
                .setLinearHeadingInterpolation(
                        Math.toRadians(55),
                        Math.toRadians(140.5))
                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        blue(new Pose(96, 94, Math.toRadians(39.5))),
                        blue(new Pose(84.888, 84.289)),
                        blue(new Pose(103.2, 75.5, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(
                        Math.toRadians(140.5),
                        Math.toRadians(180))
                .build();
    }

    public PathChain intake1() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        blue(new Pose(103.2, 75.5, Math.toRadians(0))),
                        blue(new Pose(127, 75.5, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public PathChain gate() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        blue(new Pose(127, 75.5, Math.toRadians(0))),
                        blue(new Pose(130, 73, Math.toRadians(90)))))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }

    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        blue(new Pose(127, 75, Math.toRadians(90))),
                        blue(new Pose(105, 91, Math.toRadians(30)))))
                .setLinearHeadingInterpolation(
                        Math.toRadians(90),
                        Math.toRadians(150))
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        blue(new Pose(105, 91, Math.toRadians(30))),
                        blue(new Pose(68.871, 60.124)),
                        blue(new Pose(104.2, 52, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(
                        Math.toRadians(150),
                        Math.toRadians(180))
                .build();
    }

    public PathChain intake2() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        blue(new Pose(104.2, 52, Math.toRadians(0))),
                        blue(new Pose(128, 52, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        blue(new Pose(128, 52, Math.toRadians(0))),
                        blue(new Pose(109.057, 62.471)),
                        blue(new Pose(96, 94, Math.toRadians(39)))))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(141))
                .build();
    }

    public PathChain toIntake3() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(
                        blue(new Pose(96, 94, Math.toRadians(39))),
                        blue(new Pose(68.590, 35.7)),
                        blue(new Pose(102.6, 30, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(
                        Math.toRadians(141),
                        Math.toRadians(180))
                .build();
    }

    public PathChain intake3() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        blue(new Pose(102.6, 30, Math.toRadians(0))),
                        blue(new Pose(130, 30, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public PathChain shoot4() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        blue(new Pose(130, 30, Math.toRadians(0))),
                        blue(new Pose(96, 94, Math.toRadians(30)))))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(150))
                .build();
    }

    public PathChain leave() {
        return new PathBuilder(follower)
                .addPath(new BezierLine(
                        blue(new Pose(96, 94, Math.toRadians(30))),
                        blue(new Pose(117.350, 82.191, Math.toRadians(90)))))
                .setLinearHeadingInterpolation(
                        Math.toRadians(150),
                        Math.toRadians(90))
                .build();
    }
}