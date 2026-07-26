package org.firstinspires.ftc.teamcode.decode.pedroPathing.statesAutos.twelveFar.red;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class GeneratedPathsRed12BallFarV3 {

    private final Follower follower;

    // ---------------- START POSE ----------------
    //public static final Pose START_POSE = new Pose(118.157, 128.629, Math.toRadians(45));
    public static final Pose START_POSE =
            new Pose(86, 8, Math.toRadians(90));

    // ---------------- SHARED WAYPOINTS (unchanged ball/target positions) ----------------
    private static final Pose SHOOT_POSE     = new Pose(90, 10.5, Math.toRadians(66.67));
    private static final Pose SHOOT2_POSE    = new Pose(90, 12, Math.toRadians(67));
    private static final Pose SHOOT3_POSE    = new Pose(90, 12, Math.toRadians(69));
    private static final Pose INTAKE1_START  = new Pose(90, 34, Math.toRadians(0));    // unchanged pickup lane entry
    private static final Pose INTAKE1_END    = new Pose(128, 34, Math.toRadians(0));   // unchanged pickup lane exit

    // Verified from Pedro visualizer export.
    private static final Pose WALLBUMP_1 = new Pose(130, 10, Math.toRadians(0));
    private static final Pose WALLBUMP_2 = new Pose(130, 20, Math.toRadians(0));
    private static final Pose WALLBUMP_3 = new Pose(130, 30, Math.toRadians(0));
    // Curve control points from the visualizer export (loop out toward center field between bumps)
    private static final Pose WALLBUMP_2_CONTROL = new Pose(88.064, 24.324);
    private static final Pose WALLBUMP_3_CONTROL = new Pose(87.866, 31.756);

    public GeneratedPathsRed12BallFarV3(Follower follower) {
        this.follower = follower;
    }

    // ---------------- PATHS ----------------

    // Preload shot: START_POSE -> SHOOT_POSE
    public PathChain shoot1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                START_POSE,
                                SHOOT_POSE
                        )
                )
                .setLinearHeadingInterpolation(START_POSE.getHeading(), SHOOT_POSE.getHeading())
                .build();
    }

    // Shot after intake1: robot is actually at INTAKE1_END, not START_POSE
    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                INTAKE1_END,
                                SHOOT2_POSE
                        )
                )
                .setLinearHeadingInterpolation(INTAKE1_END.getHeading(), SHOOT2_POSE.getHeading())
                .build();
    }

    // Shot after the 3-point wall bump: robot is actually at WALLBUMP_3, not START_POSE
    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                WALLBUMP_3,
                                SHOOT3_POSE
                        )
                )
                .setLinearHeadingInterpolation(WALLBUMP_3.getHeading(), SHOOT3_POSE.getHeading())
                .build();
    }

    // Starts where shoot1() actually ends (SHOOT_POSE), not the old hardcoded (90,14,75)
    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                SHOOT_POSE,
                                INTAKE1_START
                        )
                )
                .setLinearHeadingInterpolation(SHOOT_POSE.getHeading(), INTAKE1_START.getHeading())
                .build();
    }

    // Unchanged ball pickup line: starts where toIntake1() ends
    public PathChain intake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                INTAKE1_START,
                                INTAKE1_END
                        )
                )
                .setLinearHeadingInterpolation(INTAKE1_START.getHeading(), INTAKE1_END.getHeading())
                .build();
    }

    // Bump 1 of 3: starts where shoot2() actually ends (SHOOT2_POSE)
    public PathChain wallBump1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                SHOOT2_POSE,
                                WALLBUMP_1
                        )
                )
                .setLinearHeadingInterpolation(SHOOT2_POSE.getHeading(), WALLBUMP_1.getHeading())
                .build();
    }

    // Bump 2 of 3: starts where wallBump1() actually ends (WALLBUMP_1). Loops out
    // toward center field through the control point and back to the wall - matches
    // the Pedro visualizer export.
    public PathChain wallBump2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                WALLBUMP_1,
                                WALLBUMP_2_CONTROL,
                                WALLBUMP_2
                        )
                )
                .setLinearHeadingInterpolation(WALLBUMP_1.getHeading(), WALLBUMP_2.getHeading())
                .build();
    }

    // Bump 3 of 3: starts where wallBump2() actually ends (WALLBUMP_2). Loops out
    // toward center field through the control point and back to the wall - matches
    // the Pedro visualizer export.
    public PathChain wallBump3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                WALLBUMP_2,
                                WALLBUMP_3_CONTROL,
                                WALLBUMP_3
                        )
                )
                .setLinearHeadingInterpolation(WALLBUMP_2.getHeading(), WALLBUMP_3.getHeading())
                .build();
    }

    // Starts where shoot3() actually ends (SHOOT_POSE), not the old hardcoded (90,14,75)
    public PathChain leave() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                SHOOT3_POSE,
                                new Pose(90, 20, Math.toRadians(75))
                        )
                )
                .setLinearHeadingInterpolation(SHOOT3_POSE.getHeading(), Math.toRadians(75))
                .build();
    }
}