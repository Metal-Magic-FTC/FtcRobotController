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
    private static final Pose SHOOT_POSE     = new Pose(90, 12, Math.toRadians(63.5));
    private static final Pose SHOOT2_POSE    = new Pose(90, 12, Math.toRadians(68));
    private static final Pose INTAKE1_START  = new Pose(90, 34, Math.toRadians(0));    // unchanged pickup lane entry
    private static final Pose INTAKE1_END    = new Pose(128, 34, Math.toRadians(0));   // unchanged pickup lane exit

    // PLACEHOLDER x3 - not verified field positions. I could not confidently
    // read exact coordinates off the marker sketch, so these are spaced along
    // the same wall (x=130) as a starting point. Adjust each Y in the Pedro
    // visualizer to match the three spots you actually want to bump.
    private static final Pose WALLBUMP_1 = new Pose(130, 10, Math.toRadians(0));
    private static final Pose WALLBUMP_2 = new Pose(130, 16, Math.toRadians(0));
    private static final Pose WALLBUMP_3 = new Pose(130, 22, Math.toRadians(0));

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
                                SHOOT2_POSE
                        )
                )
                .setLinearHeadingInterpolation(WALLBUMP_3.getHeading(), SHOOT2_POSE.getHeading())
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

    // Bump 2 of 3: starts where wallBump1() actually ends (WALLBUMP_1), slides to a
    // different spot on the same wall
    public PathChain wallBump2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                WALLBUMP_1,
                                WALLBUMP_2
                        )
                )
                .setLinearHeadingInterpolation(WALLBUMP_1.getHeading(), WALLBUMP_2.getHeading())
                .build();
    }

    // Bump 3 of 3: starts where wallBump2() actually ends (WALLBUMP_2), slides to a
    // third spot on the same wall
    public PathChain wallBump3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                WALLBUMP_2,
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
                                SHOOT_POSE,
                                new Pose(90, 20, Math.toRadians(75))
                        )
                )
                .setLinearHeadingInterpolation(SHOOT_POSE.getHeading(), Math.toRadians(75))
                .build();
    }
}