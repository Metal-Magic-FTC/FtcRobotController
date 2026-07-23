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
    // The pose the robot shoots from. Same physical spot every time, only the
    // path we drive TO it changes depending on where we're coming from.
    private static final Pose SHOOT_POSE     = new Pose(90, 12, Math.toRadians(69));
    private static final Pose INTAKE1_START  = new Pose(90, 34, Math.toRadians(0));    // unchanged pickup lane entry
    private static final Pose INTAKE1_END    = new Pose(128, 34, Math.toRadians(0));   // unchanged pickup lane exit
    private static final Pose WALLBUMP_END   = new Pose(133, 14, Math.toRadians(0));   // unchanged wall bump target
    private static final Pose BUMP_BACK_POSE = new Pose(118, 14, Math.toRadians(0));   // unchanged back-off point for re-bumping

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
                                SHOOT_POSE
                        )
                )
                .setLinearHeadingInterpolation(INTAKE1_END.getHeading(), SHOOT_POSE.getHeading())
                .build();
    }

    // Shot after the wall-bump sequence: robot is actually at WALLBUMP_END, not START_POSE
    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                WALLBUMP_END,
                                SHOOT_POSE
                        )
                )
                .setLinearHeadingInterpolation(WALLBUMP_END.getHeading(), SHOOT_POSE.getHeading())
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

    // Starts where shoot2() actually ends (SHOOT_POSE), not the old hardcoded (90,14,70).
    // This is bump #1 in the 3-bump sequence.
    public PathChain wallBump() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                SHOOT_POSE,
                                WALLBUMP_END
                        )
                )
                .setLinearHeadingInterpolation(SHOOT_POSE.getHeading(), WALLBUMP_END.getHeading())
                .build();
    }

    // Backs off the wall after a bump. Reusable every cycle since the robot's
    // actual pose is WALLBUMP_END exactly each time it's called.
    public PathChain backUp() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                WALLBUMP_END,
                                BUMP_BACK_POSE
                        )
                )
                .setLinearHeadingInterpolation(WALLBUMP_END.getHeading(), BUMP_BACK_POSE.getHeading())
                .build();
    }

    // Drives back into the wall after backUp(). Reusable every cycle since the
    // robot's actual pose is BUMP_BACK_POSE exactly each time it's called.
    // Used for bumps #2 and #3.
    public PathChain rebump() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                BUMP_BACK_POSE,
                                WALLBUMP_END
                        )
                )
                .setLinearHeadingInterpolation(BUMP_BACK_POSE.getHeading(), WALLBUMP_END.getHeading())
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