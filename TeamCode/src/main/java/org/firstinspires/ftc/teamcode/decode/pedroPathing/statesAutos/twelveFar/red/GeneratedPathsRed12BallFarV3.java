package org.firstinspires.ftc.teamcode.decode.pedroPathing.statesAutos.twelveFar.red;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class GeneratedPathsRed12BallFarV3 {

    private final Follower follower;

    // ---------------- START POSE ----------------
    // From the real red-far states reference (GeneratedPathsRedFarStates)
    public static final Pose START_POSE =
            new Pose(86, 8, Math.toRadians(90));

    // ---------------- SHARED WAYPOINTS (from the real red-far states reference) ----------------
    private static final Pose SHOOT_POSE     = new Pose(86, 7, Math.toRadians(63));
    private static final Pose INTAKE1_START  = new Pose(101, 26, Math.toRadians(0));
    private static final Pose INTAKE1_END    = new Pose(125, 23, Math.toRadians(0));
    private static final Pose INTAKE2_START  = new Pose(101, 53, Math.toRadians(0));
    private static final Pose INTAKE2_END    = new Pose(125, 53, Math.toRadians(0));
    private static final Pose LEAVE_END      = new Pose(110, 3, Math.toRadians(90));

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

    // Shot after row 1: robot is actually at INTAKE1_END, not START_POSE
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

    // Shot after row 2: robot is actually at INTAKE2_END, not START_POSE
    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                INTAKE2_END,
                                SHOOT_POSE
                        )
                )
                .setLinearHeadingInterpolation(INTAKE2_END.getHeading(), SHOOT_POSE.getHeading())
                .build();
    }

    // Row 1 entry: starts where shoot1() actually ends (SHOOT_POSE)
    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                SHOOT_POSE,
                                INTAKE1_START
                        )
                )
                .setLinearHeadingInterpolation(SHOOT_POSE.getHeading(), INTAKE1_START.getHeading())
                .build();
    }

    // Row 1 pickup sweep
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

    // Row 2 entry: starts where shoot2() actually ends (SHOOT_POSE)
    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                SHOOT_POSE,
                                INTAKE2_START
                        )
                )
                .setLinearHeadingInterpolation(SHOOT_POSE.getHeading(), INTAKE2_START.getHeading())
                .build();
    }

    // Row 2 pickup sweep
    public PathChain intake2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                INTAKE2_START,
                                INTAKE2_END
                        )
                )
                .setLinearHeadingInterpolation(INTAKE2_START.getHeading(), INTAKE2_END.getHeading())
                .build();
    }

    // Starts where shoot3() actually ends (SHOOT_POSE). Matches leaveNew() from the reference.
    public PathChain leave() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                SHOOT_POSE,
                                LEAVE_END
                        )
                )
                .setLinearHeadingInterpolation(SHOOT_POSE.getHeading(), LEAVE_END.getHeading())
                .build();
    }
}