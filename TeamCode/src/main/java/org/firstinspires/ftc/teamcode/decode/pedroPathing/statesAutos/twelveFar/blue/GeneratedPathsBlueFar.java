package org.firstinspires.ftc.teamcode.decode.pedroPathing.statesAutos.twelveFar.blue;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;

public class GeneratedPathsBlueFar {

    private final Follower follower;

    // ---------------- START POSE ----------------

    //public static final Pose START_POSE = new Pose(118.157, 128.629, Math.toRadians(45));
    public static final Pose START_POSE =
            new Pose(63, 8, Math.toRadians(90));

    public static final Pose SCAN_POSE = new Pose(72, 50, Math.toRadians(90));

    public static final Pose SHOOT_POSE = new Pose(72, 50, Math.toRadians(90));

    public static final Pose TO_INTAKE_POSE = new Pose(72, 50, Math.toRadians(90));

    public static final Pose INTAKE_POSE = new Pose(72, 50, Math.toRadians(90));

    public static final Pose LEAVE_POSE = new Pose(72, 40, Math.toRadians(90));

    public GeneratedPathsBlueFar(Follower follower) {
        this.follower = follower;
    }

    // ---------------- PATHS ----------------

    public PathChain scan() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                START_POSE,
                                //new Pose(91.371, 102.034, Math.toRadians(45))
                                SCAN_POSE
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(90),
                        Math.toRadians(90)
                )
                .build();
    }

    public PathChain scan_to_shoot() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                SCAN_POSE,
                                //new Pose(91.371, 102.034, Math.toRadians(45))
                                SHOOT_POSE
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(90),
                        Math.toRadians(120)
                )
                .build();
    }

    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                INTAKE_POSE,
                                //new Pose(91.371, 102.034, Math.toRadians(45))
                                SHOOT_POSE
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(120)
                )
                .build();
    }


    public PathChain toIntake() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                SHOOT_POSE,
                                new Pose(45, 15),
                                TO_INTAKE_POSE
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(120),
                        Math.toRadians(180)
                )
                .build();
    }

    public PathChain intake() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                TO_INTAKE_POSE,
                                INTAKE_POSE
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public PathChain leave() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                SHOOT_POSE,
                                LEAVE_POSE
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(120),
                        Math.toRadians(90)
                )
                .build();
    }
}