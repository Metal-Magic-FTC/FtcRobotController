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
            new Pose(56, 8, Math.toRadians(90));

    public static final Pose SHOOT_POSE = new Pose(58, 20, Math.toRadians(110));

    public static final Pose TO_INTAKE_HUMAN_POSE = new Pose(35, 10, Math.toRadians(180));

    public static final Pose INTAKE_HUMAN_POSE = new Pose(20, 10, Math.toRadians(180));

    public static final Pose TO_INTAKE_ROW_POSE = new Pose(40, 35, Math.toRadians(180));
    public static final Pose INTAKE_ROW_POSE = new Pose(20, 35, Math.toRadians(180));

    public static final Pose LEAVE_POSE = new Pose(72, 40, Math.toRadians(90));

    public GeneratedPathsBlueFar(Follower follower) {
        this.follower = follower;
    }

    // ---------------- PATHS ----------------

    public PathChain start_to_shoot() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                START_POSE,
                                SHOOT_POSE
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(START_POSE.getHeading()),
                        Math.toRadians(SHOOT_POSE.getHeading())
                )
                .build();
    }

    public PathChain human_to_shoot() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                INTAKE_HUMAN_POSE,
                                SHOOT_POSE
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(INTAKE_HUMAN_POSE.getHeading()),
                        Math.toRadians(SHOOT_POSE.getHeading())
                )
                .build();
    }

    public PathChain row_to_shoot() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                INTAKE_ROW_POSE,
                                SHOOT_POSE
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(INTAKE_ROW_POSE.getHeading()),
                        Math.toRadians(SHOOT_POSE.getHeading())
                )
                .build();
    }


    public PathChain human_toIntake() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                SHOOT_POSE,
                                new Pose(45, 15),
                                TO_INTAKE_HUMAN_POSE
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(SHOOT_POSE.getHeading()),
                        Math.toRadians(TO_INTAKE_HUMAN_POSE.getHeading())
                )
                .build();
    }

    public PathChain human_intake() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                TO_INTAKE_HUMAN_POSE,
                                INTAKE_HUMAN_POSE
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(TO_INTAKE_HUMAN_POSE.getHeading()), Math.toRadians(INTAKE_HUMAN_POSE.getHeading()))
                .build();
    }

    public PathChain row_toIntake() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                SHOOT_POSE,
                                TO_INTAKE_HUMAN_POSE
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(SHOOT_POSE.getHeading()), Math.toRadians(TO_INTAKE_HUMAN_POSE.getHeading()))
                .build();
    }

    public PathChain row_intake() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                TO_INTAKE_ROW_POSE,
                                INTAKE_ROW_POSE
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(TO_INTAKE_ROW_POSE.getHeading()), Math.toRadians(INTAKE_ROW_POSE.getHeading()))
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
                        Math.toRadians(SHOOT_POSE.getHeading()),
                        Math.toRadians(LEAVE_POSE.getHeading())
                )
                .build();
    }
}