package org.firstinspires.ftc.teamcode.decode.pedroPathing.optStatesAutos;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Alliance;

public class GeneratedPathsClose12Blue {

    private final Follower follower;

    // Poses
    public static final Pose startPose = new Pose(30, 128, Math.toRadians(0));
//    public Pose scanPose = new Pose(41, 105, Math.toRadians(50));
    public Pose scorePose = new Pose(30, 115, Math.toRadians(135));
    public Pose tointake1Pose = new Pose(33.5, 84, Math.toRadians(180));
    public Pose tointake1ControlPose = new Pose(48, 84, Math.toRadians(180));
    public Pose intake1Pose = new Pose(16, 84, Math.toRadians(180));
    public Pose gatePose = new Pose(17, 75, Math.toRadians(90));
    public Pose gateControlPose = new Pose(55, 73);
    public Pose tointake2Pose = new Pose(33.5, 60, Math.toRadians(180));
    public Pose tointake2ControlPose = new Pose(45, 56, Math.toRadians(180));
    public Pose intake2Pose = new Pose(16, 60, Math.toRadians(180));
    public Pose tointake3Pose = new Pose(33.5, 35, Math.toRadians(180));
    public Pose tointake3ControlPose = new Pose(45, 31, Math.toRadians(180));
    public Pose intake3Pose = new Pose(14, 34, Math.toRadians(180));
    public Pose score4ControlPose = new Pose(43, 78);
    public Pose leavePose = new Pose(48, 72, Math.toRadians(90));


    public GeneratedPathsClose12Blue(Follower follower) {
        this.follower = follower;
    }

//    public PathChain scan(){
//        return new PathBuilder(follower)
//                .addPath(new BezierCurve(startPose, scanPose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), scanPose.getHeading())
//                .build();
//    }

    public PathChain shoot() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
    }

    public PathChain toIntake1() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(scorePose, tointake1ControlPose, tointake1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), tointake1Pose.getHeading(), 0.3)
                .build();
    }

    public PathChain intake1() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(tointake1Pose, intake1Pose))
                .setLinearHeadingInterpolation(tointake1Pose.getHeading(), intake1Pose.getHeading(), 0.3)
                .build();
    }


    public PathChain gate() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(intake1Pose, gateControlPose, gatePose))
                .setBrakingStrength(3)
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), gatePose.getHeading(), 0.3)
                .build();
    }

    public PathChain shoot2() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(gatePose, scorePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading(), 0.3)
                .build();
    }

    public PathChain toIntake2() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(scorePose, tointake2ControlPose, tointake2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), tointake2Pose.getHeading(), 0.3)
                .build();
    }

    public PathChain intake2() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(tointake2Pose, intake2Pose))
                .setBrakingStrength(4)
                .setBrakingStart(20)
                .setLinearHeadingInterpolation(tointake2Pose.getHeading(), intake2Pose.getHeading(), 0.3)
                .build();
    }

    public PathChain shoot3() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(intake2Pose, scorePose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), scorePose.getHeading(), 0.3)
                .build();
    }

    public PathChain toIntake3() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(scorePose, tointake3ControlPose, tointake3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), tointake3Pose.getHeading(), 0.3)
                .build();
    }

    public PathChain intake3() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(tointake3Pose, intake3Pose))
                .setBrakingStrength(4)
                .setBrakingStart(20)
                .setLinearHeadingInterpolation(tointake3Pose.getHeading(), intake3Pose.getHeading(), 0.3)
                .build();
    }

    public PathChain shoot4() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(intake3Pose, score4ControlPose, scorePose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), scorePose.getHeading(), 0.3)
                .build();
    }

    public PathChain leave() {
        return new PathBuilder(follower)
                .addPath(new BezierCurve(scorePose, leavePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leavePose.getHeading(), 0.3)
                .build();
    }
}
