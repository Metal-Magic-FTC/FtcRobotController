package org.firstinspires.ftc.teamcode.decode.teleOp.states.tests.limelightV2;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.GoBildaPinpointDriver;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

import java.util.List;
import java.util.function.Supplier;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.pedropathing.control.*;
public class FusedPose {
    private Limelight3A limelight;
    private Follower follower;
    private KalmanFilter filterX;
    private KalmanFilter filterY;
    private KalmanFilter filterHeading;
    private Pose startingPose;
    private Pose currentPose;
    private LLResult result;
    private Pose limelightEstimation;
    public FusedPose(HardwareMap hardwareMap, Pose pose) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);
        limelight.setPollRateHz(100);
        limelight.start();

        follower = Constants.createFollower(hardwareMap);
        filterX = new KalmanFilter(new KalmanFilterParameters(6, 1), pose.getX(), 6, 1);
        filterY = new KalmanFilter(new KalmanFilterParameters(6, 1), pose.getY(), 6, 1);
        filterHeading = new KalmanFilter(new KalmanFilterParameters(6, 1), pose.getHeading(), 6, 1);
        startingPose = new Pose(
                pose.getX(),
                pose.getY(),
                pose.getHeading()
        );
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
    }

    /*
    * LIMELIGHT METHODS
    */

    // Returns limelight object
    public Limelight3A getLimelight() {
        return limelight;
    }

    // Returns estimated limelight position if goal tag is detected otherwise null is returned
    public Pose getRobotPose(boolean convert) {
        result = limelight.getLatestResult();
        if (result == null || !result.isValid())
            return null;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId();
            if (id == 20 || id == 24) {
                Pose3D robotPose3D = fiducial.getRobotPoseFieldSpace();
                limelightEstimation = new Pose(
                        robotPose3D.getPosition().x,
                        robotPose3D.getPosition().y,
                        robotPose3D.getOrientation().getYaw()
                );
                if (convert)
                    return limelightToPedroPathing(limelightEstimation);
                return limelightEstimation;
            }
        }
        return null;
    }

    /*
    * POSE METHODS
     */

    // Converts limelight pose to pedropathing pose
    public Pose limelightToPedroPathing(Pose current) {
        return new Pose(
                72+39.3701*current.getY(),
                72-39.3701*current.getX(),
                Math.toRadians(current.getHeading()-90)
        );
    }

    public Pose pedroPathingToLimelight(Pose current) {
        return new Pose(

        );
    }

    /*
    * FOLLOWER METHODS
     */

    // Returns follower object
    public Follower getFollower() {
        return follower;
    }

    // Returns starting pose
    public Pose getStartingPose() {
        return startingPose;
    }

    // Fuse limelight estimation with follower data
    public Pose mergePoses(boolean estimate) {
        if (estimate)
            getRobotPose(false);
        if (limelightEstimation != null) {
            Pose convertedPose = limelightToPedroPathing(limelightEstimation);
            filterX.update(convertedPose.getX(), follower.getPose().getX());
            filterY.update(convertedPose.getY(), follower.getPose().getY());
            filterHeading.update(convertedPose.getHeading(), follower.getPose().getHeading());
            return new Pose(
                    filterX.getState(),
                    filterY.getState(),
                    filterHeading.getState()
            );
        }
        return follower.getPose();
    }

    public void update() {
        follower.update();
    }
}
