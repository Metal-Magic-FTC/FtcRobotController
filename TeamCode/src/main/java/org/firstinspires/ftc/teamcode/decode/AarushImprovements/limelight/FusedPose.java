package org.firstinspires.ftc.teamcode.decode.AarushImprovements.limelight;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.GoBildaPinpointDriver;

import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import java.util.List;

public class FusedPose {
    public Limelight3A limelight;
    public Follower follower;
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

    /* LIMELIGHT METHODS */

    public Limelight3A getLimelight() {
        return limelight;
    }

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

    public int detectLimelightTag() {
        result = limelight.getLatestResult();
        if (result == null || !result.isValid())
            return -1;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId();
            if (21 <= id && id <= 23) {
                return id;
            }
        }
        return -1;
    }

    /* POSE METHODS */

    public Pose limelightToPedroPathing(Pose current) {
        return new Pose(
                72 + 39.3701 * current.getY(),
                72 - 39.3701 * current.getX(),
                Math.toRadians(current.getHeading() - 90)
        );
    }

    public Pose pedroPathingToLimelight(Pose current) {
        return new Pose(
                // intentionally empty in the original — filled when field mapping is known
        );
    }

    /* FOLLOWER METHODS */

    public Follower getFollower() {
        return follower;
    }

    public Pose getStartingPose() {
        return startingPose;
    }

    public Pose mergePoses(boolean estimate) {
        if (estimate)
            getRobotPose(false);
        if (limelightEstimation != null) {
            Pose convertedPose = limelightToPedroPathing(limelightEstimation);
            filterX.update(follower.getPose().getX() - filterX.getState(), convertedPose.getX());
            filterY.update(follower.getPose().getY() - filterY.getState(), convertedPose.getY());
            filterHeading.update(follower.getPose().getHeading() - filterHeading.getState(), convertedPose.getHeading());
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
