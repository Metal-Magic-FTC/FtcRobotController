package org.firstinspires.ftc.teamcode.limeLight.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "LimelightRotate", group = "PedroPathing")
public class LimelightRotateToTag extends OpMode {

    /** Create a Follower instance from Constants */
    private Follower follower;

    /** Limelight instance */
    private Limelight3A limelight;

    /** PathChain representing rotation motion */
    private PathChain rotationPath;

    /** Saved poses */
    private Pose currentPose;
    private Pose startPose;
    private Pose endPose;

    private boolean pathBuilt = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3); // AprilTag pipeline
        limelight.setPollRateHz(100);

        telemetry.addLine("LimelightRotate Initialized. Waiting for start...");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        follower.update();

        telemetry.addLine("LimelightRotate Ready");
        telemetry.addLine("Press start to rotate toward AprilTag.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setStartingPose(new Pose(0, 0, 0));

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                // Get first visible tag
                LLResultTypes.FiducialResult tag = fiducials.get(0);
                Pose3D robotToTag = tag.getRobotPoseTargetSpace();

                if (robotToTag != null) {
                    // Get yaw to tag (in radians)
                    double yawToTag = Math.toRadians(robotToTag.getOrientation().getYaw());
                    // Invert if camera is back-mounted
                    yawToTag = -yawToTag;

                    currentPose = follower.getPose();

                    double currentHeading = currentPose.getHeading();
                    double targetHeading = currentHeading + yawToTag;

                    // Add small positional offset so Pedro registers movement
                    double offset = 0.01; // inches
                    startPose = new Pose(currentPose.getX() - offset, currentPose.getY(), currentHeading);
                    endPose = new Pose(currentPose.getX() + offset, currentPose.getY(), targetHeading);

                    // Build rotation path
                    rotationPath = follower.pathBuilder()
                            .addPath(new BezierLine(startPose, endPose))
                            .setLinearHeadingInterpolation(currentHeading, targetHeading)
                            .build();

                    follower.followPath(rotationPath);
                    follower.followPath(rotationPath);
                    pathBuilt = true;

                    telemetry.addLine("✅ Rotation path built.");
                } else {
                    telemetry.addLine("❌ Robot→Tag pose unavailable.");
                }
            } else {
                telemetry.addLine("❌ No AprilTags detected.");
            }
        } else {
            telemetry.addLine("❌ Limelight result invalid.");
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();

        if (pathBuilt && follower.atParametricEnd()) {
            telemetry.addLine("✅ Finished rotation toward AprilTag!");
        }

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Following Path", !follower.atParametricEnd());
        telemetry.update();
    }
}
