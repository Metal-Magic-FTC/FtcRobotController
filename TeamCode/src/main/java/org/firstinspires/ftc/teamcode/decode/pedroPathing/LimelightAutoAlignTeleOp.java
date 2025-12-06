package org.firstinspires.ftc.teamcode.decode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "TeleOp + AprilTag One-Line Path (Corrected)", group = "PedroPathing")
@Disabled
public class LimelightAutoAlignTeleOp extends OpMode {

    private Limelight3A limelight;
    private Follower follower;
    private boolean autoAlignActive = false;

    // Distance to stop from the tag (meters)
    private static final double STANDOFF_METERS = 1.0;

    // store original pose when snapshot taken
    private Pose snapshotPose = null;
    private Pose targetPose = null;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);
        limelight.start();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleopDrive();

        telemetry.addLine("Ready — TeleOp + AprilTag One-Line Path (Corrected)");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();

        Pose currentPose = follower.getPose();
        if (currentPose == null) currentPose = new Pose(0, 0, 0);

        // Manual drive
        if (!autoAlignActive) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, false);
        }

        // Snapshot Limelight and create **one-line path**
        if (!autoAlignActive && gamepad1.a) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
                LLResultTypes.FiducialResult tag = result.getFiducialResults().get(0);
                Pose3D robotToTag = tag.getRobotPoseTargetSpace();
                if (robotToTag != null && robotToTag.getPosition() != null) {

                    snapshotPose = currentPose; // store original pose

                    // --- rear-mounted camera offsets ---
                    double offsetX = -robotToTag.getPosition().x; // rear camera: forward on camera = back on robot
                    double offsetY = robotToTag.getPosition().y;  // left on camera

                    // convert meters to inches
                    offsetX *= 39.37;
                    offsetY *= 39.37;

                    // rotate to field coordinates
                    double heading = currentPose.getHeading();
                    double fieldOffsetX = offsetX * Math.cos(heading) - offsetY * Math.sin(heading);
                    double fieldOffsetY = offsetX * Math.sin(heading) + offsetY * Math.cos(heading);

                    // normalize and apply standoff distance
                    double vectorLength = Math.hypot(fieldOffsetX, fieldOffsetY);
                    double standoffInches = STANDOFF_METERS * 39.37;
                    double scale = (vectorLength - standoffInches) / vectorLength;

                    double targetX = currentPose.getX() + fieldOffsetX * scale;
                    double targetY = currentPose.getY() + fieldOffsetY * scale;

                    targetPose = new Pose(targetX, targetY, 0); // facing forward

                    // --- CREATE SINGLE LINE PATH ---
                    PathChain path = follower.pathBuilder()
                            .addPath(new BezierLine(currentPose, targetPose))
                            .build();

                    follower.followPath(path);
                    autoAlignActive = true;

                    telemetry.addLine("Auto-align started: one-line path");
                } else {
                    telemetry.addLine("Limelight tag pose null");
                }
            } else {
                telemetry.addLine("No valid AprilTag detected");
            }
        }

        // Finish auto-align
        if (autoAlignActive && follower.atParametricEnd()) {
            autoAlignActive = false;
            follower.startTeleopDrive();
            telemetry.addLine("Auto-align complete");
        }

        // --- telemetry ---
        telemetry.addData("Mode", autoAlignActive ? "AUTO ALIGN" : "TELEOP");
        telemetry.addData("Current Pose (X,Y,°)", "%.1f, %.1f, %.1f",
                currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        if (snapshotPose != null) {
            telemetry.addData("Original Pose (X,Y,°)", "%.1f, %.1f, %.1f",
                    snapshotPose.getX(), snapshotPose.getY(), Math.toDegrees(snapshotPose.getHeading()));
        }
        if (targetPose != null) {
            telemetry.addData("Target Pose (X,Y,°)", "%.1f, %.1f, %.1f",
                    targetPose.getX(), targetPose.getY(), Math.toDegrees(targetPose.getHeading()));
        }
        telemetry.update();
    }
}