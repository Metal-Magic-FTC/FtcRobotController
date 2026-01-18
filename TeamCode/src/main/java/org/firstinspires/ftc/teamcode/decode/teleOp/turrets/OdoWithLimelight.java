package org.firstinspires.ftc.teamcode.decode.teleOp.turrets;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@TeleOp(name = "OdoWithLimelight")
public class OdoWithLimelight extends LinearOpMode {

    private static final int TARGET_TAG_ID = 24;
    private static final double METERS_TO_INCHES = 39.3701;

    // Example: tag at origin, facing along +Y
    private static final Pose TAG_24_FIELD_POSE = new Pose(0, 24, Math.toRadians(270));

    private Limelight3A limelight;
    private Follower follower;

    @Override
    public void runOpMode() {

        // ===== Limelight =====
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3); // AprilTag pipeline
        limelight.setPollRateHz(50);
        limelight.start();

        // ===== Pedro Pathing =====
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(0, 0, Math.toRadians(90))); // start pose

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update odometry
            follower.update();

            // Auto-correct odometry if tag is visible
            correctOdometryFromTag();

            // Telemetry
            Pose p = follower.getPose();
            telemetry.addData("X", "%.2f", p.getX());
            telemetry.addData("Y", "%.2f", p.getY());
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(p.getHeading()));
            telemetry.update();
        }
    }

    private void correctOdometryFromTag() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {

            if (tag.getFiducialId() != TARGET_TAG_ID) continue;

            Pose3D robotInTag = tag.getRobotPoseTargetSpace();
            if (robotInTag == null) return;

            // Robot pose relative to tag (Limelight frame)
            double rx = robotInTag.getPosition().x * METERS_TO_INCHES;
            double ry = robotInTag.getPosition().y * METERS_TO_INCHES;
            double rYaw = Math.toRadians(robotInTag.getOrientation().getYaw());

            // Convert to field coordinates
            double cosF = Math.cos(TAG_24_FIELD_POSE.getHeading());
            double sinF = Math.sin(TAG_24_FIELD_POSE.getHeading());

            double fieldX = TAG_24_FIELD_POSE.getX() - (rx * cosF - ry * sinF);
            double fieldY = TAG_24_FIELD_POSE.getY() - (rx * sinF + ry * cosF);
            double fieldHeading = normalizeRadians(TAG_24_FIELD_POSE.getHeading() + rYaw);

            follower.setPose(new Pose(fieldX, fieldY, fieldHeading));
        }
    }

    private double normalizeRadians(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }
}