package org.firstinspires.ftc.teamcode.decode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@TeleOp(name = "Pedro FTC Vision Odo", group = "Test")
public class PedroVisionOdoUpdate extends LinearOpMode {

    private Follower follower;
    private Limelight3A limelight;

    // === FIELD POSE OF TAG 24 (CHANGE THIS) ===
    private static final double TAG_X = 0;     // inches
    private static final double TAG_Y = 48;    // inches
    private static final double TAG_HEADING = Math.PI; // 180 deg

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3); // AprilTag pipeline
        limelight.start();

        telemetry.addLine("FTC Limelight Vision Odo Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            follower.update();

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {

                    if (tag.getFiducialId() == 24) {

                        Pose3D fieldPose = tag.getRobotPoseFieldSpace();

                        if (fieldPose != null) {
                            double xIn = fieldPose.getPosition().x * 39.3701;
                            double yIn = fieldPose.getPosition().y * 39.3701;
                            double headingRad = fieldPose.getOrientation().getYaw();

                            follower.setPose(new Pose(xIn, yIn, headingRad));
                        }
                    }
                }
            }

            Pose pose = follower.getPose();

            telemetry.addData("X (in)", pose.getX());
            telemetry.addData("Y (in)", pose.getY());
            telemetry.addData("Heading (deg)", Math.toDegrees(pose.getHeading()));
            telemetry.update();
        }
    }
}
