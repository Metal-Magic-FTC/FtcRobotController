package org.firstinspires.ftc.teamcode.decode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "Rotate To Tag HEHEHHEHEHA", group = "PedroPathing")
@Disabled
public class RotateToTag extends OpMode {

    private Follower follower;
    private Limelight3A limelight;

    private boolean rotatingToTag = false;
    private double targetHeading = 0;
    private double iterate = 0;
    private boolean right = false;
    Pose startPose = new Pose (0,0,0);
    Pose endPose = new Pose (0,0,0);
    PathChain rotationPath;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3); // AprilTag pipeline

        telemetry.addLine("Rear Limelight + PedroPathing TeleOp Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        follower.update();

        // ========== Normal Drive ==========
        if (!rotatingToTag) {

            // ========== Press A to face tag ==========
            if (gamepad1.a) {
                LLResult llResult = limelight.getLatestResult();

                if (llResult != null && llResult.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
                    if (!fiducials.isEmpty()) {
                        // Pick the first detected tag
                        LLResultTypes.FiducialResult tag = fiducials.get(0);

                        // Pose of the tag relative to the robot
                        //Pose3D robotToTag = tag.getRobotPoseTargetSpace();

                        // Yaw from Limelight (tag relative to robot)
                        double yawToTag = tag.getTargetXDegrees();

                        // Because Limelight is mounted facing backwards, flip the direction
                        // and add 180° so the robot front faces the tag.
                        //yawToTag = -yawToTag + Math.PI;

                        double currentHeading = follower.getPose().getHeading();

                        if (!rotatingToTag) {
                            targetHeading = normalizeAngle(currentHeading + yawToTag);
                            rotatingToTag = true;
                            if (yawToTag>0) {
                                right = true;
                            } else {
                                right = false;
                            }
                        }
                        while (targetHeading<yawToTag) {
                            if (right) {
                                iterate+=1;
                            } else {
                                iterate-=1;
                            }
                            rotationPath = follower.pathBuilder()
                                    .addPath(new BezierLine(startPose, endPose))
                                    .setLinearHeadingInterpolation(currentHeading, targetHeading+iterate)
                                    .build();
                            follower.followPath(rotationPath);
                            telemetry.addLine("Limelight: rotating to face tag...");
                        }
                        iterate = 0;
                    } else {
                        telemetry.addLine("No AprilTags detected.");
                    }
                } else {
                    telemetry.addLine("No valid Limelight result.");
                }
            }
        }

        // ========== Rotate until facing tag ==========
        else {
//            double currentHeading = follower.getPose().getHeading();
//            double error = normalizeAngle(targetHeading - currentHeading);
//            double rotPower = Math.copySign(ROTATE_SPEED, error);
//
//            if (Math.abs(error) < HEADING_TOLERANCE) {
//                rotPower = 0;
//                rotatingToTag = false;
//                telemetry.addLine("Rotation complete.");
//            }
//
//            follower.setTeleOpDrive(0, 0, rotPower, false);
        }

        // ========== Telemetry ==========
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Target Heading (deg)", Math.toDegrees(targetHeading));
        telemetry.addData("RotatingToTag", rotatingToTag);
        telemetry.update();
    }

    /** Normalize angle to [-π, π] */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
