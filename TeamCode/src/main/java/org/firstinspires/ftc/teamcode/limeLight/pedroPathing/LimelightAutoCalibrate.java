//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.paths.PathBuilder;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//import java.util.List;
//
///**
// * Limelight Auto Calibrate  (PedroPathing v2 + Pinpoint)
// * ------------------------------------------------------
// * • Driver controls X/Y translation with gamepad1 left stick.
// * • Robot automatically rotates to face the visible AprilTag.
// * • Uses Limelight3A for tag data and PedroPathing v2 Follower for motion.
// */
//@TeleOp(name = "Limelight Auto Calibrate", group = "Sensor")
//public class LimelightAutoCalibrate extends OpMode {
//
//    private Limelight3A limelight;
//    private Follower follower;
//
//    // Simple PID gains for heading correction
//    private static final double kP_ROTATE = 0.035;
//    private static final double kI_ROTATE = 0.0;
//    private static final double kD_ROTATE = 0.001;
//
//    private double headingIntegral = 0;
//    private double lastHeadingError = 0;
//
//    @Override
//    public void init() {
//        telemetry.addLine("Initializing Limelight Auto-Calibrate...");
//
//        // --- Limelight setup ---
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.setPollRateHz(50);
//        limelight.start();
//        limelight.pipelineSwitch(3);   // use AprilTag pipeline
//
//        // --- PedroPathing v2 Follower with Pinpoint localization ---
//        follower = Constants.createFollower(hardwareMap);
//
//        telemetry.addLine("Ready — Press PLAY");
//        telemetry.update();
//    }
//
//    @Override
//    public void loop() {
//        // Update Pedro follower and localization
//        follower.update();
//
//        // Gamepad translation input (robot-centric)
//        double forward = -gamepad1.left_stick_y;
//        double strafe  =  gamepad1.left_stick_x;
//        double turn    = 0.0;
//
//        // --- Limelight AprilTag tracking ---
//        LLResult result = limelight.getLatestResult();
//        if (result != null && result.isValid()) {
//            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
//            if (tags != null && !tags.isEmpty()) {
//                LLResultTypes.FiducialResult tag = tags.get(0);
//                double xOffset = tag.getTargetXDegrees();   // horizontal offset
//
//                // PID correction toward 0° offset
//                double error = -xOffset;                    // flip sign if needed
//                headingIntegral += error;
//                double derivative = error - lastHeadingError;
//                lastHeadingError = error;
//
//                turn = kP_ROTATE * error
//                        + kI_ROTATE * headingIntegral
//                        + kD_ROTATE * derivative;
//
//                telemetry.addData("Tag ID", tag.getFiducialId());
//                telemetry.addData("Target X°", xOffset);
//                telemetry.addData("Turn Power", turn);
//            } else {
//                telemetry.addLine("No AprilTags visible");
//                resetPID();
//            }
//        } else {
//            telemetry.addLine("No Limelight data");
//            resetPID();
//        }
//
//        // --- Drive the robot (Pedro v2 uses drive(Pose) not setWeightedDrivePower) ---
//
//        Pose startPoseTemp = follower.getPose();
//
//        // Convert joystick movement into a *relative* field-space pose offset (in inches)
//        double moveScale = 5.0; // how far to move per joystick input (tweak this!)
//        Pose newPoseTemp = new Pose(
//                startPoseTemp.getX() + (forward * moveScale),
//                startPoseTemp.getY() + (strafe * moveScale),
//                startPoseTemp.getHeading() // keep heading the same
//        );
//
//        // Build a path to the new position
//        PathChain goToPos = new PathBuilder(follower)
//                .addPath(new BezierLine(startPoseTemp, newPoseTemp))
//                .setLinearHeadingInterpolation(startPoseTemp.getHeading(), newPoseTemp.getHeading())
//                .build();
//
//        // Follow the path
//        follower.followPath(goToPos);
//        follower.holdPoint(new Pose(forward, strafe, turn));
//
//
//        // --- Telemetry ---
//        Pose pose = follower.getPose();
//        telemetry.addData("Pose X (in)", pose.getX());
//        telemetry.addData("Pose Y (in)", pose.getY());
//        telemetry.addData("Heading (deg)", Math.toDegrees(pose.getHeading()));
//        telemetry.update();
//    }
//
//    private void resetPID() {
//        headingIntegral = 0;
//        lastHeadingError = 0;
//    }
//
//    @Override
//    public void stop() {
//        limelight.stop();
//        // maybe add a pedro stop here
//    }
//}