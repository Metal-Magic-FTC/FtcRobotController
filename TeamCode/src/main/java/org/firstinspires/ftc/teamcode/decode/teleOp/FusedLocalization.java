package org.firstinspires.ftc.teamcode.decode.teleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

@TeleOp(name = "!Pedro * Limelight")
public class FusedLocalization extends LinearOpMode {

    private Follower follower;
    private Limelight3A limelight;

    // Track data source for telemetry
    private String positionSource = "None";

    // Constants for unit conversion
    private static final double METERS_TO_INCHES = 39.3701;
    private static final double DEGREES_TO_RADIANS = Math.PI / 180.0;

    @Override
    public void runOpMode() {

        // Initialize Pedro Pathing Follower
        // Adjust FConstants.class and LConstants.class to match your configuration
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(116.6988847583643, 128.83271375464685, 225));

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);
        limelight.start();

        telemetry.addLine("Pedro + Limelight Fusion Ready");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Update Pedro Pathing's localizer
            follower.update();

            // Get current pose from Pedro Pathing (inches, radians)
            Pose pedroPose = follower.getPose();

            // Check for Limelight AprilTag detection
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();

                if (botpose != null) {
                    Position pos = botpose.getPosition();
                    YawPitchRollAngles angles = botpose.getOrientation();

                    // Convert Limelight data to Pedro Pathing units
                    // Limelight: meters, degrees → Pedro: inches, radians
                    double limelightX = pos.x * METERS_TO_INCHES;
                    double limelightY = pos.y * METERS_TO_INCHES;
                    double limelightHeading = angles.getYaw() * DEGREES_TO_RADIANS;

                    // Create corrected pose for Pedro Pathing
                    Pose correctedPose = new Pose(limelightX, limelightY, limelightHeading);

                    // Correct Pedro Pathing's pose with Limelight's absolute position
                    follower.setStartingPose(correctedPose);

                    positionSource = "LIMELIGHT (Corrected)";

                    // Display fused position
                    displayPose(correctedPose, "FUSED (Limelight Correction)");

                } else {
                    positionSource = "PEDRO PATHING";
                    displayPose(pedroPose, "PEDRO PATHING (No botpose)");
                }

            } else {
                positionSource = "PEDRO PATHING";
                displayPose(pedroPose, "PEDRO PATHING (No tag)");
            }

            // Display raw data for debugging
            telemetry.addLine();
            telemetry.addLine("=== RAW PEDRO POSE ===");
            telemetry.addData("X", "%.2f in", pedroPose.getX());
            telemetry.addData("Y", "%.2f in", pedroPose.getY());
            telemetry.addData("Heading", "%.2f°", Math.toDegrees(pedroPose.getHeading()));

            telemetry.update();
        }

        limelight.stop();
    }

    private void displayPose(Pose pose, String source) {
        telemetry.addLine("=== CURRENT POSITION ===");
        telemetry.addData("Source", source);
        telemetry.addData("X", "%.2f in (%.3f m)", pose.getX(), pose.getX() / METERS_TO_INCHES);
        telemetry.addData("Y", "%.2f in (%.3f m)", pose.getY(), pose.getY() / METERS_TO_INCHES);
        telemetry.addData("Heading", "%.2f°", Math.toDegrees(pose.getHeading()));
    }
}