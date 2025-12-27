package org.firstinspires.ftc.teamcode.decode.teleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "!Limelight Only Localization", group = "Test")
public class LimelightLocalization extends LinearOpMode {

    private Limelight3A limelight;

    // Stored pose when no AprilTag is visible
    private double lastX = 0;
    private double lastY = 0;
    private double lastHeading = 0;
    private boolean hasEverSeenTag = false;

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);
        limelight.start();

        telemetry.addLine("Limelight Only Localization");
        telemetry.addLine("No odometry. No IMU.");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                // Use MT1 (getBotpose) since we have no IMU for MT2
                Pose3D botpose = result.getBotpose();

                if (botpose != null) {

                    Position pos = botpose.getPosition();
                    YawPitchRollAngles angles = botpose.getOrientation();

                    // Update current pose
                    lastX = pos.x;
                    lastY = pos.y;
                    lastHeading = angles.getYaw();
                    hasEverSeenTag = true;

                    telemetry.addLine("=== LIVE POSE (Tag Visible) ===");
                    telemetry.addData("X", "%.3f m (%.1f in)", lastX, lastX * 39.3701);
                    telemetry.addData("Y", "%.3f m (%.1f in)", lastY, lastY * 39.3701);
                    telemetry.addData("Heading", "%.2f°", lastHeading);

                } else {
                    displayLastKnownPose("Pose null");
                }

            } else {
                displayLastKnownPose("No tag visible");
            }

            telemetry.update();
        }

        limelight.stop();
    }

    private void displayLastKnownPose(String reason) {
        if (hasEverSeenTag) {
            telemetry.addLine("=== LAST KNOWN POSE ===");
            telemetry.addData("Status", reason);
            telemetry.addData("X", "%.3f m (%.1f in)", lastX, lastX * 39.3701);
            telemetry.addData("Y", "%.3f m (%.1f in)", lastY, lastY * 39.3701);
            telemetry.addData("Heading", "%.2f°", lastHeading);
            telemetry.addLine();
            telemetry.addLine("WARNING: Position is stale!");
            telemetry.addLine("Find an AprilTag to update.");
        } else {
            telemetry.addLine("No AprilTag detected yet");
            telemetry.addLine("Point camera at an AprilTag");
        }
    }
}