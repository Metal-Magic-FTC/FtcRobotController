package org.firstinspires.ftc.teamcode.decode.teleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@TeleOp(name = "!Limelight Rotate", group = "Test")
public class LimelightRotate extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);
        limelight.start();

        telemetry.addLine("Limelight initialized");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                // Get horizontal angle from camera centerline to target
                double tx = result.getTx();

                telemetry.addLine("=== HORIZONTAL ANGLE ===");
                telemetry.addData("TX", "%.2f°", tx);
                telemetry.addLine();
                telemetry.addLine("Negative = Target is LEFT");
                telemetry.addLine("Positive = Target is RIGHT");
                telemetry.addLine("Zero = Target is CENTERED");

            } else {
                telemetry.addLine("No AprilTag detected");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}