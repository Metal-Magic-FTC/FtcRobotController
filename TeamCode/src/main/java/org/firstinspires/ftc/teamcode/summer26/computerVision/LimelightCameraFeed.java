package org.firstinspires.ftc.teamcode.summer26.computerVision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

/**
 * LimelightCameraFeed
 *
 * Streams the Limelight 3A camera feed to the Driver Station display and
 * overlays basic targeting information (crosshair, pipeline ID, FPS).
 *
 * INTO THE DEEP game pieces:
 *   - Yellow Samples  → use pipeline index 0 (HSV tuned for yellow)
 *   - Red Specimens   → use pipeline index 1 (HSV tuned for red)
 *   - Blue Specimens  → use pipeline index 2 (HSV tuned for blue)
 *
 * Setup in Limelight web UI (http://10.0.0.11:5801):
 *   1. Create Color Thresholding pipelines for each piece type.
 *   2. Set "Send to Dashboard" = ON for all pipelines.
 *   3. Note each pipeline index (0-based) and update constants below.
 *
 * Driver Station display:
 *   The Limelight streams MJPEG video automatically once limelight.start()
 *   is called. In the DS app go to: Camera Stream → select the Limelight.
 */
@TeleOp(name = "Limelight Camera Feed", group = "Vision")
public class LimelightCameraFeed extends LinearOpMode {

    // ── Pipeline indices ──────────────────────────────────────────────────────
    private static final int PIPELINE_YELLOW_SAMPLE = 0;
    private static final int PIPELINE_RED_SPECIMEN  = 1;
    private static final int PIPELINE_BLUE_SPECIMEN = 2;

    // dpad_up / dpad_down on gamepad1 to cycle through pipelines.

    private Limelight3A limelight;
    private int         currentPipeline = PIPELINE_YELLOW_SAMPLE;

    @Override
    public void runOpMode() {

        // ── Hardware init ─────────────────────────────────────────────────────
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(currentPipeline);
        limelight.start();

        telemetry.addData("Status",        "Limelight initialized — pipeline %d", currentPipeline);
        telemetry.addData("Camera stream", "Open DS Camera Stream tab to view feed");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ── Pipeline switching ────────────────────────────────────────────
            if (gamepad1.dpad_up && !gamepad1.dpad_down) {
                currentPipeline = (currentPipeline + 1) % 3;
                limelight.pipelineSwitch(currentPipeline);
                sleep(200);
            } else if (gamepad1.dpad_down && !gamepad1.dpad_up) {
                currentPipeline = (currentPipeline + 2) % 3;
                limelight.pipelineSwitch(currentPipeline);
                sleep(200);
            }

            // ── Pull latest result ────────────────────────────────────────────
            LLResult result = limelight.getLatestResult();

            // ── Status lines ──────────────────────────────────────────────────
            LLStatus status = limelight.getStatus();
            telemetry.addData("Pipeline", pipelineName(currentPipeline) + "  (dpad ↑↓ to switch)");
            telemetry.addData("FPS",      "%.1f", status.getFps());
            telemetry.addData("Temp (°C)","%.0f", status.getTemp());

            if (result != null && result.isValid()) {

                List<LLResultTypes.ColorResult> colorTargets = result.getColorResults();
                telemetry.addData("─── Detections", colorTargets.size());

                for (int i = 0; i < colorTargets.size(); i++) {
                    LLResultTypes.ColorResult t = colorTargets.get(i);

                    // tx / ty are degrees from crosshair (-29.8° to +29.8°)
                    telemetry.addData(
                            String.format("  [%d] tx/ty", i),
                            "%.2f° / %.2f°", t.getTargetXDegrees(), t.getTargetYDegrees());

                    // ta is percentage of image area (0–100%)
                    telemetry.addData(
                            String.format("  [%d] area", i),
                            "%.2f%%", t.getTargetArea());
                }

            } else {
                telemetry.addData("Target", "None detected");
            }

            telemetry.addLine();
            telemetry.addData("Controls", "dpad↑ next pipeline | dpad↓ prev pipeline");
            telemetry.update();
        }

        limelight.stop();
    }

    private String pipelineName(int index) {
        switch (index) {
            case PIPELINE_YELLOW_SAMPLE: return "Yellow Sample (0)";
            case PIPELINE_RED_SPECIMEN:  return "Red Specimen  (1)";
            case PIPELINE_BLUE_SPECIMEN: return "Blue Specimen (2)";
            default:                     return "Unknown       (" + index + ")";
        }
    }
}