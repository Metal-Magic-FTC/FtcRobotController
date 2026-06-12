package org.firstinspires.ftc.teamcode.summer26.computerVision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * LimelightTelemetry
 *
 * Outputs full detection coordinates for INTO THE DEEP game pieces as
 * Driver Station telemetry. No camera feed is displayed — this OpMode
 * is optimised for tuning and reading values from the field.
 *
 * Telemetry fields per detected target:
 *   tx  – horizontal offset from crosshair, degrees  (+ = right)
 *   ty  – vertical offset from crosshair, degrees    (+ = up)
 *   ta  – target area as % of image                  (proxy for distance)
 *   px  – bounding box center X in pixels (when showRawPixels = true)
 *   py  – bounding box center Y in pixels (when showRawPixels = true)
 *
 * 3D Pose (only meaningful when mount height + angle are calibrated):
 *   X / Y / Z translation in meters
 *
 * Limelight mount calibration (required for botpose):
 *   In the Limelight web UI → General Settings:
 *     Lens Height From Ground: e.g. 0.25 (meters)
 *     Mounting Angle (pitch):  e.g. 30.0 (degrees downward)
 */
@TeleOp(name = "Limelight Telemetry", group = "Vision")
public class LimelightTelemetry extends LinearOpMode {

    // ── Pipeline indices ──────────────────────────────────────────────────────
    private static final int PIPELINE_YELLOW_SAMPLE = 0;
    private static final int PIPELINE_RED_SPECIMEN  = 1;
    private static final int PIPELINE_BLUE_SPECIMEN = 2;

    // Ignore targets whose area is below this threshold (noise rejection).
    private static final double MIN_TARGET_AREA = 0.15; // percent of image

    private Limelight3A limelight;
    private int         currentPipeline = PIPELINE_YELLOW_SAMPLE;
    private boolean     showRawPixels   = false; // toggle with gamepad1.x

    @Override
    public void runOpMode() {

        // ── Hardware init ─────────────────────────────────────────────────────
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(currentPipeline);
        limelight.start();

        telemetry.addData("Status",  "Ready — waiting for start");
        telemetry.addData("Gamepad", "dpad↑↓ = pipeline  |  X = toggle raw pixels");
        telemetry.update();

        waitForStart();

        boolean lastX = false;

        while (opModeIsActive()) {

            // ── Input handling ────────────────────────────────────────────────
            if (gamepad1.dpad_up && !gamepad1.dpad_down) {
                currentPipeline = (currentPipeline + 1) % 3;
                limelight.pipelineSwitch(currentPipeline);
                sleep(200);
            } else if (gamepad1.dpad_down && !gamepad1.dpad_up) {
                currentPipeline = (currentPipeline + 2) % 3;
                limelight.pipelineSwitch(currentPipeline);
                sleep(200);
            }

            if (gamepad1.x && !lastX) {
                showRawPixels = !showRawPixels;
            }
            lastX = gamepad1.x;

            // ── Limelight status ──────────────────────────────────────────────
            LLStatus status = limelight.getStatus();
            LLResult result  = limelight.getLatestResult();

            telemetry.addData("Pipeline",  pipelineName(currentPipeline));
            telemetry.addData("FPS / Temp","%.1f fps  |  %.0f°C",
                    status.getFps(), status.getTemp());
            telemetry.addLine("─────────────────────────────────");

            if (result != null && result.isValid()) {

                // ── Color targets ─────────────────────────────────────────────
                List<LLResultTypes.ColorResult> targets = result.getColorResults();

                int reported = 0;
                for (int i = 0; i < targets.size(); i++) {
                    LLResultTypes.ColorResult t = targets.get(i);
                    if (t.getTargetArea() < MIN_TARGET_AREA) continue;

                    telemetry.addData(String.format("Target [%d]", reported),
                            pipelineName(currentPipeline));

                    // Degree-based coordinates (relative to crosshair)
                    telemetry.addData("  tx (horiz °)", "%.3f", t.getTargetXDegrees());
                    telemetry.addData("  ty (vert  °)", "%.3f", t.getTargetYDegrees());
                    telemetry.addData("  ta (area  %)", "%.3f", t.getTargetArea());

                    // Raw pixel center from bounding box corners
                    if (showRawPixels) {
                        List<List<Double>> corners = t.getTargetCorners();
                        if (corners != null && !corners.isEmpty()) {
                            double sumX = 0, sumY = 0;
                            for (List<Double> corner : corners) {
                                sumX += corner.get(0);
                                sumY += corner.get(1);
                            }
                            double centerX = sumX / corners.size();
                            double centerY = sumY / corners.size();
                            telemetry.addData("  center px (x)", "%.1f", centerX);
                            telemetry.addData("  center px (y)", "%.1f", centerY);
                        }
                    }

                    telemetry.addLine();
                    reported++;
                }

                if (reported == 0) {
                    telemetry.addData("Targets", "None above threshold (%.2f%%)", MIN_TARGET_AREA);
                } else {
                    telemetry.addData("Total detected", reported);
                }

                // ── Bot pose (one per frame, not per target) ──────────────────
                Pose3D pose = result.getBotpose();
                if (pose != null) {
                    telemetry.addLine("── Bot Pose ──");
                    telemetry.addData("  X (m)", "%.4f", pose.getPosition().x);
                    telemetry.addData("  Y (m)", "%.4f", pose.getPosition().y);
                    telemetry.addData("  Z (m)", "%.4f", pose.getPosition().z);
                }

            } else {
                telemetry.addData("Limelight result", "No valid result");
            }

            telemetry.addLine("─────────────────────────────────");
            telemetry.addData("Controls",
                    "dpad↑↓ pipeline  |  X=" + (showRawPixels ? "hide" : "show") + " pixels");
            telemetry.update();
        }

        limelight.stop();
    }

    private String pipelineName(int index) {
        switch (index) {
            case PIPELINE_YELLOW_SAMPLE: return "Yellow Sample";
            case PIPELINE_RED_SPECIMEN:  return "Red Specimen";
            case PIPELINE_BLUE_SPECIMEN: return "Blue Specimen";
            default:                     return "Pipeline " + index;
        }
    }
}