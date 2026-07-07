package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import java.util.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Limelight Tracker", group = "LinearOpMode")
public class LimelightTracking extends LinearOpMode {
    private Limelight3A limelight;
    private final int FILTER_WINDOW_SIZE = 5;
    private final Queue<Double> xFilterQueue = new LinkedList<>();
    private double runningXSum = 0.0;
    private ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0.0;
    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.addData("Status", "Limelight initializing...");
        telemetry.update();
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry.addData("Status", "Ready to start!");
        telemetry.update();
        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            double currentTime = timer.seconds();
            double loopTimeSeconds = currentTime - lastTime;
            lastTime = currentTime;
            double loopHz = 1.0 / loopTimeSeconds;
            LLResult result = limelight.getLatestResult();
            telemetry.addData("Loop Time (Hertz)", loopHz);

            if (result != null && result.isValid()) {
                // 1. Retrieve the array of all separate tracked contours/objects
                List<LLResultTypes.ColorResult> targets = result.getColorResults();
                int totalTargets = (targets != null) ? targets.size() : 0;
                if (targets != null && !targets.isEmpty()) {
                    telemetry.addData("Total Targets Found", totalTargets);

                    // 2. Loop through every visible target found in the current frame
                    for (int i = 0; i < targets.size(); i++) {
                        LLResultTypes.ColorResult target = targets.get(i);

                        // 3. Extract individual target parameters
                        String className = String.valueOf(target.getClass()); // The custom class/color label
                        double tx = target.getTargetXDegrees(); // Horizontal offset
                        double ty = target.getTargetYDegrees(); // Vertical offset
                        double ta = target.getTargetArea(); // Target area %

                        // Output data for each distinct target
                        telemetry.addLine("--- Target #" + i + " [" + className + "] ---");
                        telemetry.addData(" X Offset", "%.2f°", tx);
                        telemetry.addData(" Y Offset", "%.2f°", ty);
                        telemetry.addData(" Area", "%.2f%%", ta);
                    }
                } else {
                    telemetry.addData("Status", "Valid result, but 0 contours parsed.");
                }
            } else {
                telemetry.addData("Status", "No overall target visible.");
            }
            telemetry.update();

        }

        limelight.stop();
    }

    private double applyMovingAverageX(double newXValue) {
        xFilterQueue.add(newXValue);
        runningXSum += newXValue;

        if (xFilterQueue.size() > FILTER_WINDOW_SIZE) {
            runningXSum -= xFilterQueue.remove();
        }

        return runningXSum / xFilterQueue.size();
    }
}
