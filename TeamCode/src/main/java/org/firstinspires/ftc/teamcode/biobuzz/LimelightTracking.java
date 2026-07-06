package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import java.util.*;

@TeleOp(name = "Limelight Tracker", group = "LinearOpMode")
public class LimelightTracking extends LinearOpMode {
    private Limelight3A limelight;
    private final int FILTER_WINDOW_SIZE = 5;
    private final Queue<Double> xFilterQueue = new LinkedList<>();
    private double runningXSum = 0.0;
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

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double rawX = result.getTx();
                double rawY = result.getTy();
                double area = result.getTa();

                double smoothX = applyMovingAverageX(rawX);

                telemetry.addData("Target Status", "VISIBLE");
                telemetry.addData("Raw X Offset", "%.2f", rawX);
                telemetry.addData("Filtered X Offset", "%.2f", smoothX);
                telemetry.addData("Y Offset", "%.2f", rawY);
                telemetry.addData("Target Area", "%.2f%%", area);
            } else {
                telemetry.addData("Target Status", "NO TARGET DETECTED");
                xFilterQueue.clear();
                runningXSum = 0.0;
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
