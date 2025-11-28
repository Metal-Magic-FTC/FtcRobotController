package org.firstinspires.ftc.teamcode.limeLight;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;

import java.util.List;

@TeleOp(name = "Sensor: LimeLightAprilTagData", group = "Sensor")
public class LimeLlightAprilTagTelemetry extends OpMode {

    private Limelight3A limelight;

    // Latest AprilTag detection data
    private LLResult latestResult = null;
    private LLResultTypes.FiducialResult lastValidTag = null;

    // For stable pipeline switching
    private int currentPipeline = -1;

    // Limelight physical constants
    private static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 0; // tilt of LL from vertical
    private static final double LIMELIGHT_LENS_HEIGHT_INCHES = 4;  // lens height from floor
    private static final double TARGET_HEIGHT_INCHES = 1.17;       // target height from floor

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Poll Limelight quickly for smooth updates
        limelight.setPollRateHz(50);
        limelight.start();

        switchPipeline(0); // Start with default pipeline
    }

    @Override
    public void loop() {
        handlePipelineSwitching();

        latestResult = limelight.getLatestResult();

        if (latestResult != null && latestResult.isValid()) {
            processAprilTags(latestResult);
        } else {
            telemetry.addLine("No Limelight Data");
        }

        telemetry.update();
    }

    /**
     * Handles switching pipelines via gamepad buttons with debounce.
     */
    private void handlePipelineSwitching() {
        if (gamepad1.x) {
            switchPipeline(0); // Purple/Blue detection
        } else if (gamepad1.y) {
            switchPipeline(1); // Green detection
        } else if (gamepad1.b) {
            switchPipeline(5); // White detection
        } else if (gamepad1.a) {
            switchPipeline(3); // AprilTag pipeline
        }
    }

    /**
     * Switch pipelines only when needed (prevents spam switching).
     */
    private void switchPipeline(int pipeline) {
        if (pipeline != currentPipeline) {
            currentPipeline = pipeline;
            limelight.pipelineSwitch(pipeline);
        }
    }

    /**
     * Processes AprilTag fiducial data from the latest Limelight result.
     */
    private void processAprilTags(LLResult result) {
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        if (fiducials == null || fiducials.isEmpty()) {
            telemetry.addLine("No AprilTags Detected");
            return;
        }

        // Use the closest or first detected tag
        LLResultTypes.FiducialResult bestTag = fiducials.get(0);

        if (bestTag != null) {
            lastValidTag = bestTag; // store latest stable detection
            int id = bestTag.getFiducialId();
            double xDegrees = bestTag.getTargetXDegrees();
            double yDegrees = bestTag.getTargetYDegrees();

            // Distance estimation
            double distanceInches = calculateDistance(yDegrees);

            telemetry.addData("AprilTag ID", id);
            telemetry.addData("X Offset (deg)", xDegrees);
            telemetry.addData("Y Offset (deg)", yDegrees);
            telemetry.addData("Estimated Distance (in)", distanceInches);

            // Pose data
            double zMeters = bestTag.getRobotPoseTargetSpace().getPosition().z;
            telemetry.addData("3D Pose Z (meters)", zMeters);

            // If the tag is one of interest
            if (id == 21 || id == 22 || id == 23) {
                telemetry.addLine("Tracking special AprilTag: " + id);
            }
        } else if (lastValidTag != null) {
            telemetry.addLine("Using last known AprilTag data");
            telemetry.addData("Last Tag ID", lastValidTag.getFiducialId());
        }
    }

    /**
     * Calculates distance from Limelight to target in inches.
     */
    private double calculateDistance(double targetYOffsetDegrees) {
        double targetAngle = LIMELIGHT_MOUNT_ANGLE_DEGREES + Math.abs(targetYOffsetDegrees);
        double targetAngleRadians = Math.toRadians(targetAngle);

        return Math.abs(TARGET_HEIGHT_INCHES - LIMELIGHT_LENS_HEIGHT_INCHES) / Math.tan(targetAngleRadians);
    }
}