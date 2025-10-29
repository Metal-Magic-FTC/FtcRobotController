package org.firstinspires.ftc.teamcode.limeLight;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;

import java.util.List;

@TeleOp(name = "Sensor: Limelight AprilTag Telemetry", group = "Sensor")
public class LimelightAprilTagTelemetry2 extends OpMode {

    private Limelight3A limelight;
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;

    private LLResult latestResult = null;
    private LLResultTypes.FiducialResult lastValidTag = null;

    private int currentPipeline = -1;

    private static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 0;  // adjust for your robot
    private static final double LIMELIGHT_LENS_HEIGHT_INCHES = 4;
    private static final double TARGET_HEIGHT_INCHES = 1.17; // make this realistic!

    @Override
    public void init() {
        telemetry.addLine("Initializing Limelight...");
        telemetry.update();

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.start();
            limelight.setPollRateHz(50);
        } catch (Exception e) {
            telemetry.addLine("❌ Limelight not found in config!");
            telemetry.update();
            return;
        }

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBackDrive.setDirection(UtilityValues.compLeftBackDirection);
        leftFrontDrive.setDirection(UtilityValues.compLeftFrontDirection);
        rightBackDrive.setDirection(UtilityValues.compRightBackDirection);
        rightFrontDrive.setDirection(UtilityValues.compRightFrontDirection);

        switchPipeline(3); // Default AprilTag pipeline
        telemetry.addLine("✅ Limelight Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (limelight == null) {
            telemetry.addLine("⚠️ Limelight not initialized!");
            telemetry.update();
            return;
        }

        handlePipelineSwitching();

        latestResult = limelight.getLatestResult();

        if (latestResult != null && latestResult.isValid()) {
            processAprilTags(latestResult);
        } else {
            telemetry.addLine("No valid Limelight result");
        }

        telemetry.update();
    }

    /** Handles pipeline switching (with debounce handled naturally by manual presses) */
    private void handlePipelineSwitching() {
        if (gamepad1.x) {
            switchPipeline(0); // Blue
        } else if (gamepad1.y) {
            switchPipeline(1); // Green
        } else if (gamepad1.b) {
            switchPipeline(5); // White
        } else if (gamepad1.a) {
            switchPipeline(3); // AprilTag
        }
    }

    private void switchPipeline(int pipeline) {
        if (pipeline != currentPipeline && limelight != null) {
            currentPipeline = pipeline;
            limelight.pipelineSwitch(pipeline);
            telemetry.addData("Switched Pipeline", pipeline);
        }
    }

    private void processAprilTags(LLResult result) {
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        if (fiducials == null || fiducials.isEmpty()) {
            telemetry.addLine("No AprilTags Detected");
            return;
        }

        LLResultTypes.FiducialResult tag = fiducials.get(0); // take first tag

        if (tag != null) {
            lastValidTag = tag;

            int id = tag.getFiducialId();
            double xDeg = tag.getTargetXDegrees();
            double yDeg = tag.getTargetYDegrees();
            double distance = calculateDistance(yDeg);

            telemetry.addData("AprilTag ID", id);
            telemetry.addData("X Offset (deg)", "%.2f", xDeg);
            telemetry.addData("Y Offset (deg)", "%.2f", yDeg);
            telemetry.addData("Est. Distance (in)", "%.2f", distance);

            if (tag.getRobotPoseTargetSpace() != null) {
                double zMeters = tag.getRobotPoseTargetSpace().getPosition().z;
                telemetry.addData("Pose Z (m)", "%.3f", zMeters);
            }

            if (id == 21 || id == 22 || id == 23) {
                telemetry.addLine("Tracking special AprilTag ID: " + id);
            }
        } else if (lastValidTag != null) {
            telemetry.addLine("Using last known AprilTag data");
            telemetry.addData("Last Tag ID", lastValidTag.getFiducialId());
        }
    }

    private double calculateDistance(double targetYOffsetDegrees) {
        double targetAngle = LIMELIGHT_MOUNT_ANGLE_DEGREES + targetYOffsetDegrees;
        double targetAngleRadians = Math.toRadians(targetAngle);
        double deltaHeight = TARGET_HEIGHT_INCHES - LIMELIGHT_LENS_HEIGHT_INCHES;
        return Math.abs(deltaHeight / Math.tan(targetAngleRadians));
    }
}