package org.firstinspires.ftc.teamcode.decode.teleOp.align.turret;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.GoBildaPinpointDriver;

import java.util.List;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "!!!Auto Aim Turret")
public class AutoTurret extends OpMode {

    private Limelight3A limelight;
    private GoBildaPinpointDriver odometry;
    //turret code at line 182
    private DcMotor turretMotor;

    private boolean usePinpointIMU = true;
    private double manualHeading = 0.0;
    private double limeHeadStore = 0.0;
    private double robotHeadingStore = 0.0;
    private double currentTx = 0.0;
    private boolean showAbsoluteHeading = false; // Show MT2 heading without updateRobotOrientation


    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(50);

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        limelight.pipelineSwitch(3); // AprilTag pipeline
        limelight.setPollRateHz(100);

        // Try to initialize GoBilda Pinpoint Odometry
        try {
            odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

            // Configure odometry - Pinpoint at center of robot with swingarm pods
            odometry.setOffsets(0.0, 0.0); // Center of robot
            odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
            odometry.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );
            odometry.resetPosAndIMU();

            telemetry.addData("Pinpoint IMU", "Available (press A to enable)");
            telemetry.addData("Odometry Config", "Center mount, swingarm pods");
        } catch (Exception e) {
            odometry = null;
            telemetry.addData("Pinpoint IMU", "Not available");
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("", "Point camera at AprilTag");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        // Update odometry if available
        if (odometry != null) {
            odometry.update();
        }


        // Get current heading
        double currentHeading = getCurrentHeading();

        // Get latest result
        LLResult result;

        if (showAbsoluteHeading) {
            // Special mode: Get MegaTag2 WITHOUT calling updateRobotOrientation
            // This shows the ABSOLUTE field heading that MegaTag2 detects
            result = limelight.getLatestResult();
        } else {
            // Normal mode: Update robot orientation BEFORE getting results
            limelight.updateRobotOrientation(currentHeading);
            result = limelight.getLatestResult();
        }

        if (result == null || !result.isValid()) {
            displayNoData(currentHeading);
            return;
        }

        displayDiagnostics(result, currentHeading);
    }


    private double getCurrentHeading() {
        if (usePinpointIMU && odometry != null) {
            return Math.toDegrees(odometry.getHeading());
        }
        return manualHeading;
    }

    private void displayNoData(double currentHeading) {
        telemetry.addLine("═══ ROBOT HEADING ═══");
        telemetry.addData("Source", usePinpointIMU ? "Pinpoint IMU" : "Manual");
        telemetry.addData("Heading", "%.1f°", currentHeading);
        telemetry.addLine();
        telemetry.addData("STATUS", "❌ NO APRILTAG DETECTED");
        telemetry.addLine();
        telemetry.addLine("Point camera at an AprilTag to see data");
        telemetry.addLine();
        telemetry.update();
    }

    private void displayDiagnostics(LLResult result, double currentHeading) {
        telemetry.addLine("═══ ROBOT HEADING ═══");
        telemetry.addData("Source", usePinpointIMU ? "Pinpoint IMU" : "Manual");
        telemetry.addData("Heading", "%.1f°", currentHeading);

        // Show absolute heading mode status
        if (showAbsoluteHeading) {
            telemetry.addData("⚠ MODE", "Absolute MT2 (no updateRobotOrientation)");
        } else {
            telemetry.addData("MODE", "Normal (with updateRobotOrientation)");
        }

        // Show odometry position if available
        if (odometry != null) {
            telemetry.addData("Odo X", "%.1f mm", odometry.getPosition().getX(DistanceUnit.MM));
            telemetry.addData("Odo Y", "%.1f mm", odometry.getPosition().getY(DistanceUnit.MM));
        }


        telemetry.addLine();
        telemetry.addLine("═══ INDIVIDUAL TAG DATA ═══");
        displayTagData(result);

        telemetry.update();
    }


    private void displayTagData(LLResult result) {
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        if (fiducials == null || fiducials.isEmpty()) {
            telemetry.addData("Tags Visible", "0");
            return;
        }

        telemetry.addData("Tags Visible", fiducials.size());

        // Show first tag in detail
        LLResultTypes.FiducialResult tag = fiducials.get(0);
        telemetry.addData("Tag ID", tag.getFiducialId());

        // Robot pose relative to tag (robot-centric, NOT field-centric)
        Pose3D robotTarget = tag.getRobotPoseTargetSpace();
        if (robotTarget != null) {
            limeHeadStore = result.getTx();
            robotHeadingStore = getCurrentHeading();
            telemetry.addLine("Robot→Tag (relative):");
            telemetry.addData("  X", "%.3f m (forward)", robotTarget.getPosition().x);
            telemetry.addData("  Y", "%.3f m (left)", robotTarget.getPosition().y);
            telemetry.addData("  Z", "%.3f m (distance)", robotTarget.getPosition().z);
            telemetry.addData("Pitch: ", tag.getTargetYDegrees());
            telemetry.addData("Yaw: ", tag.getTargetXDegrees());

            //interval (-0.01, 0.01) is room of error for turret rotation, increase if needed
            if (result.getTx() > 0.01) {
                //rotate turret to the right
                turretMotor.setPower(0.1);
                //substitute 280 with amount of ticks the turret takes to rotate 360 degrees
                turretMotor.setTargetPosition((int) (280*(result.getTx()/360)));

            }
            else if (result.getTx() < -0.01) {
                //rotate turret to the left
                turretMotor.setPower(-0.1);
                //substitute 280 with amount of ticks the turret takes to rotate 360 degrees
                turretMotor.setTargetPosition((int) (280-(280*(Math.abs(result.getTx())/360))));

            } else {
                //set turret power to 0
                turretMotor.setPower(0);
            }
            // Check if this is being used incorrectly as field coordinates
            if (robotTarget.getPosition().z < 3.0) {
                telemetry.addData("⚠ NOTE", "Z decreases as you approach tag");
                telemetry.addData("", "This is TAG-RELATIVE, not field!");
            }
        }

        //auto aiming without tag (doesn't work)
        else {

//            currentTx = limeHeadStore-(getCurrentHeading()-robotHeadingStore);
//            //interval (-0.01, 0.01) is room of error for turret rotation, increase if needed
//            if (currentTx > 0.01) {
//                //rotate turret to the right
//                turretMotor.setPower(0.1);
//                //substitute 280 with amount of ticks the turret takes to rotate 360 degrees
//                turretMotor.setTargetPosition((int) (280*(currentTx/360)));
//
//            }
//            else if (currentTx < -0.01) {
//                //rotate turret to the left
//                turretMotor.setPower(-0.1);
//                //substitute 280 with amount of ticks the turret takes to rotate 360 degrees
//                turretMotor.setTargetPosition((int) (280-(280*(Math.abs(currentTx)/360))));
//
//            } else {
//                //set turret power to 0
//                turretMotor.setPower(0);
//            }
        }

    }

    /**
     * Initialize Pinpoint odometry with absolute heading from MegaTag2.
     * This demonstrates the fix for the "odometry reset" issue.
     */

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}

