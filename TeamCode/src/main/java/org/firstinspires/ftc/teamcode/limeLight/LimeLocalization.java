package org.firstinspires.ftc.teamcode.limeLight;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;

import java.util.List;

@TeleOp(name = "!limeLocalization")
public class LimeLocalization extends OpMode {

    Limelight3A limelight;
    double tx;
    double ty;
    double ta;
    int pipeline;
    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;

    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "backLeft");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "backRight");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBackDrive.setDirection(UtilityValues.compLeftBackDirection);
        leftFrontDrive.setDirection(UtilityValues.compLeftFrontDirection);
        rightBackDrive.setDirection(UtilityValues.compRightBackDirection);
        rightFrontDrive.setDirection(UtilityValues.compRightFrontDirection);
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();
            double targetOffsetAngle_Vertical = Math.abs(ty);

            // how many degrees back is your limelight rotated from perfectly vertical?
            double limelightMountAngleDegrees = 0;

            // distance from the center of the Limelight lens to the floor
            double limelightLensHeightInches = 4;

            // distance from the target to the floor
            double goalHeightInches = 1.17;

            double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

            //calculate distance
            double distanceFromLimelightToGoalInches = (Math.abs(goalHeightInches - limelightLensHeightInches)) / Math.tan(angleToGoalRadians);
        }

        //if (gamepad1.a) {
            // how many degrees back is your limelight rotated from perfectly vertical?
            double limelightMountAngleDegrees = 0;

            // distance from the center of the Limelight lens to the floor
            double targetOffsetAngle_Vertical = Math.abs(ty);
            double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
            //april tag id detection and localization
            limelight.pipelineSwitch(3);
            pipeline = 3;
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials.isEmpty()) {
                telemetry.addLine("No fiducials found");
            }
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId(); // The ID number of the fiducial
                double x = fiducial.getTargetXDegrees();
                double y = fiducial.getTargetYDegrees();
                double StrafeDistance_3D = fiducial.getRobotPoseTargetSpace().getOrientation().getPitch();

//                    telemetry.addData("Target X", tx);
//                    telemetry.addData("Target Y", ty);
//                    telemetry.addData("Target Area", ta);
//                    telemetry.addData("Angle: ", angleToGoalDegrees);
//                    telemetry.addData("Fiducial " + id, "is " + fiducial.getRobotPoseTargetSpace().getPosition().z, " meters away");
                // First, tell Limelight which way your robot is facing
                double robotYaw = 135; //imu yaw
                limelight.updateRobotOrientation(robotYaw);
                if (result != null && result.isValid()) {
                    Pose3D botpose_mt2 = result.getBotpose_MT2();
                    if (botpose_mt2 != null) {
                        double x2 = botpose_mt2.getPosition().x;
                        double y2 = botpose_mt2.getPosition().y;
                        double yaw = botpose_mt2.getOrientation().getYaw();
                        telemetry.addData("MT2 Location:", "(" + x2 + ", " + y2 + ")");
                        telemetry.addData("Yaw: ", + yaw);
                    }
                }
            }
       // }

        telemetry.update();
    }
}
