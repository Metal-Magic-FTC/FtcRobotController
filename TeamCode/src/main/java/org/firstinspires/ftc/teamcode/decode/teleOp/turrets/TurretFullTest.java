package org.firstinspires.ftc.teamcode.decode.teleOp.turrets;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

@TeleOp(name = "TurretFullTest")
public class TurretFullTest extends LinearOpMode {

    /* ================= CONSTANTS ================= */
    private static final int TARGET_TAG_ID = 24;

    private static final int TURRET_MAX_TICKS = 555;
    private static final double TICKS_PER_RADIAN =
            TURRET_MAX_TICKS / (2.0 * Math.PI);

    private static final double METERS_TO_INCHES = 39.3701;

    /* ================= HARDWARE ================= */
    private DcMotor turretMotor;
    private Limelight3A limelight;
    private Follower follower;

    @Override
    public void runOpMode() {

        /* ===== Turret ===== */
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /* ===== Limelight ===== */
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3); // AprilTag pipeline
        limelight.setPollRateHz(100);
        limelight.start();

        /* ===== Pedro Pathing ===== */
        follower = Constants.createFollower(hardwareMap);

        // 🔴 Start odometry at (0,0,0)
        follower.setPose(new Pose(0, 0, Math.toRadians(-90)));

        telemetry.addLine("TurretFullTest READY");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            follower.update();

            // Auto-correct odometry if tag is visible
            boolean visionCorrected = correctOdometryFromTag();

            // Always aim turret at origin (0,0)
            updateTurretAim();

            Pose pose = follower.getPose();

            telemetry.addLine("=== ODOMETRY ===");
            telemetry.addData("X (in)", "%.2f", pose.getX());
            telemetry.addData("Y (in)", "%.2f", pose.getY());
            telemetry.addData("Heading (deg)",
                    "%.1f", Math.toDegrees(pose.getHeading()));

            telemetry.addLine("=== TURRET ===");
            telemetry.addData("Turret Ticks",
                    turretMotor.getCurrentPosition());
            telemetry.addData("Turret Angle (deg)",
                    "%.1f",
                    Math.toDegrees(
                            turretMotor.getCurrentPosition() / TICKS_PER_RADIAN
                    ));

            telemetry.addLine("=== VISION ===");
            telemetry.addData("Tag 24 Visible",
                    visionCorrected ? "YES (ODO CORRECTED)" : "NO");

            telemetry.update();
        }
    }

    /* ===================================================== */
    /* ========== TURRET AIM: FACE (0,10) ================== */
    /* ===================================================== */
    private void updateTurretAim() {

        Pose pose = follower.getPose();

        // Target point is (0, 10)
        double dx = 0.0 - pose.getX();
        double dy = 10.0 - pose.getY();

        double fieldAngle = Math.atan2(dy, dx);

        double turretAngle =
                normalizeRadians(fieldAngle - pose.getHeading());

        int targetTicks =
                (int) Math.round(turretAngle * TICKS_PER_RADIAN);

        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.5);
    }

    /* ===================================================== */
    /* ===== LIMELIGHT → ODOMETRY CORRECTION (TAG 24) ======= */
    /* ===================================================== */
    private boolean correctOdometryFromTag() {

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return false;

        for (LLResultTypes.FiducialResult tag :
                result.getFiducialResults()) {

            if (tag.getFiducialId() != TARGET_TAG_ID) continue;

            Pose3D robotToTag = tag.getRobotPoseTargetSpace();
            if (robotToTag == null) return false;

            // Robot relative to tag (camera frame)
            double relX = robotToTag.getPosition().x * METERS_TO_INCHES;
            double relY = robotToTag.getPosition().y * METERS_TO_INCHES;

            // Turret rotation in radians
            double turretYaw = turretMotor.getCurrentPosition() / TICKS_PER_RADIAN;

            // Camera yaw in radians
            double cameraYaw = Math.toRadians(robotToTag.getOrientation().getYaw());

            // True robot heading in field frame
            double robotHeading = normalizeRadians(cameraYaw - turretYaw + Math.toRadians(60));

            // Rotate relative position into field frame
            double cos = Math.cos(robotHeading);
            double sin = Math.sin(robotHeading);

            double fieldX = -(relX * cos - relY * sin);
            double fieldY = -(relX * sin + relY * cos);

            follower.setPose(new Pose(
                    fieldX,
                    fieldY,
                    robotHeading
            ));

            telemetry.addLine("✔ ODOMETRY CORRECTED FROM TAG 24");
            return true;
        }

        return false;
    }

    /* ================= HELPERS ================= */
    private double normalizeRadians(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }
}