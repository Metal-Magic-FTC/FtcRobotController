package org.firstinspires.ftc.teamcode.decode.teleOp.turrets;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@TeleOp(name = "TurretFullTest")
public class TurretFullTest extends LinearOpMode {

    /* ================= CONSTANTS ================= */

    private static final int TARGET_TAG_ID = 24;

    private static final int TURRET_MAX_TICKS = 555;
    private static final double TICKS_PER_RADIAN =
            TURRET_MAX_TICKS / (2.0 * Math.PI);

    private static final double METERS_TO_INCHES = 39.3701;

    // ✅ Tag directly in front of robot, facing robot
    private static final Pose TAG_24_FIELD_POSE =
            new Pose(0, 24, Math.toRadians(270)); // example: 24 inches ahead

    /* ================= HARDWARE ================= */

    private DcMotor turretMotor;
    private Limelight3A limelight;
    private Follower follower;

    @Override
    public void runOpMode() {

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);
        limelight.setPollRateHz(100);
        limelight.start();

        follower = Constants.createFollower(hardwareMap);

        // ✅ Known start
        follower.setPose(new Pose(0, 0, Math.toRadians(90)));

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            correctOdometryFromTag();
            updateTurretAim();

            Pose p = follower.getPose();
            telemetry.addData("X", "%.2f", p.getX());
            telemetry.addData("Y", "%.2f", p.getY());
            telemetry.addData("H", "%.1f°", Math.toDegrees(p.getHeading()));
            telemetry.update();
        }
    }

    /* ===================================================== */
    /* ================= TURRET AIM ======================== */
    /* ===================================================== */

    private void updateTurretAim() {

        Pose pose = follower.getPose();

        double dx = -pose.getX();
        double dy = 10 - pose.getY();

        double fieldAngle = Math.atan2(dy, dx);
        double turretAngle =
                normalizeRadians(fieldAngle - pose.getHeading());

        int ticks = (int) Math.round(turretAngle * TICKS_PER_RADIAN);

        turretMotor.setTargetPosition(ticks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.5);
    }

    /* ===================================================== */
    /* ========== APRILTAG → FIELD ODOMETRY ================= */
    /* ===================================================== */

    private void correctOdometryFromTag() {

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {

            if (tag.getFiducialId() != TARGET_TAG_ID) continue;

            Pose3D robotInTag = tag.getRobotPoseTargetSpace();
            if (robotInTag == null) return;

            // Robot pose RELATIVE TO TAG (Limelight frame)
            double rx = robotInTag.getPosition().x * METERS_TO_INCHES;
            double ry = robotInTag.getPosition().y * METERS_TO_INCHES;
            double rYaw = Math.toRadians(robotInTag.getOrientation().getYaw());

            // 🔥 FIX LIMELIGHT TAG FRAME (180° flip)
            rx = -rx;
            ry = -ry;
            rYaw = normalizeRadians(rYaw + Math.PI);

            // Undo turret rotation
            double turretYaw =
                    turretMotor.getCurrentPosition() / TICKS_PER_RADIAN;

            double cosT = Math.cos(-turretYaw);
            double sinT = Math.sin(-turretYaw);

            double rxFixed = rx * cosT - ry * sinT;
            double ryFixed = rx * sinT + ry * cosT;

            // Tag → field
            double cosF = Math.cos(TAG_24_FIELD_POSE.getHeading());
            double sinF = Math.sin(TAG_24_FIELD_POSE.getHeading());

            double fieldX =
                    TAG_24_FIELD_POSE.getX() - (rxFixed * cosF - ryFixed * sinF);
            double fieldY =
                    TAG_24_FIELD_POSE.getY() - (rxFixed * sinF + ryFixed * cosF);

            double fieldHeading =
                    normalizeRadians(
                            TAG_24_FIELD_POSE.getHeading()
                                    + rYaw
                                    - turretYaw
                    );

            follower.setPose(new Pose(fieldX, fieldY, fieldHeading));
        }
    }

    private double normalizeRadians(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}