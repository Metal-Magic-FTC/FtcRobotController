package org.firstinspires.ftc.teamcode.decode.teleOp.turrets;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.teleOp.states.tests.limelightV2.FusedPose;

@TeleOp(name = "Aryan Turret Face Red Corner (Limelight + Odo)", group = "Test")
public class AryanTurretOdoLimelight extends LinearOpMode {

    private static final double TARGET_X = 130.0;
    private static final double TARGET_Y = 144.0;

    private static final int TURRET_MIN = -275;
    private static final int TURRET_MAX = 100;
    private static final double TICKS_PER_RAD = 275.0 / Math.PI;

    DcMotor turretMotor;
    Follower follower;
    FusedPose fusedPose;

    public static final Pose START_POSE =
            new Pose(108, 130.29, Math.toRadians(180));

    @Override
    public void runOpMode() {

        // ===== Turret =====
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setPower(0.6);

        // ===== Pedro =====
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(START_POSE);

        // ===== Limelight Fusion =====
        fusedPose = new FusedPose(hardwareMap, START_POSE);

        waitForStart();

        while (opModeIsActive()) {

            // Update fusion + odometry
            fusedPose.update();
            follower.update();

            // === Inject Limelight pose into Pedro ===
            Pose limelightPose = fusedPose.getRobotPose(true); // CONVERTED pose
            if (limelightPose != null) {
                follower.setPose(limelightPose);
            }

            // === Current robot pose ===
            Pose robotPose = follower.getPose();
            double robotX = robotPose.getX();
            double robotY = robotPose.getY();
            double robotHeading = robotPose.getHeading();

            // === Turret math ===
            double angleToTarget = Math.atan2(
                    TARGET_Y - robotY,
                    TARGET_X - robotX
            );

            double turretAngle = angleWrap(angleToTarget - robotHeading);
            int turretTarget = (int) Math.round(turretAngle * TICKS_PER_RAD);
            turretTarget = clamp(turretTarget, TURRET_MIN, TURRET_MAX);

            turretMotor.setTargetPosition(turretTarget);

            // === Telemetry ===
            telemetry.addData("Pose X", "%.1f", robotX);
            telemetry.addData("Pose Y", "%.1f", robotY);
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(robotHeading));
            telemetry.addData("Turret Target", turretTarget);
            telemetry.update();

            idle();
        }
    }

    // ===== Helpers =====

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private int clamp(int val, int min, int max) {
        return Math.max(min, Math.min(max, val));
    }
}