package org.firstinspires.ftc.teamcode.decode.teleOp.turrets;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.teleOp.states.tests.limelightV2.FusedPose;

@TeleOp(name = "Aryan Turret Face Red Corner (Limelight + Odo)", group = "Test")
public class AryanTurretOdoLimelight extends LinearOpMode {

    private static final double TARGET_X = 160.0;
    private static final double TARGET_Y = 137.0;

    private static final int TURRET_MIN = -275;
    private static final int TURRET_MAX = 275;
    private static final double TICKS_PER_RAD = 275.0 / Math.PI;
    public DcMotorEx launchMotor;

    public double velocity = 2000;
    public double distance = 0;

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
        turretMotor.setPower(0.8);
        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setDirection(DcMotor.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(300.0, 0, 0, 12.9);
        launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

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
            distance = Math.pow(Math.pow(Math.abs(TARGET_Y - robotY),2)+Math.pow(Math.abs(TARGET_X - robotX),2), 0.5);

            if (gamepad1.yWasPressed()) {
                velocity+=100;
            }

            if (gamepad1.bWasPressed()) {
                velocity-=100;
            }
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