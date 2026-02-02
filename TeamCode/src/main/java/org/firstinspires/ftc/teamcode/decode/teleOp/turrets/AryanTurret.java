package org.firstinspires.ftc.teamcode.decode.teleOp.turrets;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

@TeleOp(name = "Aryan Turret Face Red Corner", group = "Test")
public class AryanTurret extends LinearOpMode {

    private static final double TARGET_X = 130.0;
    private static final double TARGET_Y = 144.0;
    private static final int TURRET_MIN = -275;
    private static final int TURRET_MAX = 100;

    // Encoder mapping
    private static final double TICKS_PER_RAD = 275.0 / Math.PI;

    DcMotor turretMotor;
    Follower follower;

    public static final Pose START_POSE =
            new Pose(108, 130.2926713735558, Math.toRadians(180));

    @Override
    public void runOpMode() {

        // init ts
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0); // REQUIRED before RUN_TO_POSITION
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setPower(0.6);

        // pedro pedro pedro
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(START_POSE);

        waitForStart();

        while (opModeIsActive()) {


            follower.update();

            // --- LIVE pose ---
            Pose robotPose = follower.getPose();

            double robotX = robotPose.getX();
            double robotY = robotPose.getY();
            double robotHeading = robotPose.getHeading();

            // --- Angle to red corner ---
            double angleToTarget = Math.atan2(
                    TARGET_Y - robotY,
                    TARGET_X - robotX
            );

            // --- Turret relative angle ---
            double turretAngle = angleWrap(angleToTarget - robotHeading);

            // --- Encoder conversion ---
            int turretTarget = (int) Math.round(turretAngle * TICKS_PER_RAD);

            // --- Wiring safety clamp ---
            turretTarget = clamp(turretTarget, TURRET_MIN, TURRET_MAX);

            // --- Command turret ---
            turretMotor.setTargetPosition(turretTarget);

            // --- Telemetry ---
            telemetry.addData("Pose", "(%.1f, %.1f, %.1f°)",
                    robotX, robotY, Math.toDegrees(robotHeading));
            telemetry.addData("Turret Target", turretTarget);
            telemetry.addData("Turret Encoder", turretMotor.getCurrentPosition());
            telemetry.update();

            idle();
        }
    }

    // --- Helpers ---

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private int clamp(int val, int min, int max) {
        return Math.max(min, Math.min(max, val));
    }
}