package org.firstinspires.ftc.teamcode.decode.teleOp.turrets;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

@Autonomous(name = "!Turret Odometry Track Test", group = "Test")
public class TurretOdometryTrack extends LinearOpMode {

    // --------------------
    // CONSTANTS
    // --------------------
    private static final double START_X = 144 - 116.6988847583643;
    private static final double START_Y = 128.83271375464685;
    private static final double START_HEADING =
            Math.toRadians(180 - 225);

    private static final int TURRET_MIN = 0+278;
    private static final int TURRET_MAX = 555+278;
    private static final double TICKS_PER_RADIAN =
            TURRET_MAX / (2 * Math.PI);

    // --------------------
    // HARDWARE
    // --------------------
    private DcMotor turretMotor;
    private Follower follower;

    // --------------------
    // START POSE
    // --------------------
    private Pose startPose = new Pose(
            START_X,
            START_Y,
            START_HEADING
    );

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        telemetry.addLine("Turret Odometry Track Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            follower.update();

            updateTurretAim();

            telemetry.addData("Robot X", follower.getPose().getX());
            telemetry.addData("Robot Y", follower.getPose().getY());
            telemetry.addData("Robot Heading (deg)",
                    Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Turret Encoder",
                    turretMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    // --------------------
    // TURRET AIM LOGIC
    // --------------------
    private void updateTurretAim() {

        Pose robotPose = follower.getPose();

        double dx = START_X - robotPose.getX();
        double dy = START_Y - robotPose.getY();

        double fieldAngleToStart = Math.atan2(dy, dx);

        double turretAngle =
                normalizeRadians(fieldAngleToStart - robotPose.getHeading());

        int targetTicks =
                (int) Math.round(turretAngle * TICKS_PER_RADIAN);

        targetTicks = clamp(targetTicks, TURRET_MIN, TURRET_MAX);

        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.4);
    }

    // --------------------
    // HELPERS
    // --------------------
    private double normalizeRadians(double angle) {
        while (angle < 0) angle += 2 * Math.PI;
        while (angle >= 2 * Math.PI) angle -= 2 * Math.PI;
        return angle;
    }

    private int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }

    // --------------------
    // INIT
    // --------------------
    private void initHardware() {

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}