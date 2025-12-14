package org.firstinspires.ftc.teamcode.decode.teleOp.turrets;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

@TeleOp(name = "🔥 Turret Full Test (Odo + Lime)")
public class TurretFullTest extends LinearOpMode {

    private static final double START_X = 116.7;
    private static final double START_Y = 128.8;
    private static final double START_HEADING = Math.toRadians(225);

    private static final int TURRET_MIN = 0;
    private static final int TURRET_MAX = 555;
    private static final double TICKS_PER_RADIAN =
            TURRET_MAX / (2 * Math.PI);

    private static final int CORRECTION_TAG_ID = 24;
    private static final double METERS_TO_INCHES = 39.3701;

    private static final double POS_CORRECTION_THRESHOLD = 2.0; // inches
    private static final double HEADING_CORRECTION_THRESHOLD = Math.toRadians(3);
    private DcMotor turretMotor;
    private Limelight3A limelight;
    private Follower follower;

    private final Pose startPose = new Pose(
            START_X,
            START_Y,
            START_HEADING
    );

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        telemetry.addLine("Turret Full Test Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            follower.update();

            correctOdometryWithLimelight();

            updateTurretAim();

            Pose p = follower.getPose();
            telemetry.addData("X", "%.2f", p.getX());
            telemetry.addData("Y", "%.2f", p.getY());
            telemetry.addData("Heading", "%.1f°",
                    Math.toDegrees(p.getHeading()));
            telemetry.addData("Turret Encoder",
                    turretMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    private void updateTurretAim() {

        Pose robotPose = follower.getPose();

        double dx = 136 - robotPose.getX();
        double dy = 136 - robotPose.getY();

        double fieldAngle = Math.atan2(dy, dx);

        double turretAngle =
                normalizeRadians(fieldAngle - robotPose.getHeading());

        int targetTicks =
                (int) Math.round(turretAngle * TICKS_PER_RADIAN);

        targetTicks = clamp(targetTicks, TURRET_MIN, TURRET_MAX);

        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.6);
    }

    private void correctOdometryWithLimelight() {

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        for (LLResultTypes.FiducialResult tag :
                result.getFiducialResults()) {

            if (tag.getFiducialId() != CORRECTION_TAG_ID) continue;

            Pose3D robotFieldPose =
                    tag.getRobotPoseFieldSpace();

            if (robotFieldPose == null) return;

            double visionX =
                    robotFieldPose.getPosition().x * METERS_TO_INCHES;
            double visionY =
                    robotFieldPose.getPosition().y * METERS_TO_INCHES;
            double visionHeading =
                    Math.toRadians(
                            robotFieldPose.getOrientation().getYaw());

            Pose current = follower.getPose();

            double dx = visionX - current.getX();
            double dy = visionY - current.getY();
            double dh = normalizeRadians(
                    visionHeading - current.getHeading());

            double posError = Math.hypot(dx, dy);

            if (posError > POS_CORRECTION_THRESHOLD ||
                    Math.abs(dh) > HEADING_CORRECTION_THRESHOLD) {

                follower.setPose(new Pose(
                        visionX,
                        visionY,
                        visionHeading
                ));
            }
            return;
        }
    }

    private double normalizeRadians(double angle) {
        while (angle < -Math.PI) angle += 2 * Math.PI;
        while (angle > Math.PI) angle -= 2 * Math.PI;
        return angle;
    }

    private int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }

    /* ===================== INIT ===================== */
    private void initHardware() {

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3); // AprilTag pipeline
        limelight.setPollRateHz(100);
        limelight.start();
    }
}