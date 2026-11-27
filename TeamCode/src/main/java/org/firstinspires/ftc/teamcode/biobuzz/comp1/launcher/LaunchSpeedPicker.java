package org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.v5.PathsV5;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher.advanced.Flywheel;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.RedHiveGoal;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.TurretController;
import org.firstinspires.ftc.teamcode.biobuzz.util.BulkReader;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * RED: simple launcher speed picker for collecting (distance, speed) data points for FlywheelSpeedTable.
 *
 * Same launcher controller (Flywheel kS/kV/kP), intake, transfer and ramp as BioBuzzTeleOp, but YOU pick the speed.
 * The turret auto-aims from odometry and telemetry shows the turret-to-cell distance. No Limelight.
 * Speeds are encoder ticks/s (the same units as the TeleOp presets and FlywheelSpeedTable).
 *
 * BEFORE INIT: robot on the Auto V5 start spot (29.0, 133.7) facing the audience (270 deg), turret facing FRONT.
 *
 * gamepad1
 *   sticks / right bumper  - drive / slow
 *   X                      - launcher on / off (on at START)
 *   dpad up / down         - speed +/- 25 ticks/s
 *   dpad right / left      - speed +/- 5 ticks/s (fine)
 *   left trigger (hold)    - SHOOT (transfer feeds up)
 *   right trigger          - intake on / off,  Y - flip intake direction
 *   left bumper (hold)     - flower ramp down
 *   A                      - RECORD a data point: "it scores at this distance with this speed"
 *   BACK                   - delete the last data point
 *   START                  - reset pose to the start spot (put the robot there first)
 *
 * Data points are listed on telemetry and saved to FIRST/launch_speed_points.csv on the Control Hub. Send them over
 * (distance + speed) and they go into the speed table.
 */
@TeleOp(name = "Launch Speed Picker (Red)", group = "BioBuzz Tests")
public class LaunchSpeedPicker extends LinearOpMode {

    private static final double START_TPS = 1550;
    private static final double BIG_STEP = 25, SMALL_STEP = 5;
    private static final double MAX_TPS = 2600;

    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_FEED_POWER = -0.8; // same as the TeleOp default
    private static final double TRANSFER_DOWN_POWER = 0.5;
    private static final double RAMP_UP = 0.730, RAMP_DOWN = 0.140;
    private static final double TRIGGER_DOWN = 0.5;
    private static final double SLOW_SCALE = 0.4;
    private static final String CSV_NAME = "launch_speed_points.csv";

    private final List<double[]> points = new ArrayList<>(); // {distance in, target tps, actual tps}
    private String status = "";

    @Override
    public void runOpMode() {
        DcMotor intake = hardwareMap.dcMotor.get("intakeMotor");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor transfer = hardwareMap.dcMotor.get("transferMotor");
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        Servo flowerServo = hardwareMap.get(Servo.class, "flowerServo"); // not moved until START

        Flywheel flywheel = new Flywheel();
        flywheel.init(hardwareMap);
        flywheel.stopMotors();
        TurretController turret = new TurretController(hardwareMap); // zeroes the turret: must face FRONT

        telemetry.addLine("Calibrating Pinpoint - keep the robot STILL");
        telemetry.update();
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(PathsV5.start);
        BulkReader bulk = new BulkReader(hardwareMap); // after Constants.create / flywheel.init
        RedHiveGoal goal = new RedHiveGoal(PathsV5.start.y());

        while (opModeInInit()) {
            bulk.update();
            follower.update();
            Pose p = follower.pose();
            telemetry.addLine("Robot on the Auto V5 start spot facing the audience? Turret facing FRONT?");
            telemetry.addData("Pose", "(%.1f, %.1f) %.1f°", p.x(), p.y(), Math.toDegrees(p.heading()));
            telemetry.update();
        }

        double targetTps = START_TPS;
        boolean launcherOn = true;
        boolean intakeOn = false, rightWasDown = false;
        double intakeDirection = 1;

        try {
            while (opModeIsActive()) {
                bulk.update();

                // ---- drive + pose ----
                double s = gamepad1.right_bumper ? SLOW_SCALE : 1.0;
                follower.manual(-gamepad1.left_stick_y * s, -gamepad1.left_stick_x * s, -gamepad1.right_stick_x * s);
                follower.update();
                if (gamepad1.startWasPressed()) {
                    follower.setPose(PathsV5.start);
                    turret.resetHeadingTracking();
                }
                Pose pose = follower.pose();
                turret.updateHeading(pose.heading());
                goal.update(pose.y());

                // ---- turret aims; distance measured from the TURRET (same as the TeleOp / speed table) ----
                turret.aimAt(pose.x(), pose.y(), pose.heading(), goal.x(), goal.y());
                double h = pose.heading();
                double tx = pose.x() + TurretConstants.TURRET_FORWARD_IN * Math.cos(h) - TurretConstants.TURRET_LEFT_IN * Math.sin(h);
                double ty = pose.y() + TurretConstants.TURRET_FORWARD_IN * Math.sin(h) + TurretConstants.TURRET_LEFT_IN * Math.cos(h);
                double distance = Math.hypot(goal.x() - tx, goal.y() - ty);

                // ---- speed ----
                if (gamepad1.dpadUpWasPressed()) targetTps += BIG_STEP;
                if (gamepad1.dpadDownWasPressed()) targetTps -= BIG_STEP;
                if (gamepad1.dpadRightWasPressed()) targetTps += SMALL_STEP;
                if (gamepad1.dpadLeftWasPressed()) targetTps -= SMALL_STEP;
                targetTps = Range.clip(targetTps, 0, MAX_TPS);
                if (gamepad1.xWasPressed()) launcherOn = !launcherOn;
                if (launcherOn) flywheel.setMotorRPM(Flywheel.ticksPerSecToRpm(targetTps)); // every loop
                else flywheel.stopMotors();
                double wheel = flywheel.getTicksPerSec();

                // ---- intake / transfer / ramp ----
                boolean rightDown = gamepad1.right_trigger > TRIGGER_DOWN;
                if (rightDown && !rightWasDown) intakeOn = !intakeOn;
                rightWasDown = rightDown;
                if (gamepad1.yWasPressed()) intakeDirection = -intakeDirection;
                intake.setPower(intakeOn ? intakeDirection * INTAKE_POWER : 0);
                boolean shooting = gamepad1.left_trigger > TRIGGER_DOWN;
                transfer.setPower(shooting ? TRANSFER_FEED_POWER : TRANSFER_DOWN_POWER);
                flowerServo.setPosition(gamepad1.left_bumper ? RAMP_DOWN : RAMP_UP);

                // ---- data points ----
                if (gamepad1.aWasPressed()) {
                    double[] pt = {distance, targetTps, wheel};
                    points.add(pt);
                    status = appendCsv(pt);
                }
                if (gamepad1.backWasPressed() && !points.isEmpty()) {
                    points.remove(points.size() - 1);
                    status = "Deleted the last point here (the CSV keeps every line)";
                }

                // ---- telemetry ----
                telemetry.addData("SPEED", "%.0f ticks/s target   (actual %.0f)   launcher %s",
                        targetTps, wheel, launcherOn ? "ON" : "OFF (X)");
                telemetry.addData("DISTANCE", "%.1f in to the %s cell", distance, goal.aimingTop() ? "TOP" : "BOTTOM");
                telemetry.addData("Speed table says", "%.0f ticks/s at this distance",
                        FlywheelSpeedTable.velocityFor(distance));
                telemetry.addData("Pose", "(%.1f, %.1f) %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
                telemetry.addLine("dpad U/D ±25 | dpad R/L ±5 | LT shoot | RT intake | A record | BACK undo");
                if (!status.isEmpty()) telemetry.addLine(status);
                telemetry.addLine("--- DATA POINTS (distance in -> speed ticks/s) ---");
                for (int i = 0; i < points.size(); i++) {
                    double[] pt = points.get(i);
                    telemetry.addLine(String.format("%d.  %5.1f in  ->  %4.0f   (actual %4.0f)", i + 1, pt[0], pt[1], pt[2]));
                }
                telemetry.update();
            }
        } finally {
            flywheel.stopMotors();
            intake.setPower(0);
            transfer.setPower(0);
            turret.stop();
        }
    }

    /** Appends one point to FIRST/launch_speed_points.csv on the Control Hub (kept between runs). */
    private String appendCsv(double[] pt) {
        File file = new File(AppUtil.FIRST_FOLDER, CSV_NAME);
        boolean isNew = !file.exists();
        try (FileWriter w = new FileWriter(file, true)) {
            if (isNew) w.write("distance_in,target_tps,actual_tps\n");
            w.write(String.format("%.1f,%.0f,%.0f\n", pt[0], pt[1], pt[2]));
            return String.format("Recorded %.1f in -> %.0f (saved to FIRST/%s)", pt[0], pt[1], CSV_NAME);
        } catch (IOException e) {
            return "Recorded here, but the CSV save failed: " + e.getMessage();
        }
    }
}
