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
import org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher.advanced.Flywheel;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.AxonTurretAimOdo;
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
 * RED: tunes the launcher speed vs. distance for the turret launcher (ball exits at EXIT_ANGLE_DEG = 70 deg).
 *
 * For each recorded shot it works out the ball speed physics says is needed at that distance (no drag) and the
 * RATIO  launcher ticks/s  /  required ball speed (in/s). Real shots need a bit more than physics as the distance
 * grows (drag), so the ratio is fitted as a straight line vs. distance. With that fit it predicts the speed for any
 * distance and prints ready-to-paste rows for FlywheelSpeedTable.
 *
 * Speeds are encoder TICKS PER SECOND (same units as FlywheelSpeedTable and the 1650 wall shot).
 *
 * SPEED DIP: whenever the wheel is more than DIP_TPS (75) under the target it runs at FULL power to recover, and while
 * shooting the transfer slows to TRANSFER_SLOW_FRACTION of its feed power, so the next ball waits for the wheel.
 * (The Flywheel class's own dip boost is switched off while this OpMode runs and put back afterwards.)
 *
 * BEFORE INIT: robot on the Auto V4 start spot (29.0, 133.7) facing the audience, still; turret facing the FRONT.
 *
 * gamepad1
 *   sticks / right bumper     - drive / slow
 *   left trigger  (hold)      - SHOOT: transfer feeds up (slowed during a speed dip)
 *   right trigger             - intake on / off          Y - flip intake direction
 *   left bumper   (hold)      - flower ramp down
 *   dpad up / down            - target speed +/- 25 ticks/s (starts at 1700)
 *   X                         - flywheel on / off
 *   A                         - RECORD: "that shot scored at this distance and speed"
 *   BACK                      - delete the last record
 *   B                         - set the target to the fitted prediction for this distance (needs 1+ records)
 *   START                     - reset pose to the start spot (put the robot there first)
 *
 * HOW TO TUNE
 *   1. Drive to a distance (telemetry "Distance"), stop, X to spin up, shoot a few balls.
 *   2. dpad up/down until the balls score reliably in the middle of the cell, then A to record.
 *   3. Repeat at 4-6 distances across the range you shoot from (close, mid, far).
 *   4. Copy the "Table rows" into FlywheelSpeedTable.VELOCITY_TPS. Records are also saved on the Control Hub in
 *      FIRST/shot_distance_tuning.csv.
 */
@TeleOp(name = "Shot Distance Tuner (Red)", group = "BioBuzz Tests")
public class ShotDistanceTuner extends LinearOpMode {

    // ======================= SHOT GEOMETRY (MEASURE) =======================
    /** Ball exit angle above horizontal. */
    public static final double EXIT_ANGLE_DEG = 70.0;
    /** Height the ball has to climb from where it leaves the launcher to the cell opening, inches. MEASURE. */
    public static final double RISE_IN = 42.0;
    private static final double G_IN_PER_S2 = 386.09;

    // ======================= SPEED =======================
    private static final double START_TPS = 1700;
    private static final double STEP_TPS = 25;
    private static final double MAX_TPS = 2600;
    /** Below target - DIP_TPS: full power + slow transfer. */
    private static final double DIP_TPS = 75;
    /** Back to the normal controller once within this of the target (keeps it from flickering at the edge). */
    private static final double DIP_EXIT_TPS = 15;

    // ======================= MECHANISMS (same as BioBuzzTeleOp) =======================
    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_FEED_POWER = -0.7;
    private static final double TRANSFER_DOWN_POWER = 0.5;
    /** Transfer power during a speed dip, as a fraction of TRANSFER_FEED_POWER. */
    private static final double TRANSFER_SLOW_FRACTION = 0.4;
    private static final double RAMP_UP = 0.730;
    private static final double RAMP_DOWN = 0.140;
    private static final double TRIGGER_DOWN = 0.5;
    private static final double SLOW_SCALE = 0.4;

    private static final Pose START = AxonTurretAimOdo.START;
    private static final String CSV_NAME = "shot_distance_tuning.csv";

    /** One shot that scored. */
    private static class Record {
        final double distanceIn, targetTps, actualTps, ratio;

        Record(double distanceIn, double targetTps, double actualTps, double ratio) {
            this.distanceIn = distanceIn;
            this.targetTps = targetTps;
            this.actualTps = actualTps;
            this.ratio = ratio;
        }
    }

    private final List<Record> records = new ArrayList<>();
    private final Flywheel flywheel = new Flywheel();
    private DcMotor intake, transfer;
    private Servo flowerServo;
    private String saveStatus = "";

    @Override
    public void runOpMode() {
        intake = hardwareMap.dcMotor.get("intakeMotor");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer = hardwareMap.dcMotor.get("transferMotor");
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        flowerServo = hardwareMap.get(Servo.class, "flowerServo"); // not moved until START

        flywheel.init(hardwareMap);
        flywheel.stopMotors();
        TurretController turret = new TurretController(hardwareMap); // zeroes the turret encoder: must face FRONT

        telemetry.addLine("Calibrating Pinpoint - keep the robot STILL");
        telemetry.update();
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(START);
        // AFTER Constants.create and flywheel.init: both switch the hubs to AUTO caching, this switches them to MANUAL.
        BulkReader bulk = new BulkReader(hardwareMap);
        RedHiveGoal goal = new RedHiveGoal(START.y());

        // This OpMode does its own dip handling (absolute 75 ticks/s); put the class setting back when done.
        boolean savedBoost = Flywheel.BOOST_ENABLED;
        Flywheel.BOOST_ENABLED = false;
        try {
            while (opModeInInit()) {
                bulk.update();
                follower.update();
                Pose pose = follower.pose();
                telemetry.addLine("Robot on the start spot (29.0, 133.7) facing the audience? Turret facing FRONT?");
                telemetry.addData("Pose", "(%.1f, %.1f) %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
                telemetry.update();
            }
            run(bulk, follower, turret, goal);
        } finally {
            Flywheel.BOOST_ENABLED = savedBoost;
            flywheel.stopMotors();
            intake.setPower(0);
            transfer.setPower(0);
            turret.stop();
        }
    }

    private void run(BulkReader bulk, Follower follower, TurretController turret, RedHiveGoal goal) {
        double targetTps = START_TPS;
        boolean flywheelOn = false;
        boolean dip = false;
        boolean intakeOn = false, rightWasDown = false;
        double intakeDirection = 1;

        while (opModeIsActive()) {
            bulk.update();

            // ---- drive + pose ----
            double s = gamepad1.right_bumper ? SLOW_SCALE : 1.0;
            follower.manual(-gamepad1.left_stick_y * s, -gamepad1.left_stick_x * s, -gamepad1.right_stick_x * s);
            follower.update();
            if (gamepad1.startWasPressed()) {
                follower.setPose(START);
                turret.resetHeadingTracking();
            }
            Pose pose = follower.pose();
            turret.updateHeading(pose.heading());
            goal.update(pose.y());

            // ---- turret aims at the cell; distance from the TURRET (same as BioBuzzTeleOp / FlywheelSpeedTable) ----
            turret.aimAt(pose.x(), pose.y(), pose.heading(), goal.x(), goal.y());
            double h = pose.heading();
            double tx = pose.x() + TurretConstants.TURRET_FORWARD_IN * Math.cos(h) - TurretConstants.TURRET_LEFT_IN * Math.sin(h);
            double ty = pose.y() + TurretConstants.TURRET_FORWARD_IN * Math.sin(h) + TurretConstants.TURRET_LEFT_IN * Math.cos(h);
            double distance = Math.hypot(goal.x() - tx, goal.y() - ty);
            double required = requiredBallSpeed(distance); // in/s, NaN if unreachable at 70 deg

            // ---- speed buttons ----
            if (gamepad1.dpadUpWasPressed()) targetTps += STEP_TPS;
            if (gamepad1.dpadDownWasPressed()) targetTps -= STEP_TPS;
            if (gamepad1.bWasPressed()) {
                double p = predictTps(distance);
                if (!Double.isNaN(p)) targetTps = Math.round(p / STEP_TPS) * STEP_TPS; // keep it on the 25 grid
            }
            targetTps = Range.clip(targetTps, 0, MAX_TPS);
            if (gamepad1.xWasPressed()) flywheelOn = !flywheelOn;

            // ---- flywheel with the 75 ticks/s dip rule ----
            double wheel = flywheel.getTicksPerSec();
            if (!flywheelOn) {
                dip = false;
                flywheel.stopMotors();
            } else {
                if (!dip && wheel < targetTps - DIP_TPS) dip = true;
                if (dip && wheel >= targetTps - DIP_EXIT_TPS) dip = false;
                if (dip) flywheel.setMotorPower(1.0); // accelerate as fast as the motor can
                else flywheel.setMotorRPM(Flywheel.ticksPerSecToRpm(targetTps));
            }

            // ---- intake / transfer / ramp ----
            boolean rightDown = gamepad1.right_trigger > TRIGGER_DOWN;
            if (rightDown && !rightWasDown) intakeOn = !intakeOn;
            rightWasDown = rightDown;
            if (gamepad1.yWasPressed()) intakeDirection = -intakeDirection;
            intake.setPower(intakeOn ? intakeDirection * INTAKE_POWER : 0);

            boolean shooting = gamepad1.left_trigger > TRIGGER_DOWN;
            double transferPower = !shooting ? TRANSFER_DOWN_POWER
                    : dip ? TRANSFER_FEED_POWER * TRANSFER_SLOW_FRACTION : TRANSFER_FEED_POWER;
            transfer.setPower(transferPower);
            flowerServo.setPosition(gamepad1.left_bumper ? RAMP_DOWN : RAMP_UP);

            // ---- records ----
            if (gamepad1.aWasPressed()) {
                if (Double.isNaN(required)) {
                    saveStatus = "NOT recorded: too close for a 70° shot to rise " + RISE_IN + " in";
                } else {
                    Record r = new Record(distance, targetTps, wheel, targetTps / required);
                    records.add(r);
                    saveStatus = appendCsv(r);
                }
            }
            if (gamepad1.backWasPressed() && !records.isEmpty()) {
                records.remove(records.size() - 1);
                saveStatus = "last record deleted here (the CSV keeps every line)";
            }

            // ---- telemetry ----
            String state = !flywheelOn ? "OFF" : dip ? "DIP - FULL POWER, transfer slowed"
                    : Math.abs(wheel - targetTps) <= DIP_EXIT_TPS ? "READY" : "settling";
            telemetry.addData("Flywheel (X)", state);
            telemetry.addData("Speed", "%.0f / %.0f ticks/s  (dpad ±%.0f, dip below %.0f)",
                    wheel, targetTps, STEP_TPS, targetTps - DIP_TPS);
            telemetry.addData("Transfer", !shooting ? "down" : dip ? "SLOW feed" : "feeding");
            telemetry.addData("Distance", "%.1f in to the %s cell", distance, goal.aimingTop() ? "TOP" : "BOTTOM");
            telemetry.addData("Physics ball speed", Double.isNaN(required) ? "unreachable (too close)"
                    : String.format("%.0f in/s at %.0f°", required, EXIT_ANGLE_DEG));
            telemetry.addData("Ratio now", Double.isNaN(required) ? "--"
                    : String.format("%.3f ticks/s per in/s", targetTps / required));
            double predicted = predictTps(distance);
            telemetry.addData("Predicted (B)", Double.isNaN(predicted) ? "record a shot first (A)"
                    : String.format("%.0f ticks/s   fit: %s", predicted, fitDescription()));
            telemetry.addData("Pose", "(%.1f, %.1f) %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            if (!saveStatus.isEmpty()) telemetry.addLine(saveStatus);

            telemetry.addLine("--- RECORDS (A add, BACK delete last) ---");
            for (int i = records.size() - 1; i >= 0 && i >= records.size() - 8; i--) {
                Record r = records.get(i);
                telemetry.addLine(String.format("%5.1f in | %4.0f tps (actual %4.0f) | ratio %.3f",
                        r.distanceIn, r.targetTps, r.actualTps, r.ratio));
            }
            addTableRows();
            telemetry.update();
        }
    }

    // ======================= PHYSICS =======================

    /**
     * Ball speed (in/s) needed to pass through a point `distanceIn` away horizontally and RISE_IN higher, leaving at
     * EXIT_ANGLE_DEG, with no air drag:  v = d / cos(a) * sqrt(g / (2 (d tan(a) - rise))).
     * NaN when the target is too close (a 70 deg shot cannot reach that height that soon).
     */
    static double requiredBallSpeed(double distanceIn) {
        double a = Math.toRadians(EXIT_ANGLE_DEG);
        double drop = distanceIn * Math.tan(a) - RISE_IN;
        if (distanceIn <= 0 || drop <= 0) return Double.NaN;
        return distanceIn / Math.cos(a) * Math.sqrt(G_IN_PER_S2 / (2 * drop));
    }

    // ======================= FIT =======================

    /** Ratio at a distance: straight line through the records (or their average if they don't span 12 in yet). */
    private double ratioAt(double distanceIn) {
        int n = records.size();
        if (n == 0) return Double.NaN;
        double sx = 0, sy = 0, sxx = 0, sxy = 0, min = Double.MAX_VALUE, max = -Double.MAX_VALUE;
        for (Record r : records) {
            sx += r.distanceIn;
            sy += r.ratio;
            sxx += r.distanceIn * r.distanceIn;
            sxy += r.distanceIn * r.ratio;
            min = Math.min(min, r.distanceIn);
            max = Math.max(max, r.distanceIn);
        }
        double mean = sy / n;
        if (n < 2 || max - min < 12.0) return mean;
        double slope = (n * sxy - sx * sy) / (n * sxx - sx * sx);
        double intercept = (sy - slope * sx) / n;
        return intercept + slope * distanceIn;
    }

    private double predictTps(double distanceIn) {
        double v = requiredBallSpeed(distanceIn);
        double ratio = ratioAt(distanceIn);
        if (Double.isNaN(v) || Double.isNaN(ratio)) return Double.NaN;
        return ratio * v;
    }

    private String fitDescription() {
        int n = records.size();
        if (n == 0) return "none";
        double min = Double.MAX_VALUE, max = -Double.MAX_VALUE;
        for (Record r : records) {
            min = Math.min(min, r.distanceIn);
            max = Math.max(max, r.distanceIn);
        }
        if (n < 2 || max - min < 12.0) return String.format("average of %d (spread them out for a line)", n);
        return String.format("line through %d records, %.0f-%.0f in (outside that is a guess)", n, min, max);
    }

    /** FlywheelSpeedTable rows from the fit. The table multiplies by SPEED_SCALE, so the rows are divided by it. */
    private void addTableRows() {
        if (records.isEmpty()) return;
        StringBuilder rows = new StringBuilder();
        for (int i = 0; i < FlywheelSpeedTable.DISTANCE_IN.length; i++) {
            double p = predictTps(FlywheelSpeedTable.DISTANCE_IN[i]);
            if (i > 0) rows.append(", ");
            rows.append(Double.isNaN(p) ? "--" : String.format("%.0f", p / FlywheelSpeedTable.SPEED_SCALE));
        }
        telemetry.addLine("--- Table rows for FlywheelSpeedTable.VELOCITY_TPS ---");
        telemetry.addLine("at " + java.util.Arrays.toString(FlywheelSpeedTable.DISTANCE_IN) + " in:");
        telemetry.addLine("{" + rows + "}  (already divided by SPEED_SCALE " + FlywheelSpeedTable.SPEED_SCALE + ")");
    }

    // ======================= CSV =======================

    /** Appends one record to FIRST/shot_distance_tuning.csv on the Control Hub (kept between runs). */
    private String appendCsv(Record r) {
        File file = new File(AppUtil.FIRST_FOLDER, CSV_NAME);
        boolean isNew = !file.exists();
        try (FileWriter w = new FileWriter(file, true)) {
            if (isNew) w.write("distance_in,target_tps,actual_tps,ratio,exit_angle_deg,rise_in\n");
            w.write(String.format("%.1f,%.0f,%.0f,%.4f,%.0f,%.1f\n",
                    r.distanceIn, r.targetTps, r.actualTps, r.ratio, EXIT_ANGLE_DEG, RISE_IN));
            return String.format("Recorded %.1f in @ %.0f (saved to FIRST/%s)", r.distanceIn, r.targetTps, CSV_NAME);
        } catch (IOException e) {
            return "Recorded here, but the CSV save failed: " + e.getMessage();
        }
    }
}
