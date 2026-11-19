package org.firstinspires.ftc.teamcode.biobuzz.comp1.turret;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry;

/**
 * RED alliance, Axon MK2 turret: learn the turret's travel by hand, lock it in as hard limits, then auto-aim at
 * the CENTER of the red HIVE. Only the turret servo is ever powered; the drive motors are never touched.
 *
 * BEFORE PRESSING INIT
 *   1. Robot at START_POSE (bottom-left corner, red wall / audience wall, facing away from the audience),
 *      same spot as TurretAimGoal.
 *   2. Turret pointing straight out the robot FRONT. That becomes turret 0 deg.
 *   3. Robot still while telemetry says it is resetting the Pinpoint.
 *
 * STEP 1, LIMITS (works in INIT, and after START until locked)
 *   left stick X - drive the turret SLOWLY (fixed AxonTurret.CAPTURE_POWER, stick left = counter-clockwise).
 *                  Drive it all the way to one end, then all the way to the other. When it reaches an end and
 *                  stops moving, that direction cuts out ("END STOP" on telemetry); let go of the stick to re-arm.
 *   A            - lock the hard limits (one time; restart the OpMode to redo)
 *   B            - restart the capture
 *   Finish by driving the turret back near the front, so it does not take off when auto aim starts.
 *
 * STEP 2, AIM (after START, once limits are locked)
 *   A            - toggle auto aim (on by default)
 *   left stick X - manual turret while auto aim is off (still inside the limits)
 *   dpad up/down - raise/lower the speed cap (starts at AxonTurret.START_MAX_POWER)
 *   B            - toggle limp (PWM off)
 *   X            - reset odometry to START_POSE (put the robot back on the start spot first, hold it still)
 */
@TeleOp(name = "Axon Turret Aim Hive (Red)", group = "BioBuzz Tests")
public class AxonTurretAimHive extends LinearOpMode {

    private static final double START_X = Constants.ROBOT_WIDTH_IN / 2.0;
    private static final double START_Y = Constants.ROBOT_LENGTH_IN / 2.0;
    private static final double START_HEADING_DEG = 90.0;
    private static final Pose START_POSE = new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));

    /** Center of the red HIVE (between the two cells, on the pivot line). */
    private static final double HIVE_X = HiveTagGeometry.RED_HIVE_X_IN;
    private static final double HIVE_Y = HiveTagGeometry.PIVOT_Y_IN;

    private static final double SPEED_STEP = 0.05;

    private PinpointLocalizer localizer;
    private AxonTurret turret;
    private String lockMessage = "";

    @Override
    public void runOpMode() {
        turret = new AxonTurret(hardwareMap); // limp unless the stick is driving it

        telemetry.addLine("Resetting Pinpoint - keep the robot STILL");
        telemetry.update();
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        resetOdometry();
        turret.update();
        turret.resetCapture();

        while (opModeInInit()) {
            Pose pose = updateAll();
            handleCapture();
            telemetry.addLine(turret.limitsLocked()
                    ? "Limits LOCKED. Turret off until START."
                    : "Stick X: drive SLOWLY to both ends, then A to lock. B = restart capture.");
            addTelemetry(pose, Double.NaN, Double.NaN);
            telemetry.update();
        }

        boolean autoAim = true;
        while (opModeIsActive()) {
            Pose pose = updateAll();

            if (!turret.limitsLocked()) {
                // Still capturing: only the slow capture drive, no auto aim.
                handleCapture();
                telemetry.addLine("Limits NOT locked. Stick X: drive SLOWLY to both ends, then A to lock.");
                addTelemetry(pose, Double.NaN, Double.NaN);
                telemetry.update();
                continue;
            }

            if (gamepad1.aWasPressed()) autoAim = !autoAim;
            if (gamepad1.bWasPressed()) turret.setLimp(!turret.isLimp());
            if (gamepad1.dpadUpWasPressed()) turret.changeMaxPower(SPEED_STEP);
            if (gamepad1.dpadDownWasPressed()) turret.changeMaxPower(-SPEED_STEP);
            if (gamepad1.xWasPressed()) {
                turret.stop();
                resetOdometry();
                pose = localizer.pose();
            }

            double desired = TurretController.desiredDeg(pose.x(), pose.y(), pose.heading(), HIVE_X, HIVE_Y);
            double target = Double.NaN;
            if (autoAim) {
                target = turret.chooseTarget(desired);
                turret.goTo(target);
            } else {
                turret.manual(-gamepad1.left_stick_x); // stick left = counter-clockwise
            }

            telemetry.addData("Auto aim (A)", autoAim);
            addTelemetry(pose, desired, target);
            telemetry.update();
        }

        turret.stop();
    }

    private void handleCapture() {
        if (turret.limitsLocked()) {
            turret.stop();
            return;
        }
        turret.captureDrive(-gamepad1.left_stick_x); // stick left = counter-clockwise
        if (gamepad1.bWasPressed()) {
            turret.resetCapture();
            lockMessage = "Capture restarted.";
        }
        if (gamepad1.aWasPressed()) {
            lockMessage = turret.lockLimits()
                    ? "Locked."
                    : String.format("Range too small to lock (need > %.0f° after %.0f° margin each side).",
                    AxonTurret.MIN_SPAN_DEG, AxonTurret.LIMIT_MARGIN_DEG);
        }
    }

    private Pose updateAll() {
        turret.update();
        localizer.update();
        return localizer.pose();
    }

    private void resetOdometry() {
        localizer.reset(); // recalibrates the Pinpoint IMU (~500 ms), robot must be still
        localizer.setPose(START_POSE);
    }

    private static String fmtDeg(double deg) {
        return Double.isNaN(deg) ? "--" : String.format("%.1f°", deg);
    }

    private void addTelemetry(Pose pose, double desiredDeg, double targetDeg) {
        if (turret.fault() != null) {
            telemetry.addLine("!!! TURRET FAULT (restart OpMode): " + turret.fault());
        }
        if (!lockMessage.isEmpty()) telemetry.addLine(lockMessage);

        telemetry.addData("Turret angle", "%.1f°  (feedback %.3f V)", turret.angleDeg(), turret.feedbackVoltage());
        if (turret.limitsLocked()) {
            telemetry.addData("HARD limits", "%.1f° .. %.1f°", turret.minDeg(), turret.maxDeg());
        } else {
            telemetry.addData("Seen so far", "%.1f° .. %.1f°  (span %.1f°)",
                    turret.seenMinDeg(), turret.seenMaxDeg(), turret.seenMaxDeg() - turret.seenMinDeg());
            telemetry.addData("End stops felt", "CW %s | CCW %s",
                    fmtDeg(turret.endStopNegDeg()), fmtDeg(turret.endStopPosDeg()));
            telemetry.addData("Capture speed", "%.2f (fixed)", AxonTurret.CAPTURE_POWER);
            if (turret.atEndStop()) telemetry.addLine("END STOP reached - release the stick");
        }
        telemetry.addData("Speed cap (dpad)", "%.2f  (max %.2f)", turret.maxPower(), AxonTurret.ABSOLUTE_MAX_POWER);
        telemetry.addData("Power", "%.2f%s", turret.lastPower(), turret.isLimp() ? "  LIMP" : "");

        telemetry.addData("Robot", "(%.1f, %.1f) %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Hive center", "(%.1f, %.1f), %.1f in away",
                HIVE_X, HIVE_Y, Math.hypot(HIVE_X - pose.x(), HIVE_Y - pose.y()));
        if (!Double.isNaN(targetDeg)) {
            telemetry.addData("Aim", "want %.1f°, target %.1f°, error %+.1f°%s", desiredDeg, targetDeg,
                    targetDeg - turret.angleDeg(), turret.targetInDeadZone(desiredDeg) ? "  (OUT OF RANGE)" : "");
        }
    }
}
