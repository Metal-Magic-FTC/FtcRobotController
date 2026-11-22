package org.firstinspires.ftc.teamcode.biobuzz.comp1.turret;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;

/**
 * RED: drive around, the Axon turret keeps aiming at the red HIVE using ODOMETRY ONLY (no Limelight).
 * Aims at the bottom cell in the bottom half of the field, the top cell in the top half (RedHiveGoal).
 *
 * BEFORE PRESSING INIT
 *   1. Robot on the Auto V4 start spot: (29.0, 133.7) facing the audience (heading 270).
 *   2. Robot STILL while the Pinpoint calibrates. The turret can point anywhere (absolute encoder).
 *
 * gamepad1:
 *   left stick / right stick X - drive (robot-centric) / turn,  right bumper - slow
 *   A                          - auto aim on / off (OFF at start - check direction with the triggers first)
 *   left / right trigger       - manual turret left / right (while auto aim is off)
 *   B                          - reset pose to the start spot (put the robot there first)
 *   X                          - turret limp (power 0) toggle
 *   dpad up / left / right / down - TEST: hold the turret at 0 (front) / +90 (left) / -90 (right) / 180 (back)
 *                                   robot-relative. Checks the angle reading without odometry. Press A to go back.
 *
 * gamepad2 (LIVE TUNING - copy the final numbers into AxonTurretController):
 *   dpad up / down    - KP x1.25 / /1.25        dpad right / left - KD x1.25 / /1.25
 *   Y / A             - SERVO_DEADBAND +/- 0.005 right / left bumper - KV x1.25 / /1.25
 *   B / X             - tolerance +/- 0.25 deg
 *   gamepad1 BACK     - measure SERVO_DEADBAND automatically (turret creeps a few degrees each way)
 *   gamepad1 START    - measure KV automatically (turret spins ~90 deg at two speeds, toward the middle)
 *   Tip: use gamepad1 dpad up / left to jump the turret between 0 and 90 and watch it settle.
 *
 * At START it nudges the turret for a moment to learn which way the servo turns (no need to set SERVO_REVERSED).
 */
@TeleOp(name = "Axon Turret Aim (Red, odo only)", group = "BioBuzz Tests")
public class AxonTurretAimOdo extends LinearOpMode {

    private static final PoseFactory P = PoseFactory.degrees();
    /** Same start as Auto V4. */
    public static final Pose START = P.of(29.0132, 133.6524, 270);
    private static final double SLOW_SCALE = 0.4;

    @Override
    public void runOpMode() {
        AxonTurretController turret = new AxonTurretController(hardwareMap);

        telemetry.addLine("Calibrating Pinpoint - keep the robot STILL");
        telemetry.update();
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(START);
        RedHiveGoal goal = new RedHiveGoal(START.y());

        boolean autoAim = false; // off at start: check the servo direction with the triggers first
        boolean limp = false;
        double testAngle = Double.NaN; // dpad test target, NaN = off

        while (opModeInInit()) {
            follower.update();
            Pose pose = follower.pose();
            turret.update(pose.heading());
            turret.stop();
            telemetry.addLine("Robot on the start spot (29.0, 133.7) facing the audience?");
            telemetry.addData("Pose", "(%.1f, %.1f) %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            telemetry.addData("Turret", "%.1f°  (%.3f V)", turret.angleDeg(), turret.volts());
            telemetry.update();
        }
        follower.manual(0, 0, 0);

        // ---- Direction check: nudge the turret and see which way the encoder moves ----
        String directionResult = checkDirection(turret, follower);
        String deadbandResult = "not measured (gamepad1 BACK)";
        String speedResult = "not measured (gamepad1 START)";
        com.qualcomm.robotcore.util.ElapsedTime loopTimer = new com.qualcomm.robotcore.util.ElapsedTime();
        double loopMs = 0;

        while (opModeIsActive()) {
            double s = gamepad1.right_bumper ? SLOW_SCALE : 1.0;
            follower.manual(-gamepad1.left_stick_y * s, -gamepad1.left_stick_x * s, -gamepad1.right_stick_x * s);
            follower.update();
            Pose pose = follower.pose();
            turret.update(pose.heading());
            goal.update(pose.y());

            if (gamepad1.dpadUpWasPressed()) testAngle = 0;
            if (gamepad1.dpadLeftWasPressed()) testAngle = 90;
            if (gamepad1.dpadRightWasPressed()) testAngle = -90;
            if (gamepad1.dpadDownWasPressed()) testAngle = turret.angleDeg() >= 0 ? 178 : -178;
            if (gamepad1.aWasPressed()) {
                autoAim = !autoAim;
                testAngle = Double.NaN;
            }
            if (gamepad1.xWasPressed()) limp = !limp;
            if (gamepad1.bWasPressed()) {
                follower.setPose(START);
                turret.resetHeadingTracking();
            }

            double target = Double.NaN;
            // ---- live tuning on gamepad2 ----
            if (gamepad2.dpadUpWasPressed()) AxonTurretController.KP *= 1.25;
            if (gamepad2.dpadDownWasPressed()) AxonTurretController.KP /= 1.25;
            if (gamepad2.dpadRightWasPressed()) AxonTurretController.KD =
                    AxonTurretController.KD <= 0 ? 0.0002 : AxonTurretController.KD * 1.25;
            if (gamepad2.dpadLeftWasPressed()) AxonTurretController.KD /= 1.25;
            if (gamepad2.yWasPressed()) AxonTurretController.SERVO_DEADBAND += 0.005;
            if (gamepad2.aWasPressed()) AxonTurretController.SERVO_DEADBAND =
                    Math.max(0, AxonTurretController.SERVO_DEADBAND - 0.005);
            if (gamepad1.backWasPressed()) deadbandResult = measureDeadband(turret, follower);
            if (gamepad1.startWasPressed()) speedResult = measureSpeed(turret, follower);
            loopMs += 0.1 * (loopTimer.milliseconds() - loopMs);
            loopTimer.reset();
            if (gamepad2.rightBumperWasPressed()) AxonTurretController.KV *= 1.25;
            if (gamepad2.leftBumperWasPressed()) AxonTurretController.KV /= 1.25;
            if (gamepad2.bWasPressed()) AxonTurretController.TOLERANCE_DEG += 0.25;
            if (gamepad2.xWasPressed()) AxonTurretController.TOLERANCE_DEG =
                    Math.max(0, AxonTurretController.TOLERANCE_DEG - 0.25);

            double desired = AxonTurretController.desiredDeg(pose.x(), pose.y(), pose.heading(), goal.x(), goal.y());
            if (limp) {
                turret.stop();
            } else if (!Double.isNaN(testAngle)) {
                turret.driveTo(testAngle);
                target = testAngle;
            } else if (autoAim) {
                target = turret.aimAt(pose.x(), pose.y(), pose.heading(), goal.x(), goal.y());
            } else {
                turret.manual(gamepad1.left_trigger - gamepad1.right_trigger); // left trigger = turret left (+)
            }

            telemetry.addData("Mode", limp ? "LIMP (X)" : !Double.isNaN(testAngle) ? "TEST HOLD (dpad), A to leave"
                    : autoAim ? "AUTO AIM (A)" : "MANUAL (triggers)");
            telemetry.addData("Direction check", directionResult);
            telemetry.addData("HIVE is at", "%+.1f° from robot front  (+ = left)", desired);
            telemetry.addData("Pose", "(%.1f, %.1f) %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            telemetry.addData("Aiming at", "%s cell (%.1f, %.1f)", goal.aimingTop() ? "TOP" : "BOTTOM", goal.x(), goal.y());
            telemetry.addData("Turret", "%.1f°  (limits %.0f .. %.0f)  %.3f V", turret.angleDeg(),
                    AxonTurretController.MIN_DEG, AxonTurretController.MAX_DEG, turret.volts());
            if (!Double.isNaN(target)) {
                telemetry.addData("Turret target", "%.1f°  error %+.1f°", target, target - turret.angleDeg());
            }
            telemetry.addData("Power", "%+.2f   robot turn %.0f°/s", turret.power(), turret.robotOmegaDeg());
            telemetry.addData("Gains (gamepad2)", "KP %.4f  KD %.5f  deadband %.3f  KV %.4f  tol %.2f°",
                    AxonTurretController.KP, AxonTurretController.KD, AxonTurretController.SERVO_DEADBAND,
                    AxonTurretController.KV, AxonTurretController.TOLERANCE_DEG);
            telemetry.addData("Deadband measure (BACK)", deadbandResult);
            telemetry.addData("KV measure (START)", speedResult);
            telemetry.addData("Loop", "%.0f ms", loopMs);
            telemetry.update();
        }
        turret.stop();
    }

    /**
     * Runs the turret at two raw powers and measures its speed, then sets KV = power per deg/s (from the slope) and
     * SERVO_DEADBAND = where the speed line hits zero. Runs toward the middle of the travel so it never hits a limit.
     */
    private String measureSpeed(AxonTurretController turret, Follower follower) {
        double[] powers = {0.2, 0.4};
        double[] speeds = new double[2];
        for (int i = 0; i < 2 && opModeIsActive(); i++) {
            turret.update(follower.pose().heading());
            double dir = turret.angleDeg() > 0 ? -1 : 1; // toward / through the middle
            double start = turret.angleDeg();
            com.qualcomm.robotcore.util.ElapsedTime t = new com.qualcomm.robotcore.util.ElapsedTime();
            double tStart = 0, aStart = start;
            boolean timing = false;
            while (opModeIsActive() && t.seconds() < 0.8 && Math.abs(turret.angleDeg() - start) < 150) {
                follower.update();
                turret.update(follower.pose().heading());
                turret.rawPower(dir * powers[i]);
                if (!timing && t.seconds() > 0.25) { // skip the spin-up
                    timing = true;
                    tStart = t.seconds();
                    aStart = turret.angleDeg();
                }
            }
            double elapsed = t.seconds() - tStart;
            speeds[i] = timing && elapsed > 0.1 ? Math.abs(turret.angleDeg() - aStart) / elapsed : Double.NaN;
            turret.stop();
            t.reset();
            while (opModeIsActive() && t.seconds() < 0.5) {
                follower.update();
                turret.update(follower.pose().heading());
            }
        }
        if (Double.isNaN(speeds[0]) || Double.isNaN(speeds[1]) || speeds[1] <= speeds[0] + 1) {
            return String.format("FAILED (speeds %.0f / %.0f deg/s)", speeds[0], speeds[1]);
        }
        double degPerSecPerPower = (speeds[1] - speeds[0]) / (powers[1] - powers[0]);
        double deadband = powers[0] - speeds[0] / degPerSecPerPower;
        // Our command is shifted past the deadband, so KV maps deg/s onto the part ABOVE it.
        // speed = slope * (DB + (1-DB) * p - DB)  ->  p = speed / (slope * (1-DB))
        AxonTurretController.KV = 1.0 / (degPerSecPerPower * (1.0 - AxonTurretController.SERVO_DEADBAND));
        return String.format("%.0f deg/s @0.2, %.0f deg/s @0.4 -> KV = %.5f, deadband ~%.3f (copy KV into code)",
                speeds[0], speeds[1], AxonTurretController.KV, deadband);
    }

    /**
     * Finds the smallest power that actually moves the turret, each direction, and sets SERVO_DEADBAND to it.
     * Ramps the RAW servo power up slowly from 0 until the turret moves at 3 deg/s.
     */
    private String measureDeadband(AxonTurretController turret, Follower follower) {
        double saved = AxonTurretController.SERVO_DEADBAND;
        AxonTurretController.SERVO_DEADBAND = 0; // measure the raw servo
        double[] found = new double[2];
        double[] dirs = {turret.angleDeg() > 0 ? -1 : 1, turret.angleDeg() > 0 ? 1 : -1}; // first toward the middle
        for (int i = 0; i < 2 && opModeIsActive(); i++) {
            double power = 0;
            found[i] = Double.NaN;
            com.qualcomm.robotcore.util.ElapsedTime t = new com.qualcomm.robotcore.util.ElapsedTime();
            double startAngle = turret.angleDeg();
            while (opModeIsActive() && power < 0.3) {
                follower.update();
                turret.update(follower.pose().heading());
                power = 0.02 * t.seconds() + 0.005; // +0.02 power per second
                turret.rawPower(dirs[i] * power);
                if (Math.abs(turret.angleDeg() - startAngle) > 1.0 && Math.abs(turret.turretVelDeg()) > 3.0) {
                    found[i] = power;
                    break;
                }
            }
            turret.stop();
            t.reset();
            while (opModeIsActive() && t.seconds() < 0.4) { // let it stop
                follower.update();
                turret.update(follower.pose().heading());
            }
        }
        if (Double.isNaN(found[0]) || Double.isNaN(found[1])) {
            AxonTurretController.SERVO_DEADBAND = saved;
            return "FAILED - turret did not move below 0.3 power";
        }
        // Slightly under the measured start so tiny commands don't overshoot
        AxonTurretController.SERVO_DEADBAND = 0.9 * Math.min(found[0], found[1]);
        return String.format("moves at %.3f / %.3f -> SERVO_DEADBAND = %.3f (copy into code)",
                found[0], found[1], AxonTurretController.SERVO_DEADBAND);
    }

    /**
     * Pushes the turret gently for 0.3 s (away from the nearer limit) and checks the encoder moved the same way.
     * Flips the servo direction if it moved the opposite way.
     */
    private String checkDirection(AxonTurretController turret, Follower follower) {
        turret.update(follower.pose().heading());
        double start = turret.angleDeg();
        double push = start > 0 ? -0.2 : 0.2; // toward the middle of the travel
        com.qualcomm.robotcore.util.ElapsedTime t = new com.qualcomm.robotcore.util.ElapsedTime();
        while (opModeIsActive() && t.seconds() < 0.3) {
            follower.update();
            turret.update(follower.pose().heading());
            turret.rawPower(push);
        }
        turret.stop();
        t.reset();
        while (opModeIsActive() && t.seconds() < 0.2) { // let it stop
            follower.update();
            turret.update(follower.pose().heading());
        }
        double moved = turret.angleDeg() - start;
        if (Math.abs(moved) < 1.0) {
            return String.format("FAILED - turret moved only %.1f° (servo not moving? encoder?)", moved);
        }
        if (Math.signum(moved) != Math.signum(push)) {
            turret.setDirectionSign(-1);
            return String.format("servo was backwards - FLIPPED (moved %+.1f°). Set SERVO_REVERSED = %b",
                    moved, !AxonTurretController.SERVO_REVERSED);
        }
        return String.format("OK (moved %+.1f°)", moved);
    }
}
