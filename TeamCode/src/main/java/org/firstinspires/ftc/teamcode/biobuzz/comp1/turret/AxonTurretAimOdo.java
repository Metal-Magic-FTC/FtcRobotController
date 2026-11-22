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
 *   Y / A             - KS +/- 0.005             right / left bumper - KV x1.25 / /1.25
 *   B / X             - tolerance +/- 0.25 deg
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
            if (gamepad2.yWasPressed()) AxonTurretController.KS += 0.005;
            if (gamepad2.aWasPressed()) AxonTurretController.KS = Math.max(0, AxonTurretController.KS - 0.005);
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
            telemetry.addData("Gains (gamepad2)", "KP %.4f  KD %.5f  KS %.3f  KV %.4f  tol %.2f°",
                    AxonTurretController.KP, AxonTurretController.KD, AxonTurretController.KS,
                    AxonTurretController.KV, AxonTurretController.TOLERANCE_DEG);
            telemetry.update();
        }
        turret.stop();
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
