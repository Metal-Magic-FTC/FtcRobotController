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

        while (opModeIsActive()) {
            double s = gamepad1.right_bumper ? SLOW_SCALE : 1.0;
            follower.manual(-gamepad1.left_stick_y * s, -gamepad1.left_stick_x * s, -gamepad1.right_stick_x * s);
            follower.update();
            Pose pose = follower.pose();
            turret.update(pose.heading());
            goal.update(pose.y());

            if (gamepad1.aWasPressed()) autoAim = !autoAim;
            if (gamepad1.xWasPressed()) limp = !limp;
            if (gamepad1.bWasPressed()) {
                follower.setPose(START);
                turret.resetHeadingTracking();
            }

            double target = Double.NaN;
            if (limp) {
                turret.stop();
            } else if (autoAim) {
                target = turret.aimAt(pose.x(), pose.y(), pose.heading(), goal.x(), goal.y());
            } else {
                turret.manual(gamepad1.left_trigger - gamepad1.right_trigger); // left trigger = turret left (+)
            }

            telemetry.addData("Mode", limp ? "LIMP (X)" : autoAim ? "AUTO AIM (A)" : "MANUAL (triggers)");
            telemetry.addData("Pose", "(%.1f, %.1f) %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            telemetry.addData("Aiming at", "%s cell (%.1f, %.1f)", goal.aimingTop() ? "TOP" : "BOTTOM", goal.x(), goal.y());
            telemetry.addData("Turret", "%.1f°  (limits %.0f .. %.0f)  %.3f V", turret.angleDeg(),
                    AxonTurretController.MIN_DEG, AxonTurretController.MAX_DEG, turret.volts());
            if (!Double.isNaN(target)) {
                telemetry.addData("Turret target", "%.1f°  error %+.1f°", target, target - turret.angleDeg());
            }
            telemetry.addData("Power", "%+.2f   robot turn %.0f°/s", turret.power(), turret.robotOmegaDeg());
            telemetry.update();
        }
        turret.stop();
    }
}
