package org.firstinspires.ftc.teamcode.biobuzz.comp1.turret;

import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;

/**
 * RED alliance: keeps the turret pointed at our HIVE (the goal) while the robot is pushed around by hand.
 *   robot in the BOTTOM half of the field (Y < 72, audience side) -> aims at the BOTTOM (audience) cell
 *   robot in the TOP half (Y > 72)                                -> aims at the TOP (scoring) cell
 *
 * Only the turret motor is ever powered. Odometry comes straight from the Pinpoint (Pedro's PinpointLocalizer,
 * not the Follower), so the drive motors are never touched.
 *
 * FIELD FRAME (Pedro): origin = bottom-left corner seen from the audience (red wall / audience wall),
 * +X right, +Y away from the audience, heading 0 = facing +X, counter-clockwise positive, inches.
 *
 * BEFORE PRESSING INIT
 *   1. Robot at START_POSE: bottom-left corner (red wall / audience wall), FACING AWAY FROM THE AUDIENCE
 *      (+Y, toward the far wall), back against the audience wall, left side against the red wall.
 *   2. Turret pointing straight out the robot FRONT (also away from the audience). That is turret 0 deg.
 *   3. Robot perfectly still until telemetry says ready (Pinpoint IMU calibrates).
 *
 * gamepad1:
 *   A            - toggle auto aim (on at start)
 *   left stick X - manual turret while auto aim is off
 *   X            - reset odometry to START_POSE (put the robot back on the start spot first, hold it still)
 */
@TeleOp(name = "Turret Aim Goal (Red)", group = "BioBuzz Tests")
public class TurretAimGoal extends LinearOpMode {

    // ======================= START POSE =======================
    /** Left side against the red wall. */
    private static final double START_X = Constants.ROBOT_WIDTH_IN / 2.0;
    /** Back against the audience wall. */
    private static final double START_Y = Constants.ROBOT_LENGTH_IN / 2.0;
    private static final double START_HEADING_DEG = 90.0;
    private static final Pose START_POSE = new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));

    private PinpointLocalizer localizer;
    private TurretController turret;

    @Override
    public void runOpMode() {
        turret = new TurretController(hardwareMap);

        telemetry.addLine("Resetting Pinpoint - keep the robot STILL");
        telemetry.update();
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        resetOdometry();

        RedHiveGoal goal = new RedHiveGoal(START_Y);

        // Odometry runs during INIT too (turret stays unpowered), so you can check the pose before START.
        while (opModeInInit()) {
            Pose pose = updatePose();
            telemetry.addLine("Ready. Robot in bottom-left corner facing away from audience, turret facing FRONT?");
            addTelemetry(pose, goal, Double.NaN);
            telemetry.update();
        }

        boolean autoAim = true;
        while (opModeIsActive()) {
            Pose pose = updatePose();

            if (gamepad1.aWasPressed()) autoAim = !autoAim;
            if (gamepad1.xWasPressed()) {
                turret.stop();
                resetOdometry();
                pose = localizer.pose();
            }

            goal.update(pose.y());

            double targetDeg = Double.NaN;
            if (autoAim) {
                targetDeg = turret.aimAt(pose.x(), pose.y(), pose.heading(), goal.x(), goal.y());
            } else {
                turret.manual(-gamepad1.left_stick_x); // stick left = counter-clockwise
            }

            telemetry.addData("Auto aim (A)", autoAim);
            addTelemetry(pose, goal, targetDeg);
            telemetry.update();
        }

        turret.stop();
    }

    private void resetOdometry() {
        localizer.reset(); // recalibrates the Pinpoint IMU (500 ms wait), robot must be still
        localizer.setPose(START_POSE);
        turret.resetHeadingTracking();
    }

    private Pose updatePose() {
        localizer.update();
        Pose pose = localizer.pose();
        turret.updateHeading(pose.heading());
        return pose;
    }

    private void addTelemetry(Pose pose, RedHiveGoal goal, double targetDeg) {
        telemetry.addLine("A auto aim | X reset to start pose | stick X manual");
        telemetry.addData("Robot", "(%.1f, %.1f) %.1f°",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Field half", pose.y() < RedHiveGoal.HALF_LINE_Y ? "BOTTOM" : "TOP");
        telemetry.addData("Aiming at", "%s cell (%.1f, %.1f), %.1f in away", goal.aimingTop() ? "TOP" : "BOTTOM",
                goal.x(), goal.y(), Math.hypot(goal.x() - pose.x(), goal.y() - pose.y()));
        double cur = turret.angleDeg();
        telemetry.addData("Turret", "%.1f°  (limits %.0f .. %.0f)", cur, TurretConstants.MIN_DEG, TurretConstants.MAX_DEG);
        if (!Double.isNaN(targetDeg)) {
            telemetry.addData("Turret target", "%.1f°  error %+.1f°", targetDeg, targetDeg - cur);
        }
    }
}
