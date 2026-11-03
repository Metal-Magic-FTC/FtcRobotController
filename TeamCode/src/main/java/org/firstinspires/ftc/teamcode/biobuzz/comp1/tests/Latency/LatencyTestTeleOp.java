package org.firstinspires.ftc.teamcode.biobuzz.comp1.tests.Latency;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.TurretController;
import org.firstinspires.ftc.teamcode.biobuzz.util.BulkReader;

/**
 * Loop-time comparison. LatencyTestBulkReadOn and LatencyTestBulkReadOff run exactly this loop; the only difference
 * is whether BulkReader is used. Run each, drive the same way for ~20 s, press X to reset the numbers, and compare.
 *
 * Each loop does what a real teleop does: Pedro drive (Pinpoint odometry), turret auto-aim (turret encoder +
 * velocity), and telemetry that reads the drive motors' encoders and velocities.
 *
 * Robot starts at (0, 0) facing +X. BEFORE INIT: turret pointing out the robot FRONT.
 *
 * gamepad1:
 *   left stick - drive, right stick X - turn, right bumper - slow mode
 *   A - toggle turret auto aim (on at start), triggers - manual turret while auto aim is off
 *   X - reset the loop-time numbers
 */
public abstract class LatencyTestTeleOp extends LinearOpMode {

    private static final double SLOW_MODE_SCALE = 0.4;
    /** Field point the turret aims at, inches. To the robot's right because the turret only turns clockwise. */
    private static final double TARGET_X = 48.0, TARGET_Y = -48.0;
    private static final String[] DRIVE_NAMES = {"frontLeft", "frontRight", "backLeft", "backRight"};

    protected abstract boolean useBulkReading();

    @Override
    public void runOpMode() {
        BulkReader bulk = null;
        if (useBulkReading()) {
            bulk = new BulkReader(hardwareMap);
        } else {
            // Hubs can keep the mode from the previous OpMode, so turn it off explicitly for a fair test.
            for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
            }
        }

        TurretController turret = new TurretController(hardwareMap);
        telemetry.addLine("Calibrating Pinpoint - keep the robot STILL");
        telemetry.update();
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));

        // Pedro owns these motors; we only read them for telemetry.
        DcMotorEx[] drive = new DcMotorEx[DRIVE_NAMES.length];
        for (int i = 0; i < drive.length; i++) drive[i] = hardwareMap.get(DcMotorEx.class, DRIVE_NAMES[i]);

        telemetry.addData("Bulk reading", useBulkReading() ? "ON" : "OFF");
        telemetry.addLine("Turret facing robot FRONT? Press START.");
        telemetry.update();
        waitForStart();

        boolean autoAim = true;
        ElapsedTime loopTimer = new ElapsedTime();
        double lastMs = 0, totalMs = 0, maxMs = 0;
        int loops = 0;

        while (opModeIsActive()) {
            if (bulk != null) bulk.update();

            if (gamepad1.aWasPressed()) autoAim = !autoAim;
            if (gamepad1.xWasPressed()) {
                totalMs = maxMs = 0;
                loops = 0;
            }

            double scale = gamepad1.right_bumper ? SLOW_MODE_SCALE : 1.0;
            follower.manual(-gamepad1.left_stick_y * scale, -gamepad1.left_stick_x * scale,
                    -gamepad1.right_stick_x * scale);
            follower.update();
            Pose pose = follower.pose();
            turret.updateHeading(pose.heading());

            double targetDeg = Double.NaN;
            if (autoAim) {
                targetDeg = turret.aimAt(pose.x(), pose.y(), pose.heading(), TARGET_X, TARGET_Y);
            } else {
                turret.manual(gamepad1.left_trigger - gamepad1.right_trigger);
            }

            telemetry.addData("Bulk reading", useBulkReading() ? "ON" : "OFF");
            telemetry.addData("Loop", "avg %.1f ms (%.0f Hz) | max %.1f | last %.1f",
                    loops == 0 ? 0 : totalMs / loops, loops == 0 ? 0 : 1000.0 * loops / totalMs, maxMs, lastMs);
            telemetry.addData("Loops measured", "%d  (X to reset)", loops);
            telemetry.addLine();
            telemetry.addData("Robot", "(%.1f, %.1f) %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            telemetry.addData("Turret", autoAim ? "%.1f° -> %.1f° (auto aim)" : "%.1f° (manual)",
                    turret.angleDeg(), targetDeg);
            for (int i = 0; i < drive.length; i++) {
                telemetry.addData(DRIVE_NAMES[i], "%d ticks, %.0f ticks/s",
                        drive[i].getCurrentPosition(), drive[i].getVelocity());
            }
            telemetry.update();

            lastMs = loopTimer.milliseconds();
            loopTimer.reset();
            totalMs += lastMs;
            maxMs = Math.max(maxMs, lastMs);
            loops++;
        }

        turret.stop();
    }
}
