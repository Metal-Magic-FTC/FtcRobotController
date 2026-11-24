package org.firstinspires.ftc.teamcode.biobuzz.comp1.teleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher.LauncherConstants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.AxonTurretAimOdo;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.TurretController;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry;

/**
 * RED TeleOp, everything together: drive, motor turret (TurretController) auto-aiming at the CENTER of the red HIVE (odometry only),
 * tuned flywheel, intake and shoot.
 *
 * BEFORE PRESSING INIT: robot on the Auto V4 start spot (29.0, 133.7) facing the audience, and STILL while the
 * Pinpoint calibrates. TURRET POINTING STRAIGHT OUT THE ROBOT FRONT: its encoder is zeroed at INIT (that is 0 deg).
 *
 * gamepad1 (driver)
 *   left stick / right stick X - drive (robot-centric) / turn      right bumper - slow
 *   left trigger  (hold)       - INTAKE: intake in, transfer backward at 0.25 (keeps balls out of the flywheel)
 *   right trigger (hold)       - SHOOT:  intake in, transfer forward at 0.6
 *   X (hold)                   - unjam: intake and transfer both backward
 *   Y                          - flywheel on / off
 *   A                          - turret auto aim on / off (on at start); when off, dpad left/right turns it
 *   B                          - reset pose to the start spot (put the robot there first)
 * gamepad2 (operator)
 *   dpad up / down             - flywheel target +/- 50 ticks/s (starts at 1650)
 *   Y                          - flywheel on / off
 */
@TeleOp(name = "BioBuzz Unified TeleOp (Red)", group = "BioBuzz")
public class BioBuzzUnifiedTeleOp extends LinearOpMode {

    // ======================= DRIVE =======================
    private static final Pose START = AxonTurretAimOdo.START;
    private static final double SLOW_SCALE = 0.4;

    // ======================= TURRET TARGET: center of the red HIVE =======================
    private static final double HIVE_X = HiveTagGeometry.RED_HIVE_X_IN;
    private static final double HIVE_Y = HiveTagGeometry.PIVOT_Y_IN;

    // ======================= INTAKE / TRANSFER =======================
    private static final double INTAKE_POWER = 1.0;
    private static final double INTAKE_TRANSFER_POWER = -0.25; // while intaking: hold balls back
    private static final double SHOOT_TRANSFER_POWER = 0.6;
    private static final double TRIGGER_PRESSED = 0.3;

    // ======================= FLYWHEEL (ticks/s) =======================
    private static final double DEFAULT_VELOCITY = 1650;
    private static final double VELOCITY_STEP = 50;
    private static final double MAX_VELOCITY = 2600; // 6000 rpm motor, 28 ticks/rev -> ~2800 free speed

    private DcMotor intake, transfer;
    private DcMotorEx flywheel;

    @Override
    public void runOpMode() {
        // ---- hardware ----
        intake = hardwareMap.dcMotor.get("intakeMotor");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transfer = hardwareMap.dcMotor.get("transferMotor");
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheel = hardwareMap.get(DcMotorEx.class, LauncherConstants.MOTOR_NAME);
        flywheel.setDirection(LauncherConstants.MOTOR_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // coast down
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                LauncherConstants.VEL_P, LauncherConstants.VEL_I, LauncherConstants.VEL_D, LauncherConstants.VEL_F));
        flywheel.setVelocity(0);

        TurretController turret = new TurretController(hardwareMap); // zeroes the turret encoder: must face FRONT

        telemetry.addLine("Calibrating Pinpoint - keep the robot STILL");
        telemetry.update();
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(START);

        while (opModeInInit()) {
            follower.update();
            Pose pose = follower.pose();
            turret.updateHeading(pose.heading());
            turret.stop();
            telemetry.addLine("Robot on the start spot (29.0, 133.7) facing the audience? Turret facing FRONT?");
            telemetry.addData("Pose", "(%.1f, %.1f) %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            telemetry.addData("Turret", "%.1f°  (limits %.0f .. %.0f)", turret.angleDeg(), TurretConstants.MIN_DEG, TurretConstants.MAX_DEG);
            telemetry.update();
        }

        boolean autoAim = true;
        boolean flywheelOn = false;
        double targetVelocity = DEFAULT_VELOCITY;

        while (opModeIsActive()) {
            // ---- drive ----
            double s = gamepad1.right_bumper ? SLOW_SCALE : 1.0;
            follower.manual(-gamepad1.left_stick_y * s, -gamepad1.left_stick_x * s, -gamepad1.right_stick_x * s);
            follower.update();
            Pose pose = follower.pose();
            turret.updateHeading(pose.heading());

            if (gamepad1.bWasPressed()) {
                follower.setPose(START);
                turret.resetHeadingTracking();
                pose = follower.pose();
            }

            // ---- turret ----
            if (gamepad1.aWasPressed()) autoAim = !autoAim;
            double turretTarget = Double.NaN;
            if (autoAim) {
                turretTarget = turret.aimAt(pose.x(), pose.y(), pose.heading(), HIVE_X, HIVE_Y);
            } else {
                double manual = (gamepad1.dpad_left ? 1 : 0) - (gamepad1.dpad_right ? 1 : 0); // left = +
                turret.manual(manual);
            }

            // ---- flywheel ----
            if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) flywheelOn = !flywheelOn;
            if (gamepad2.dpadUpWasPressed()) targetVelocity += VELOCITY_STEP;
            if (gamepad2.dpadDownWasPressed()) targetVelocity -= VELOCITY_STEP;
            targetVelocity = Range.clip(targetVelocity, 0, MAX_VELOCITY);
            flywheel.setVelocity(flywheelOn ? targetVelocity : 0);
            double velocity = flywheel.getVelocity();
            boolean atSpeed = flywheelOn
                    && Math.abs(targetVelocity - velocity) <= LauncherConstants.VELOCITY_TOLERANCE * targetVelocity;

            // ---- intake / shoot ----
            String feed;
            if (gamepad1.x) {
                setFeed(-INTAKE_POWER, -SHOOT_TRANSFER_POWER);
                feed = "UNJAM";
            } else if (gamepad1.right_trigger >= TRIGGER_PRESSED) {
                setFeed(INTAKE_POWER, SHOOT_TRANSFER_POWER);
                feed = flywheelOn ? "SHOOTING" : "SHOOTING - flywheel is OFF (Y)";
            } else if (gamepad1.left_trigger >= TRIGGER_PRESSED) {
                setFeed(INTAKE_POWER, INTAKE_TRANSFER_POWER);
                feed = "INTAKING";
            } else {
                setFeed(0, 0);
                feed = "idle";
            }

            // ---- telemetry ----
            telemetry.addData("Flywheel (Y)", !flywheelOn ? "OFF" : atSpeed ? "READY" : "spinning up");
            telemetry.addData("Flywheel speed", "%.0f / %.0f ticks/s  (gp2 dpad)", velocity, targetVelocity);
            telemetry.addData("Feed", feed);
            telemetry.addData("Turret (A)", autoAim ? "AUTO AIM" : "MANUAL (dpad L/R)");
            if (!Double.isNaN(turretTarget)) {
                telemetry.addData("Turret", "%.1f° -> %.1f°", turret.angleDeg(), turretTarget);
            } else {
                telemetry.addData("Turret", "%.1f°", turret.angleDeg());
            }
            telemetry.addData("Pose", "(%.1f, %.1f) %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            telemetry.addData("Hive distance", "%.1f in", Math.hypot(HIVE_X - pose.x(), HIVE_Y - pose.y()));
            telemetry.update();
        }

        flywheel.setVelocity(0);
        setFeed(0, 0);
        turret.stop();
    }

    private void setFeed(double intakePower, double transferPower) {
        intake.setPower(intakePower);
        transfer.setPower(transferPower);
    }
}
