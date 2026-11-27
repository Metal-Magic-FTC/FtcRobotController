package org.firstinspires.ftc.teamcode.biobuzz.comp1.teleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher.LauncherConstants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher.advanced.Flywheel;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.AxonTurretAimOdo;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.RedHiveGoal;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.TurretController;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry;
import org.firstinspires.ftc.teamcode.biobuzz.util.BulkReader;

/**
 * RED TeleOp, everything together: drive, motor turret (TurretController) auto-aiming at the CENTER of the red HIVE (odometry only),
 * tuned flywheel, intake and shoot.
 *
 * BEFORE PRESSING INIT: robot on the Auto V4 start spot (29.0, 133.7) facing the audience, and STILL while the
 * Pinpoint calibrates. TURRET POINTING STRAIGHT OUT THE ROBOT FRONT: its encoder is zeroed at INIT (that is 0 deg).
 *
 * gamepad1 (driver)
 *   left stick / right stick X - drive (robot-centric) / turn      right bumper - slow
 *   left bumper (hold)         - flower ramp DOWN (otherwise UP the whole match)
 *   dpad up                    - transfer shoot speed: slow (default) <-> full
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
    /** Aims at the red HIVE cell on the robot's half of the field (RedHiveGoal: (57.8, 54.2) / (57.8, 89.8)). */
    private final RedHiveGoal goal = new RedHiveGoal(AxonTurretAimOdo.START.y());

    // ======================= INTAKE / TRANSFER =======================
    private static final double INTAKE_POWER = 1.0;
    private static final double INTAKE_TRANSFER_POWER = -0.25; // while intaking: hold balls back
    /** Transfer while shooting. gamepad1 dpad up toggles SLOW (default) / FULL. */
    private static final double SHOOT_TRANSFER_SLOW = 0.6;  // the slower option (gamepad1 dpad up), was 0.7
    private static final double SHOOT_TRANSFER_FULL = 0.8;  // default (was 0.7)
    private boolean fullTransfer = true; // starts at FULL (1.0); gamepad1 dpad up toggles to 0.7
    private static final double TRIGGER_PRESSED = 0.3;

    // ======================= FLYWHEEL (ticks/s) =======================
    private static final double DEFAULT_VELOCITY = 1600; // +3% (was 1555)
    private static final double VELOCITY_STEP = 50;
    private static final double MAX_VELOCITY = 2600; // 6000 rpm motor, 28 ticks/rev -> ~2800 free speed

    private DcMotor intake, transfer;
    /** Flower ramp servo ("flowerServo"): UP all match, DOWN only while LEFT BUMPER is held. Measured positions. */
    private static final double RAMP_UP = 0.730;
    private static final double RAMP_DOWN = 0.140;
    private Servo flowerServo;
    /** launcher/advanced/Flywheel: kS + kV + kP, voltage compensation, full-power boost after each ball. */
    private final Flywheel flywheel = new Flywheel();

    @Override
    public void runOpMode() {
        // ---- hardware ----
        intake = hardwareMap.dcMotor.get("intakeMotor");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transfer = hardwareMap.dcMotor.get("transferMotor");
        flowerServo = hardwareMap.get(Servo.class, "flowerServo"); // not moved until START
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheel.init(hardwareMap); // "launchMotor", direction from LauncherConstants
        flywheel.stopMotors();

        TurretController turret = new TurretController(hardwareMap); // zeroes the turret encoder: must face FRONT

        telemetry.addLine("Calibrating Pinpoint - keep the robot STILL");
        telemetry.update();
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(START);
        // AFTER Constants.create and flywheel.init: both switch the hubs to AUTO caching, this switches them to MANUAL.
        BulkReader bulk = new BulkReader(hardwareMap);

        while (opModeInInit()) {
            bulk.update();
            follower.update();
            Pose pose = follower.pose();
            turret.updateHeading(pose.heading());
            turret.stop();
            telemetry.addData("TURRET ENCODER", "%d ticks  (%.1f°)   limits %d .. %d ticks", turret.ticks(), turret.angleDeg(),
                    Math.round(TurretConstants.HARD_MIN_DEG / 360.0 * TurretConstants.TICKS_PER_TURRET_REV),
                    Math.round(TurretConstants.HARD_MAX_DEG / 360.0 * TurretConstants.TICKS_PER_TURRET_REV));
            telemetry.addData("ODO POSE", "x %.1f  y %.1f  heading %.1f°", pose.x(), pose.y(),
                    Math.toDegrees(pose.heading()));
            telemetry.addLine("Robot on the start spot (29.0, 133.7) facing the audience? Turret facing FRONT?");
            telemetry.addData("Pose", "(%.1f, %.1f) %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            telemetry.addData("Turret", "%.1f°  (limits %.0f .. %.0f)", turret.angleDeg(), TurretConstants.MIN_DEG, TurretConstants.MAX_DEG);
            telemetry.update();
        }

        boolean autoAim = true;
        boolean flywheelOn = false;
        double targetVelocity = DEFAULT_VELOCITY;

        while (opModeIsActive()) {
            bulk.update(); // FIRST: one fresh read of every hub's encoders / velocities this loop

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
                goal.update(pose.y());
                turretTarget = turret.aimAt(pose.x(), pose.y(), pose.heading(), goal.x(), goal.y());
            } else {
                double manual = (gamepad1.dpad_left ? 1 : 0) - (gamepad1.dpad_right ? 1 : 0); // left = +
                turret.manual(manual);
            }

            // ---- flower ramp: hold left bumper = down, otherwise up ----
            flowerServo.setPosition(gamepad1.left_bumper ? RAMP_DOWN : RAMP_UP);

            // ---- flywheel ----
            if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) flywheelOn = !flywheelOn;
            if (gamepad2.dpadUpWasPressed()) targetVelocity += VELOCITY_STEP;
            if (gamepad2.dpadDownWasPressed()) targetVelocity -= VELOCITY_STEP;
            targetVelocity = Range.clip(targetVelocity, 0, MAX_VELOCITY);
            // target stays in ticks/s (same numbers as before), converted to RPM for Flywheel; runs every loop
            if (flywheelOn) flywheel.setMotorRPM(Flywheel.ticksPerSecToRpm(targetVelocity));
            else flywheel.stopMotors();
            double velocity = flywheel.getTicksPerSec();
            boolean atSpeed = flywheelOn
                    && Math.abs(targetVelocity - velocity) <= LauncherConstants.VELOCITY_TOLERANCE * targetVelocity;

            // ---- intake / shoot ----
            if (gamepad1.dpadUpWasPressed()) fullTransfer = !fullTransfer;
            double shootTransfer = fullTransfer ? SHOOT_TRANSFER_FULL : SHOOT_TRANSFER_SLOW;
            String feed;
            if (gamepad1.x) {
                setFeed(-INTAKE_POWER, -shootTransfer);
                feed = "UNJAM";
            } else if (gamepad1.right_trigger >= TRIGGER_PRESSED) {
                setFeed(INTAKE_POWER, shootTransfer);
                feed = flywheelOn ? "SHOOTING" : "SHOOTING - flywheel is OFF (Y)";
            } else if (gamepad1.left_trigger >= TRIGGER_PRESSED) {
                setFeed(INTAKE_POWER, INTAKE_TRANSFER_POWER);
                feed = "INTAKING";
            } else {
                setFeed(0, 0);
                feed = "idle";
            }

            // ---- telemetry ---- (VERY TOP: raw turret motor encoder)
            telemetry.addData("TURRET ENCODER", "%d ticks  (%.1f°)   limits %d .. %d ticks", turret.ticks(), turret.angleDeg(),
                    Math.round(TurretConstants.HARD_MIN_DEG / 360.0 * TurretConstants.TICKS_PER_TURRET_REV),
                    Math.round(TurretConstants.HARD_MAX_DEG / 360.0 * TurretConstants.TICKS_PER_TURRET_REV));
            telemetry.addData("ODO POSE", "x %.1f  y %.1f  heading %.1f°", pose.x(), pose.y(),
                    Math.toDegrees(pose.heading()));
            telemetry.addData("Transfer speed (dpad up)", fullTransfer ? "FULL" : "slow");
            telemetry.addData("Flower ramp (LB)", gamepad1.left_bumper ? "DOWN" : "up");
            telemetry.addData("Flywheel (Y)", !flywheelOn ? "OFF" : flywheel.isBoosting() ? "BOOST"
                    : atSpeed ? "READY" : "spinning up");
            telemetry.addData("Flywheel speed", "%.0f / %.0f ticks/s  (gp2 dpad)", velocity, targetVelocity);
            telemetry.addData("Feed", feed);
            telemetry.addData("Turret (A)", autoAim ? "AUTO AIM" : "MANUAL (dpad L/R)");
            if (!Double.isNaN(turretTarget)) {
                telemetry.addData("Turret", "%.1f° -> %.1f°", turret.angleDeg(), turretTarget);
            } else {
                telemetry.addData("Turret", "%.1f°", turret.angleDeg());
            }
            telemetry.addData("Pose", "(%.1f, %.1f) %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            telemetry.addData("Hive target", "%s cell (%.1f, %.1f), %.1f in away", goal.aimingTop() ? "TOP" : "BOTTOM",
                    goal.x(), goal.y(), Math.hypot(goal.x() - pose.x(), goal.y() - pose.y()));
            telemetry.update();
        }

        flywheel.stopMotors();
        setFeed(0, 0);
        turret.stop();
    }

    private void setFeed(double intakePower, double transferPower) {
        intake.setPower(intakePower);
        transfer.setPower(transferPower);
    }
}
