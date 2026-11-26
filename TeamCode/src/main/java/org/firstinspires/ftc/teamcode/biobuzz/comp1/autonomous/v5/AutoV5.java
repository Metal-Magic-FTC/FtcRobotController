package org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.v5;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher.FlywheelSpeedTable;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher.advanced.Flywheel;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.RedHiveGoal;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.TurretController;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.Hive;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.TagObservation;

/**
 * Auto V5 (RED), the full auto: paths from "yeehw.pp (1).json" (PathsV5) with intake, transfer, launcher and turret.
 *
 * The whole time: intake ON, flower ramp DOWN, launcher spinning at the speed for the current distance (same table as
 * the TeleOp), turret aiming at the red HIVE cell on the robot's half of the field (odometry).
 *
 *   1. Ramp down, wait for the red HIVE to tip (Limelight sees a red tag)
 *   2. Shoot the 4 preloads
 *   3. Path 1 -> far wall FLOWER
 *   4. Intake from the FLOWER
 *   5. Shoot those 4
 *   6. Path 2 -> red wall FLOWER
 *   7. Intake from that FLOWER
 *   8. Path 3 -> shooting spot
 *   9. Shoot
 *  10. Path 4 -> corner, intaking on the way (+ a short hold in the corner)
 *  11. Shoot from the corner
 *  12. Path 5 -> park, intake off
 *
 * BEFORE PRESSING INIT
 *   1. Robot at PathsV5.start (29.0, 133.7) facing the audience (heading 270), STILL while the Pinpoint calibrates.
 *   2. TURRET POINTING STRAIGHT OUT THE ROBOT FRONT (its encoder is zeroed at INIT).
 *   3. 4 preloads loaded. Nothing moves until START.
 */
@Autonomous(name = "BioBuzz Auto V5 (full)", group = "BioBuzz")
public class AutoV5 extends LinearOpMode {

    // ======================= TIMING (tune) =======================
    /** Wait this long for the HIVE to tip, then go anyway. POSITIVE_INFINITY = wait as long as it takes. */
    private static final double TIP_TIMEOUT_SEC = Double.POSITIVE_INFINITY;
    private static final int RED_TAG_CONFIRM_FRAMES = 5;
    /** Before feeding: wait (up to this) for the launcher to be at speed and the turret on target. */
    private static final double READY_TIMEOUT_SEC = 1.0;
    /** How long the transfer feeds per shot (long enough for all 4 balls). */
    private static final double SHOOT_SEC = 2.0;
    /** Parked at a FLOWER pulling balls out. */
    private static final double FLOWER_INTAKE_SEC = 1.5;
    /** Extra time sitting in the corner after path 4 to finish grabbing balls. */
    private static final double CORNER_INTAKE_SEC = 0.5;
    /** Give up on a path after this long so a stuck robot still runs the rest of the auto. */
    private static final double PATH_TIMEOUT_SEC = 6.0;

    // ======================= MECHANISMS (same as the TeleOp) =======================
    private static final double INTAKE_POWER = 1.0;          // + = in
    private static final double TRANSFER_FEED_POWER = -1.0;  // - = up into the launcher
    private static final double TRANSFER_DOWN_POWER = 0.5;   // down, away from the launcher, when not shooting
    private static final double RAMP_DOWN = 0.140;
    /** Ready to feed when the launcher is within this of its target and the turret within TURRET_READY_DEG. */
    private static final double AT_SPEED_FRACTION = 0.03;
    private static final double TURRET_READY_DEG = 3.0;

    private Follower follower;
    private TurretController turret;
    private Flywheel launcher;
    private DcMotor intake, transfer;
    private Servo flowerServo;
    private Limelight3A limelight;
    private RedHiveGoal goal;

    private boolean intakeOn = true;
    private boolean feeding = false;
    private boolean mechanismsOn = false;
    private double launchTps = FlywheelSpeedTable.DEFAULT_TPS;
    private double turretTarget = Double.NaN;
    private String step = "init";

    @Override
    public void runOpMode() {
        turret = new TurretController(hardwareMap); // zeroes the turret encoder: must face FRONT
        follower = Constants.create(hardwareMap);
        follower.setPose(PathsV5.start);
        goal = new RedHiveGoal(PathsV5.start.y());

        launcher = new Flywheel();
        launcher.init(hardwareMap);
        intake = hardwareMap.dcMotor.get("intakeMotor");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer = hardwareMap.dcMotor.get("transferMotor");
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        flowerServo = hardwareMap.get(Servo.class, "flowerServo"); // not moved until START

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(HiveTagGeometry.APRILTAG_PIPELINE);
        limelight.start();

        // Build the paths during INIT so START is instant
        PathsV5 paths = new PathsV5();
        Path path1 = paths.path1(), path2 = paths.path2(), path3 = paths.path3(),
                path4 = paths.path4(), path5 = paths.path5();

        // INIT: odometry only (nothing moves). Check the start pose on telemetry.
        while (opModeInInit()) {
            follower.update();
            Pose p = follower.pose();
            telemetry.addLine("Auto V5 ready. Turret facing robot FRONT? 4 preloads in?");
            telemetry.addData("Pose", "(%.1f, %.1f) %.1f°  (should be 29.0, 133.7, 270)",
                    p.x(), p.y(), Math.toDegrees(p.heading()));
            telemetry.update();
        }
        if (isStopRequested()) {
            limelight.stop();
            return;
        }

        // START: everything on
        mechanismsOn = true;
        flowerServo.setPosition(RAMP_DOWN);                              // 1. ramp down right away
        intakeOn = true;

        waitForRedHive();                                                // 1. wait for the tip
        limelight.stop();
        shoot("2. Shooting preloads");                                   // 2.

        runPath(path1, "3. Path 1 -> far wall FLOWER");                  // 3.
        holdFor(FLOWER_INTAKE_SEC, "4. Intaking from FLOWER");           // 4.
        shoot("5. Shooting FLOWER balls");                               // 5.

        runPath(path2, "6. Path 2 -> red wall FLOWER");                  // 6.
        holdFor(FLOWER_INTAKE_SEC, "7. Intaking from FLOWER");           // 7.
        runPath(path3, "8. Path 3 -> shooting spot");                    // 8.
        shoot("9. Shooting");                                            // 9.

        runPath(path4, "10. Path 4 -> corner (intaking)");               // 10.
        holdFor(CORNER_INTAKE_SEC, "10. Intaking in corner");
        shoot("11. Shooting from corner");                               // 11.

        intakeOn = false;                                                // 12.
        runPath(path5, "12. Path 5 -> park");

        // Done: hold the park spot, everything else off
        mechanismsOn = false;
        step = "Done - parked";
        while (opModeIsActive()) {
            updateRobot();
            telemetry.update();
        }
        stopAll();
    }

    // ======================= STEPS =======================

    /** From the far wall the red tags face away until the HIVE tips, so the first red tag seen = it came down. */
    private void waitForRedHive() {
        ElapsedTime timer = new ElapsedTime();
        int frames = 0;
        while (opModeIsActive() && frames < RED_TAG_CONFIRM_FRAMES && timer.seconds() < TIP_TIMEOUT_SEC) {
            frames = seesRedTag() ? frames + 1 : 0;
            step = String.format("1. Waiting for red HIVE to tip (%.1f s, red tag %d/%d)",
                    timer.seconds(), frames, RED_TAG_CONFIRM_FRAMES);
            updateRobot();
            telemetry.update();
        }
    }

    private boolean seesRedTag() {
        for (TagObservation tag : HiveTagGeometry.readTags(limelight.getLatestResult())) {
            if (tag.info.hive == Hive.RED) return true;
        }
        return false;
    }

    /** Wait (briefly) until the launcher is at speed and the turret is on target, then feed for SHOOT_SEC. */
    private void shoot(String status) {
        ElapsedTime timer = new ElapsedTime();
        step = status + " (getting ready)";
        while (opModeIsActive() && timer.seconds() < READY_TIMEOUT_SEC && !readyToShoot()) {
            updateRobot();
            telemetry.update();
        }
        feeding = true;
        step = status;
        timer.reset();
        while (opModeIsActive() && timer.seconds() < SHOOT_SEC) {
            updateRobot();
            telemetry.update();
        }
        feeding = false;
    }

    private boolean readyToShoot() {
        boolean atSpeed = launcher.getTicksPerSec() >= launchTps * (1 - AT_SPEED_FRACTION);
        boolean aimed = !Double.isNaN(turretTarget) && Math.abs(turretTarget - turret.angleDeg()) <= TURRET_READY_DEG;
        return atSpeed && aimed;
    }

    private void runPath(Path path, String status) {
        step = status;
        follower.follow(path);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && follower.isBusy() && timer.seconds() < PATH_TIMEOUT_SEC) {
            updateRobot();
            telemetry.update();
        }
    }

    /** Holds the end of the last path (the follower keeps holding) while everything keeps running. */
    private void holdFor(double seconds, String status) {
        step = status;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            updateRobot();
            telemetry.update();
        }
    }

    // ======================= EVERY LOOP =======================

    /** Drive, aim the turret, run the launcher / intake / transfer. Call once per loop. */
    private void updateRobot() {
        follower.update();
        Pose pose = follower.pose();
        turret.updateHeading(pose.heading());
        goal.update(pose.y());

        if (!mechanismsOn) {
            turret.stop();
            launcher.stopMotors();
            intake.setPower(0);
            transfer.setPower(0);
            addTelemetry(pose);
            return;
        }

        turretTarget = turret.aimAt(pose.x(), pose.y(), pose.heading(), goal.x(), goal.y());

        // Launcher speed from the turret-to-cell distance (same table as the TeleOp)
        double h = pose.heading();
        double tx = pose.x() + TurretConstants.TURRET_FORWARD_IN * Math.cos(h) - TurretConstants.TURRET_LEFT_IN * Math.sin(h);
        double ty = pose.y() + TurretConstants.TURRET_FORWARD_IN * Math.sin(h) + TurretConstants.TURRET_LEFT_IN * Math.cos(h);
        launchTps = FlywheelSpeedTable.velocityFor(Math.hypot(goal.x() - tx, goal.y() - ty));
        launcher.setMotorRPM(Flywheel.ticksPerSecToRpm(launchTps)); // must run every loop

        intake.setPower(intakeOn ? INTAKE_POWER : 0);
        transfer.setPower(feeding ? TRANSFER_FEED_POWER : TRANSFER_DOWN_POWER);
        addTelemetry(pose);
    }

    private void stopAll() {
        turret.stop();
        launcher.stopMotors();
        intake.setPower(0);
        transfer.setPower(0);
        limelight.stop();
    }

    private void addTelemetry(Pose pose) {
        telemetry.addData("Step", step);
        telemetry.addData("Pose", "(%.1f, %.1f) %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Aiming at", "%s cell (%.1f, %.1f)", goal.aimingTop() ? "TOP" : "BOTTOM", goal.x(), goal.y());
        telemetry.addData("Turret", "%.1f° -> %s", turret.angleDeg(),
                Double.isNaN(turretTarget) ? "-" : String.format("%.1f°", turretTarget));
        telemetry.addData("Launcher", "%.0f / %.0f ticks/s%s", launcher.getTicksPerSec(), launchTps,
                launcher.isBoosting() ? "  BOOST" : "");
        telemetry.addData("Intake / transfer", "%s / %s", intakeOn ? "IN" : "off", feeding ? "FEEDING" : "down");
    }
}
