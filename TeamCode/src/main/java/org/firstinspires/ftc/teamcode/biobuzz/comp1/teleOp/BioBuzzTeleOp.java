package org.firstinspires.ftc.teamcode.biobuzz.comp1.teleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher.FlywheelSpeedTable;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher.LauncherConstants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher.advanced.Flywheel;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.RedHiveGoal;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.TurretController;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.Hive;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.HiveTilt;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.TagObservation;
import org.firstinspires.ftc.teamcode.biobuzz.util.BulkReader;

import java.util.ArrayDeque;
import java.util.List;

/**
 * RED TeleOp: drive anywhere, the Limelight finds the robot on the field from the AprilTags under the HIVE cells
 * (accounting for which way each HIVE is tipped), Pedro keeps tracking it with the Pinpoint, and the turret aims at
 * the red HIVE (bottom cell in the bottom half of the field, top cell in the top half).
 *
 * LOCALIZATION
 *   1. Start anywhere. Until the robot is located the turret holds still and telemetry says "SEARCHING".
 *   2. Point the Limelight at a HIVE so it sees at least 2 tags on the same CELL. Their direction gives the
 *      robot heading; their 3D positions (placed on the field using the HIVE tilt) give X/Y. After LOCK_SAMPLES
 *      consistent frames the Pedro pose is set and the robot is "LOCALIZED".
 *   3. From then on the Pinpoint tracks the robot and every tag sighting nudges X/Y (and slowly heading) back
 *      toward vision, so drift is cancelled whenever the HIVE is in view.
 *
 * BEFORE PRESSING INIT: turret pointing straight out the robot FRONT (turret 0 deg). Camera mount numbers are in
 * HiveTagGeometry and must be measured.
 *
 * LAUNCHER: the 72 mm wheel ("launchMotor") runs the WHOLE match at a speed picked from the turret-to-cell distance
 * (FlywheelSpeedTable). Before the robot is localized it runs the measured wall-shot speed.
 * TRANSFER: spins DOWN the whole time, except while the shoot button is held, then it feeds UP into the launcher.
 *
 * gamepad1:
 *   left stick      - drive (robot-centric), right stick X - turn, right bumper - slow mode
 *   LEFT TRIGGER    - SHOOT (hold): transfer feeds up into the launcher
 *   RIGHT TRIGGER   - intake on / off (toggle)
 *   Y               - flip intake direction
 *   LEFT BUMPER     - hold: flower ramp DOWN (otherwise UP the whole match)
 *   DPAD UP         - transfer feed speed: slow (default) <-> full
 *   B / A / X       - launcher FIXED speed 1510 / 1560 / 1460 (press the same one again = back to auto distance)
 *   dpad left/right - manual turret while auto aim is off (left = counter-clockwise)
 *
 * gamepad2:
 *   A               - toggle turret auto aim (on at start)
 *   B               - forget the pose and re-localize from the tags
 *   X               - toggle continuous vision correction (on at start)
 *   dpad up / down  - launcher speed trim +/- 2% (all speeds)
 *   TURRET FAILSAFE:
 *   left stick X    - MANUAL OVERRIDE while held: turns the turret (left = counter-clockwise), beats auto aim.
 *                     Limits still apply.
 *   Y               - turret to 0 (straight out the FRONT) and hold. A = back to auto aim.
 *   BACK            - "the turret points at the FRONT right now": makes this 0 deg (fixes a slipped encoder),
 *                     then holds there. Point it at the front first (left stick), then press BACK.
 */
@TeleOp(name = "BioBuzz TeleOp (Red)", group = "BioBuzz")
public class BioBuzzTeleOp extends LinearOpMode {

    // ======================= VISION LOCALIZATION (tune) =======================
    private static final long MAX_STALENESS_MS = 100;
    /** Ignore tags farther than this (3D solve gets noisy). */
    private static final double MAX_TAG_RANGE_IN = 110.0;
    /** First fix, step 1: frames with a heading (2+ tags on one cell) that must agree within HEADING_MAX_SPREAD_DEG. */
    private static final int HEADING_SAMPLES = 5;
    private static final double HEADING_MAX_SPREAD_DEG = 5.0;
    /** First fix, step 2: frames (any tags) whose position must agree within LOCK_MAX_SPREAD_IN. */
    private static final int LOCK_SAMPLES = 8;
    private static final double LOCK_MAX_SPREAD_IN = 4.0;
    /** Lock frames older than this are dropped. */
    private static final double LOCK_WINDOW_SEC = 2.0;
    /** Fraction of the vision error applied per frame once localized (X/Y and heading). */
    private static final double XY_GAIN = 0.2;
    private static final double HEADING_GAIN = 0.05;
    /** A sighting further than this from the current pose is ignored ... */
    private static final double MAX_JUMP_IN = 18.0;
    /** ... unless it keeps happening this many frames in a row (then re-localize). */
    private static final int RELOCK_AFTER_FRAMES = 15;
    /** No vision while spinning faster than this (motion blur). */
    private static final double MAX_TURN_RATE_FOR_VISION_DEG = 120.0;
    private static final double POSE_HISTORY_SEC = 1.0;

    private static final double SLOW_MODE_SCALE = 0.4;

    // ======================= TURRET FAILSAFE (gamepad2) =======================
    /** gamepad2 left stick X beyond this = manual turret override. */
    private static final double TURRET_OVERRIDE_DEADBAND = 0.15;
    /** gamepad2 Y / BACK: hold the turret at 0 deg until A turns auto aim back on. */
    private boolean turretHome = false;
    private int rezeroCount = 0;

    // ======================= TURN FEEL =======================
    /**
     * Turn output = TURN_MAX * stick^TURN_POWER (after the deadband). Higher TURN_POWER = much gentler for small
     * stick pushes, while full stick still reaches TURN_MAX. 1 = straight line (old feel).
     * With 3.0 / 0.9:  25% stick -> 1%,  50% -> 10%,  75% -> 36%,  100% -> 90% turn speed.
     */
    private static final double TURN_POWER = 3.0;
    private static final double TURN_MAX = 0.9;
    /** Right-stick wobble near center ignored (no jump at its edge). */
    private static final double TURN_DEADBAND = 0.05;

    // ======================= DRIVE FEEL =======================
    /**
     * Drive output = DRIVE_EXPO * t^3 + (1 - DRIVE_EXPO) * t, on how far the left stick is pushed (any direction,
     * so diagonals keep their direction). 0 = straight line, 1 = pure cubic. Full stick is still full speed.
     * With 0.4:  25% stick -> 13%,  50% -> 33%,  75% -> 60%,  100% -> 100%.
     */
    private static final double DRIVE_EXPO = 0.4;
    private static final double DRIVE_DEADBAND = 0.05;

    // ======================= INTAKE / TRANSFER / LAUNCHER =======================
    /** Signs from FullLaunch: intake + = in; transfer - = up into the launcher (FullLaunch needed X for this). */
    private static final double INTAKE_POWER = 1.0;
    /** Feed up into the launcher while shooting. gamepad1 dpad up toggles SLOW (default) / FULL. */
    private static final double TRANSFER_FEED_SLOW = -0.6;  // the slower option (gamepad1 dpad up), was 0.7
    private static final double TRANSFER_FEED_FULL = -0.8;  // default, was 1.0
    private boolean fullTransfer = true; // starts at 0.8; gamepad1 dpad up toggles to 0.6
    private double lowestWhileShootingTps = Double.NaN;
    private boolean wasShooting = false;
    private static final double TRANSFER_DOWN_POWER = 0.5;  // down / away from the launcher (the rest of the time)
    // Launcher control: launcher/advanced/Flywheel (kS + kV + kP, voltage compensation, full-power boost after each
    // ball). The speeds below stay in ticks/s (same numbers as before) and are converted to RPM for Flywheel.
    private static final double TRIGGER_DOWN = 0.5;

    private DcMotor intake, transfer;
    /** Flower ramp servo ("flowerServo"): UP all match, DOWN only while LEFT BUMPER is held. Measured positions. */
    private static final double RAMP_UP = 0.730;
    private static final double RAMP_DOWN = 0.140;
    private Servo flowerServo;
    private final Flywheel launcher = new Flywheel();
    private boolean launcherReady = false;
    private boolean intakeOn = false;
    private double intakeDirection = 1;
    private boolean rightTriggerWasDown = false;
    /** Fixed launcher speeds on gamepad1 B / A / X (ticks/s). NaN = auto speed from distance. */
    // was 1650 / 1700 / 1600 - shots went too far with the new Flywheel controller
    private static final double FIXED_B_TPS = 1510, FIXED_A_TPS = 1560, FIXED_X_TPS = 1460; // +4% (was 1450/1500/1400)
    /** Live speed trim on gamepad2 dpad up/down, applied to EVERY launcher speed (auto and presets). */
    private static final double TRIM_STEP = 0.02;
    private double speedTrim = 1.0;
    private double fixedTps = Double.NaN;

    private Follower follower;
    private Limelight3A limelight;
    private TurretController turret;
    private final RedHiveGoal goal = new RedHiveGoal(0);
    private final ElapsedTime clock = new ElapsedTime();

    /** {time sec, x, y, heading} so a sighting can be matched with the pose from when the photo was taken. */
    private final ArrayDeque<double[]> poseHistory = new ArrayDeque<>();
    /** Frames seen while searching. */
    private final ArrayDeque<LockFrame> lockFrames = new ArrayDeque<>();
    private String lockStatus = "no tags";

    /** One Limelight frame kept for the first fix. */
    private static class LockFrame {
        final double time;
        /** Odometry {time, x, y, heading} when the frame was captured. */
        final double[] then;
        final List<TagObservation> tags;
        final HiveTilt red, blue;
        /** Field heading minus odometry heading from this frame's tags, NaN if it had no usable cell. */
        final double dTheta;

        LockFrame(double time, double[] then, List<TagObservation> tags, HiveTilt red, HiveTilt blue, double dTheta) {
            this.time = time;
            this.then = then;
            this.tags = tags;
            this.red = red;
            this.blue = blue;
            this.dTheta = dTheta;
        }
    }

    private boolean localized = false;
    private boolean visionCorrection = true;
    private int outlierFrames = 0;

    // for telemetry
    private List<TagObservation> lastTags;
    private HiveTilt lastRedTilt, lastBlueTilt;
    private double lastVisionError = Double.NaN;
    @Override
    public void runOpMode() {
        turret = new TurretController(hardwareMap);
        initMechanisms();

        telemetry.addLine("Calibrating Pinpoint - keep the robot STILL");
        telemetry.update();
        follower = Constants.create(hardwareMap);
        // AFTER Constants.create and launcher.init: both switch the hubs to AUTO caching, this switches them to MANUAL.
        BulkReader bulk = new BulkReader(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(HiveTagGeometry.APRILTAG_PIPELINE);
        limelight.start();

        // Localize during INIT too, so the robot can already be located when START is pressed.
        while (opModeInInit()) {
            bulk.update();
            Pose pose = updatePose();
            updateVision(pose);
            telemetry.addLine("Turret facing robot FRONT? Point the Limelight at a HIVE to localize.");
            addTelemetry(follower.pose(), Double.NaN, true);
            telemetry.update();
        }

        boolean autoAim = true;
        while (opModeIsActive()) {
            bulk.update(); // FIRST: one fresh read of every hub's encoders / velocities this loop

            double scale = gamepad1.right_bumper ? SLOW_MODE_SCALE : 1.0;
            double fwd = -gamepad1.left_stick_y, strafe = -gamepad1.left_stick_x;
            double gain = driveGain(Math.hypot(fwd, strafe));
            follower.manual(fwd * gain * scale, strafe * gain * scale,
                    turnCurve(-gamepad1.right_stick_x) * scale);

            Pose pose = updatePose();
            updateVision(pose);
            pose = follower.pose();

            if (gamepad2.aWasPressed()) autoAim = !autoAim;
            if (gamepad2.xWasPressed()) visionCorrection = !visionCorrection;
            if (gamepad2.bWasPressed()) relocalize();
            // ---- TURRET FAILSAFE (gamepad2) ----
            if (gamepad2.yWasPressed()) {        // go to 0 (front) and hold; A resumes auto aim
                autoAim = false;
                turretHome = true;
            }
            boolean justRezeroed = false;
            if (gamepad2.backWasPressed()) {     // "it's pointing at the front NOW": this becomes 0 deg
                turret.rezero();
                justRezeroed = true;
                autoAim = false;
                turretHome = true;               // hold here (= the new 0) until A
                rezeroCount++;
            }
            if (autoAim || gamepad1.dpad_left || gamepad1.dpad_right) turretHome = false;
            double gp2Stick = -gamepad2.left_stick_x; // stick left = counter-clockwise (+)
            boolean override = Math.abs(gp2Stick) > TURRET_OVERRIDE_DEADBAND;

            double targetDeg = Double.NaN;
            if (justRezeroed) {
                // This loop's bulk-read angle is from BEFORE the reset: don't act on it. Fresh reading next loop.
                turret.stop();
            } else if (override) {
                // Manual override beats everything (auto aim, homing) while the stick is held. Limits still apply.
                turret.manual(gp2Stick);
                turretHome = false;
            } else if (turretHome) {
                turret.driveTo(0);
                targetDeg = 0;
            } else if (!autoAim) {
                turret.manual((gamepad1.dpad_left ? 1 : 0) - (gamepad1.dpad_right ? 1 : 0));
            } else if (localized) {
                goal.update(pose.y());
                targetDeg = turret.aimAt(pose.x(), pose.y(), pose.heading(), goal.x(), goal.y());
            } else {
                turret.stop(); // do not know where the goal is yet
            }

            updateMechanisms(pose);

            addTelemetry(pose, targetDeg, autoAim);
            telemetry.update();
        }

        turret.stop();
        launcher.stopMotors();
        intake.setPower(0);
        transfer.setPower(0);
        limelight.stop();
    }

    // ======================= INTAKE / TRANSFER / LAUNCHER =======================
    private void initMechanisms() {
        intake = hardwareMap.dcMotor.get("intakeMotor");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transfer = hardwareMap.dcMotor.get("transferMotor");
        flowerServo = hardwareMap.get(Servo.class, "flowerServo"); // not moved until START
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);

        launcher.init(hardwareMap); // "launchMotor", direction from LauncherConstants
        launcher.stopMotors();
        launcherReady = true;
    }

    private double shotDistanceIn = Double.NaN;
    private double launchTargetTps = FlywheelSpeedTable.DEFAULT_TPS;

    private void updateMechanisms(Pose pose) {
        // Flower ramp: hold left bumper = down, otherwise up
        flowerServo.setPosition(gamepad1.left_bumper ? RAMP_DOWN : RAMP_UP);

        // Intake: right trigger toggles on/off, Y flips direction
        boolean rightDown = gamepad1.right_trigger > TRIGGER_DOWN;
        if (rightDown && !rightTriggerWasDown) intakeOn = !intakeOn;
        rightTriggerWasDown = rightDown;
        if (gamepad1.yWasPressed()) intakeDirection = -intakeDirection;
        intake.setPower(intakeOn ? intakeDirection * INTAKE_POWER : 0);

        // Transfer: down all the time, up only while shooting
        boolean shooting = gamepad1.left_trigger > TRIGGER_DOWN;
        if (gamepad1.dpadUpWasPressed()) fullTransfer = !fullTransfer;
        double feedPower = fullTransfer ? TRANSFER_FEED_FULL : TRANSFER_FEED_SLOW;
        double wheel = launcher.getTicksPerSec();
        if (shooting && !wasShooting) lowestWhileShootingTps = Double.NaN; // new burst
        wasShooting = shooting;
        if (shooting) {
            lowestWhileShootingTps = Double.isNaN(lowestWhileShootingTps) ? wheel : Math.min(lowestWhileShootingTps, wheel);
            transfer.setPower(feedPower);
        } else {
            transfer.setPower(TRANSFER_DOWN_POWER);
        }

        // Launcher: speed from the turret-to-cell distance, the whole match
        if (localized) {
            goal.update(pose.y()); // keep the cell choice current even with auto aim off
            double h = pose.heading();
            double tx = pose.x() + TurretConstants.TURRET_FORWARD_IN * Math.cos(h) - TurretConstants.TURRET_LEFT_IN * Math.sin(h);
            double ty = pose.y() + TurretConstants.TURRET_FORWARD_IN * Math.sin(h) + TurretConstants.TURRET_LEFT_IN * Math.cos(h);
            shotDistanceIn = Math.hypot(goal.x() - tx, goal.y() - ty);
        } else {
            shotDistanceIn = Double.NaN;
        }
        if (gamepad1.bWasPressed()) fixedTps = fixedTps == FIXED_B_TPS ? Double.NaN : FIXED_B_TPS;
        if (gamepad1.aWasPressed()) fixedTps = fixedTps == FIXED_A_TPS ? Double.NaN : FIXED_A_TPS;
        if (gamepad1.xWasPressed()) fixedTps = fixedTps == FIXED_X_TPS ? Double.NaN : FIXED_X_TPS;
        if (gamepad2.dpadUpWasPressed()) speedTrim += TRIM_STEP;
        if (gamepad2.dpadDownWasPressed()) speedTrim = Math.max(0.5, speedTrim - TRIM_STEP);
        launchTargetTps = speedTrim * (Double.isNaN(fixedTps) ? FlywheelSpeedTable.velocityFor(shotDistanceIn) : fixedTps);
        launcher.setMotorRPM(Flywheel.ticksPerSecToRpm(launchTargetTps)); // must run every loop
    }

    private void addMechanismTelemetry() {
        double v = launcher.getTicksPerSec();
        boolean atSpeed = Math.abs(v - launchTargetTps) <= LauncherConstants.VELOCITY_TOLERANCE * launchTargetTps;
        telemetry.addLine();
        telemetry.addData("Launcher", "%s  target %.0f  actual %.0f ticks/s  (%s)",
                launcher.isBoosting() ? "BOOST" : atSpeed ? "READY" : "spinning up",
                launchTargetTps, v, Double.isNaN(fixedTps) ? "AUTO from distance" : "FIXED - press again for auto");
        telemetry.addData("Speed trim (gp2 dpad U/D)", "x%.2f", speedTrim);
        telemetry.addData("Launcher RPM", "%.0f / %.0f   power %.2f   battery %.1f V", launcher.getRPM(),
                Flywheel.ticksPerSecToRpm(launchTargetTps), launcher.lastPower(), launcher.batteryVoltage());
        telemetry.addData("Shot distance", Double.isNaN(shotDistanceIn) ? "not localized - using wall speed"
                : String.format("%.1f in", shotDistanceIn));
        telemetry.addData("Flower ramp (LB)", gamepad1.left_bumper ? "DOWN" : "up");
        telemetry.addData("Intake (RT / Y)", intakeOn ? (intakeDirection > 0 ? "IN" : "OUT") : "off");
        boolean shootingNow = gamepad1.left_trigger > TRIGGER_DOWN;
        telemetry.addData("Transfer (LT = shoot)", shootingNow ? "FEEDING UP" : "down");
        telemetry.addData("Transfer speed (gp1 dpad up)", fullTransfer ? "FULL" : "slow");
        telemetry.addData("Lowest wheel speed while shooting", Double.isNaN(lowestWhileShootingTps) ? "-"
                : String.format("%.0f ticks/s (%.0f%% of target)", lowestWhileShootingTps,
                100 * lowestWhileShootingTps / launchTargetTps));
    }

    // ======================= ODOMETRY =======================
    private Pose updatePose() {
        follower.update();
        Pose pose = follower.pose();
        double now = clock.seconds();
        poseHistory.addLast(new double[]{now, pose.x(), pose.y(), pose.heading()});
        while (now - poseHistory.peekFirst()[0] > POSE_HISTORY_SEC) poseHistory.removeFirst();
        turret.updateHeading(pose.heading());
        return pose;
    }

    /** Closest recorded pose to a past time: {time, x, y, heading}. */
    private double[] poseAt(double time) {
        double[] best = poseHistory.peekLast();
        for (double[] p : poseHistory) {
            if (Math.abs(p[0] - time) < Math.abs(best[0] - time)) best = p;
        }
        return best;
    }

    private void relocalize() {
        localized = false;
        lockFrames.clear();
        outlierFrames = 0;
    }

    // ======================= TURN FEEL =======================
    /** -1..1 stick -> -TURN_MAX..TURN_MAX: deadband removed smoothly, then stick^TURN_POWER (sign kept). */
    private static double turnCurve(double stick) {
        double a = Math.abs(stick);
        if (a < TURN_DEADBAND) return 0;
        double t = Math.min(1.0, (a - TURN_DEADBAND) / (1.0 - TURN_DEADBAND));
        return Math.signum(stick) * TURN_MAX * Math.pow(t, TURN_POWER);
    }

    /**
     * Multiplier for the left stick: curved speed / raw stick distance, so the stick's direction is kept and only its
     * size is curved. 0 inside the deadband.
     */
    private static double driveGain(double stickMag) {
        double m = Math.min(1.0, stickMag);
        if (m < DRIVE_DEADBAND) return 0;
        double t = (m - DRIVE_DEADBAND) / (1.0 - DRIVE_DEADBAND);
        double out = DRIVE_EXPO * t * t * t + (1.0 - DRIVE_EXPO) * t;
        return out / m;
    }

    // ======================= VISION =======================
    private void updateVision(Pose pose) {
        lastTags = null;
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getStaleness() > MAX_STALENESS_MS) return;
        if (Math.abs(turret.robotOmegaDeg()) > MAX_TURN_RATE_FOR_VISION_DEG) return;

        List<TagObservation> tags = HiveTagGeometry.readTags(result);
        tags.removeIf(t -> t.rangeIn > MAX_TAG_RANGE_IN);
        lastTags = tags;
        if (tags.isEmpty()) {
            lockStatus = "no tags";
            return;
        }
        lastRedTilt = HiveTagGeometry.measureHive(tags, Hive.RED);
        lastBlueTilt = HiveTagGeometry.measureHive(tags, Hive.BLUE);

        // Odometry pose when the frame was captured (the robot may have moved since)
        double ageSec = (result.getCaptureLatency() + result.getTargetingLatency() + result.getStaleness()) / 1000.0;
        double[] then = poseAt(clock.seconds() - ageSec);
        double visionHeading = HiveTagGeometry.headingFromTagPair(tags);

        if (!localized) {
            tryLock(tags, visionHeading, then);
        } else if (visionCorrection) {
            correct(tags, visionHeading, then, pose);
        }
    }

    /**
     * First fix. The robot heading is unknown, so: (1) average the heading from frames that see 2+ tags on a cell,
     * then (2) use that heading to place the robot from every frame and average the positions. Frames are stored as
     * "field = odometry rotated by dTheta, then shifted", so the robot may move while this happens.
     */
    private void tryLock(List<TagObservation> tags, double visionHeading, double[] then) {
        double now = clock.seconds();
        double dTheta = Double.isNaN(visionHeading) ? Double.NaN : HiveTagGeometry.normalizeAngle(visionHeading - then[3]);
        lockFrames.addLast(new LockFrame(now, then, tags, lastRedTilt, lastBlueTilt, dTheta));
        while (!lockFrames.isEmpty() && now - lockFrames.peekFirst().time > LOCK_WINDOW_SEC) lockFrames.removeFirst();

        // (1) heading offset: circular mean, then keep only frames close to it
        double sumSin = 0, sumCos = 0;
        int headingFrames = 0;
        for (LockFrame f : lockFrames) {
            if (Double.isNaN(f.dTheta)) continue;
            sumSin += Math.sin(f.dTheta);
            sumCos += Math.cos(f.dTheta);
            headingFrames++;
        }
        if (headingFrames < HEADING_SAMPLES) {
            lockStatus = String.format("heading %d/%d (need 2+ tags on one cell)", headingFrames, HEADING_SAMPLES);
            return;
        }
        double roughTheta = Math.atan2(sumSin, sumCos);
        sumSin = 0;
        sumCos = 0;
        int kept = 0;
        for (LockFrame f : lockFrames) {
            if (Double.isNaN(f.dTheta)) continue;
            if (Math.abs(HiveTagGeometry.normalizeAngle(f.dTheta - roughTheta)) > Math.toRadians(HEADING_MAX_SPREAD_DEG)) continue;
            sumSin += Math.sin(f.dTheta);
            sumCos += Math.cos(f.dTheta);
            kept++;
        }
        if (kept < HEADING_SAMPLES) {
            lockStatus = String.format("heading noisy (%d/%d agree)", kept, HEADING_SAMPLES);
            return;
        }
        double meanTheta = Math.atan2(sumSin, sumCos);
        double c = Math.cos(meanTheta), sn = Math.sin(meanTheta);

        // (2) translation from every frame using that heading
        double[][] shifts = new double[lockFrames.size()][];
        int n = 0;
        double sumX = 0, sumY = 0;
        for (LockFrame f : lockFrames) {
            double[] xy = estimateXY(f.tags, f.red, f.blue, f.then[3] + meanTheta);
            if (xy == null) continue;
            double tx = xy[0] - (f.then[1] * c - f.then[2] * sn);
            double ty = xy[1] - (f.then[1] * sn + f.then[2] * c);
            shifts[n++] = new double[]{tx, ty};
            sumX += tx;
            sumY += ty;
        }
        if (n < LOCK_SAMPLES) {
            lockStatus = String.format("position %d/%d", n, LOCK_SAMPLES);
            return;
        }
        double meanX = sumX / n, meanY = sumY / n;
        double keptX = 0, keptY = 0;
        int keptN = 0;
        for (int i = 0; i < n; i++) {
            if (Math.hypot(shifts[i][0] - meanX, shifts[i][1] - meanY) > LOCK_MAX_SPREAD_IN) continue;
            keptX += shifts[i][0];
            keptY += shifts[i][1];
            keptN++;
        }
        if (keptN < LOCK_SAMPLES) {
            lockStatus = String.format("position noisy (%d/%d agree)", keptN, LOCK_SAMPLES);
            return;
        }
        double tx = keptX / keptN, ty = keptY / keptN;

        // Apply the averaged transform to the CURRENT odometry pose
        Pose now0 = follower.pose();
        follower.setPose(new Pose(
                tx + now0.x() * c - now0.y() * sn,
                ty + now0.x() * sn + now0.y() * c,
                HiveTagGeometry.normalizeAngle(now0.heading() + meanTheta)));
        poseHistory.clear();
        lockFrames.clear();
        localized = true;
        turret.resetHeadingTracking(); // the heading jumped - don't read it as the robot spinning
        outlierFrames = 0;
    }

    /** Once localized: nudge the pose toward vision, measured against where the robot was when the photo was taken. */
    private void correct(List<TagObservation> tags, double visionHeading, double[] then, Pose current) {
        double[] xy = estimateXY(tags, lastRedTilt, lastBlueTilt, then[3]);
        if (xy == null) return;
        double errX = xy[0] - then[1];
        double errY = xy[1] - then[2];
        lastVisionError = Math.hypot(errX, errY);

        if (lastVisionError > MAX_JUMP_IN) {
            if (++outlierFrames >= RELOCK_AFTER_FRAMES) relocalize();
            return;
        }
        outlierFrames = 0;

        double dHeading = 0;
        if (!Double.isNaN(visionHeading)) {
            double errH = HiveTagGeometry.normalizeAngle(visionHeading - then[3]);
            if (Math.abs(errH) < Math.toRadians(15)) dHeading = HEADING_GAIN * errH;
        }
        double dx = XY_GAIN * errX, dy = XY_GAIN * errY;
        follower.setPose(new Pose(current.x() + dx, current.y() + dy,
                HiveTagGeometry.normalizeAngle(current.heading() + dHeading)));
        turret.notePoseShift(dx, dy, dHeading); // a correction is not the robot turning
        for (double[] p : poseHistory) { // keep the history in the same frame as the corrected pose
            p[1] += dx;
            p[2] += dy;
            p[3] += dHeading;
        }
    }

    /** Range-weighted robot X/Y implied by every visible tag, using each HIVE's measured (or snapped) tilt. */
    private static double[] estimateXY(List<TagObservation> tags, HiveTilt redTilt, HiveTilt blueTilt, double heading) {
        double sumX = 0, sumY = 0, sumW = 0;
        for (TagObservation t : tags) {
            HiveTilt tilt = t.info.hive == Hive.RED ? redTilt : blueTilt;
            double tiltRad = Math.toRadians(HiveTagGeometry.tiltForLocalizationDeg(tilt));
            double[] xy = HiveTagGeometry.robotPositionFromTag(t, tiltRad, heading);
            double w = 1.0 / (t.rangeIn * t.rangeIn);
            sumX += xy[0] * w;
            sumY += xy[1] * w;
            sumW += w;
        }
        return sumW == 0 ? null : new double[]{sumX / sumW, sumY / sumW};
    }

    // ======================= TELEMETRY =======================
    private void addTelemetry(Pose pose, double targetDeg, boolean autoAim) {
        // VERY TOP: raw turret motor encoder
        telemetry.addData("TURRET ENCODER", "%d ticks  (%.1f°)   limits %d .. %d ticks", turret.ticks(), turret.angleDeg(),
                Math.round(TurretConstants.HARD_MIN_DEG / 360.0 * TurretConstants.TICKS_PER_TURRET_REV),
                Math.round(TurretConstants.HARD_MAX_DEG / 360.0 * TurretConstants.TICKS_PER_TURRET_REV));
        telemetry.addData("ODO POSE", "x %.1f  y %.1f  heading %.1f°", pose.x(), pose.y(),
                Math.toDegrees(pose.heading()));
        telemetry.addLine("gp1: LT shoot | RT intake | Y intake dir | B/A/X = 1510/1560/1460 | LB ramp | RB slow | dpad up feed speed");
        telemetry.addLine("gp2: A auto aim | B re-localize | X vision | dpad U/D speed trim");
        if (launcherReady && opModeIsActive()) addMechanismTelemetry();
        telemetry.addData("Localization", localized ? "LOCALIZED" : "SEARCHING: " + lockStatus);
        telemetry.addData("Robot", "(%.1f, %.1f) %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Tags seen", lastTags == null ? 0 : lastTags.size());
        telemetry.addData("Vision correction", visionCorrection
                ? (Double.isNaN(lastVisionError) ? "on" : String.format("on, last error %.1f in", lastVisionError))
                : "off");
        addTiltTelemetry(lastRedTilt);
        addTiltTelemetry(lastBlueTilt);
        telemetry.addLine();
        boolean override = Math.abs(gamepad2.left_stick_x) > TURRET_OVERRIDE_DEADBAND;
        telemetry.addData("Turret mode", override ? "MANUAL OVERRIDE (gp2 stick)"
                : turretHome ? "HOLD AT 0 (gp2 A = back to auto aim)"
                : autoAim ? "auto aim" : "manual (gp1 dpad L/R)");
        if (rezeroCount > 0) telemetry.addData("Turret re-zeroed (gp2 BACK)", "%d time(s)", rezeroCount);
        telemetry.addData("Aiming at", "%s cell", goal.aimingTop() ? "TOP" : "BOTTOM");
        double cur = turret.angleDeg();
        telemetry.addData("Turret", "%.1f°  (limits %.0f .. %.0f)", cur, TurretConstants.MIN_DEG, TurretConstants.MAX_DEG);
        if (!Double.isNaN(targetDeg)) {
            telemetry.addData("Turret target", "%.1f°  error %+.1f°", targetDeg, targetDeg - cur);
        }
    }

    private void addTiltTelemetry(HiveTilt tilt) {
        if (tilt == null || tilt.tagCount == 0) return;
        telemetry.addData(tilt.hive + " HIVE", "%s (%.1f°, %d tags)", tilt.state.label, tilt.tiltDeg, tilt.tagCount);
    }
}
