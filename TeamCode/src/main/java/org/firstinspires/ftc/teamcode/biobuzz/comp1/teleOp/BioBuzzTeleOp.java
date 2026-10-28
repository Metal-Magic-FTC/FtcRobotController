package org.firstinspires.ftc.teamcode.biobuzz.comp1.teleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.RedHiveGoal;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.TurretController;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.Hive;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.HiveTilt;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.TagObservation;

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
 * gamepad1:
 *   left stick      - drive (robot-centric), right stick X - turn, right bumper - slow mode
 *   A               - toggle turret auto aim (on at start)
 *   left/right trig - manual turret while auto aim is off (left = counter-clockwise)
 *   B               - forget the pose and re-localize from the tags
 *   X               - toggle continuous vision correction (on at start)
 */
@TeleOp(name = "BioBuzz TeleOp (Red)", group = "BioBuzz")
public class BioBuzzTeleOp extends LinearOpMode {

    // ======================= VISION LOCALIZATION (tune) =======================
    private static final long MAX_STALENESS_MS = 100;
    /** Ignore tags farther than this (3D solve gets noisy). */
    private static final double MAX_TAG_RANGE_IN = 110.0;
    /** Consistent frames needed for the first fix. */
    private static final int LOCK_SAMPLES = 8;
    /** Lock samples older than this are dropped. */
    private static final double LOCK_WINDOW_SEC = 1.5;
    /** The lock samples must agree within this (inches) or we keep collecting. */
    private static final double LOCK_MAX_SPREAD_IN = 3.0;
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

    private Follower follower;
    private Limelight3A limelight;
    private TurretController turret;
    private final RedHiveGoal goal = new RedHiveGoal(0);
    private final ElapsedTime clock = new ElapsedTime();

    /** {time sec, x, y, heading} so a sighting can be matched with the pose from when the photo was taken. */
    private final ArrayDeque<double[]> poseHistory = new ArrayDeque<>();
    /** Field-from-odometry transforms seen while searching: {dTheta, tx, ty, time}. */
    private final ArrayDeque<double[]> lockSamples = new ArrayDeque<>();

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

        telemetry.addLine("Calibrating Pinpoint - keep the robot STILL");
        telemetry.update();
        follower = Constants.create(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(HiveTagGeometry.APRILTAG_PIPELINE);
        limelight.start();

        // Localize during INIT too, so the robot can already be located when START is pressed.
        while (opModeInInit()) {
            Pose pose = updatePose();
            updateVision(pose);
            telemetry.addLine("Turret facing robot FRONT? Point the Limelight at a HIVE to localize.");
            addTelemetry(follower.pose(), Double.NaN, true);
            telemetry.update();
        }

        boolean autoAim = true;
        while (opModeIsActive()) {
            double scale = gamepad1.right_bumper ? SLOW_MODE_SCALE : 1.0;
            follower.manual(-gamepad1.left_stick_y * scale, -gamepad1.left_stick_x * scale,
                    -gamepad1.right_stick_x * scale);

            Pose pose = updatePose();
            updateVision(pose);
            pose = follower.pose();

            if (gamepad1.aWasPressed()) autoAim = !autoAim;
            if (gamepad1.xWasPressed()) visionCorrection = !visionCorrection;
            if (gamepad1.bWasPressed()) relocalize();

            double targetDeg = Double.NaN;
            if (!autoAim) {
                turret.manual(gamepad1.left_trigger - gamepad1.right_trigger);
            } else if (localized) {
                goal.update(pose.y());
                targetDeg = turret.aimAt(pose.x(), pose.y(), pose.heading(), goal.x(), goal.y());
            } else {
                turret.stop(); // do not know where the goal is yet
            }

            addTelemetry(pose, targetDeg, autoAim);
            telemetry.update();
        }

        turret.stop();
        limelight.stop();
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
        lockSamples.clear();
        outlierFrames = 0;
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
        if (tags.isEmpty()) return;
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

    /** First fix: needs heading from a tag pair. Collects field-from-odometry transforms until they agree. */
    private void tryLock(List<TagObservation> tags, double visionHeading, double[] then) {
        if (Double.isNaN(visionHeading)) return;
        double[] xy = estimateXY(tags, visionHeading);
        if (xy == null) return;

        // Transform T with field = T(odometry): rotate by dTheta, then translate
        double dTheta = HiveTagGeometry.normalizeAngle(visionHeading - then[3]);
        double cos = Math.cos(dTheta), sin = Math.sin(dTheta);
        double tx = xy[0] - (then[1] * cos - then[2] * sin);
        double ty = xy[1] - (then[1] * sin + then[2] * cos);
        double now = clock.seconds();
        lockSamples.addLast(new double[]{dTheta, tx, ty, now});
        while (!lockSamples.isEmpty() && now - lockSamples.peekFirst()[3] > LOCK_WINDOW_SEC) lockSamples.removeFirst();
        if (lockSamples.size() < LOCK_SAMPLES) return;

        // Average the transforms (circular mean for the angle) and check they agree
        double sumSin = 0, sumCos = 0, sumX = 0, sumY = 0;
        for (double[] s : lockSamples) {
            sumSin += Math.sin(s[0]);
            sumCos += Math.cos(s[0]);
            sumX += s[1];
            sumY += s[2];
        }
        int n = lockSamples.size();
        double meanX = sumX / n, meanY = sumY / n;
        for (double[] s : lockSamples) {
            if (Math.hypot(s[1] - meanX, s[2] - meanY) > LOCK_MAX_SPREAD_IN) return;
        }
        double meanTheta = Math.atan2(sumSin, sumCos);

        // Apply the averaged transform to the CURRENT odometry pose
        Pose now0 = follower.pose();
        double c = Math.cos(meanTheta), s = Math.sin(meanTheta);
        follower.setPose(new Pose(
                meanX + now0.x() * c - now0.y() * s,
                meanY + now0.x() * s + now0.y() * c,
                HiveTagGeometry.normalizeAngle(now0.heading() + meanTheta)));
        poseHistory.clear();
        lockSamples.clear();
        localized = true;
        outlierFrames = 0;
    }

    /** Once localized: nudge the pose toward vision, measured against where the robot was when the photo was taken. */
    private void correct(List<TagObservation> tags, double visionHeading, double[] then, Pose current) {
        double[] xy = estimateXY(tags, then[3]);
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
        for (double[] p : poseHistory) { // keep the history in the same frame as the corrected pose
            p[1] += dx;
            p[2] += dy;
            p[3] += dHeading;
        }
    }

    /** Range-weighted robot X/Y implied by every visible tag, using each HIVE's measured (or snapped) tilt. */
    private double[] estimateXY(List<TagObservation> tags, double heading) {
        double sumX = 0, sumY = 0, sumW = 0;
        for (TagObservation t : tags) {
            HiveTilt tilt = t.info.hive == Hive.RED ? lastRedTilt : lastBlueTilt;
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
        telemetry.addLine("A auto aim | B re-localize | X vision correction | RB slow");
        telemetry.addData("Localization", localized ? "LOCALIZED" : String.format("SEARCHING (%d/%d frames)",
                lockSamples.size(), LOCK_SAMPLES));
        telemetry.addData("Robot", "(%.1f, %.1f) %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Tags seen", lastTags == null ? 0 : lastTags.size());
        telemetry.addData("Vision correction", visionCorrection
                ? (Double.isNaN(lastVisionError) ? "on" : String.format("on, last error %.1f in", lastVisionError))
                : "off");
        addTiltTelemetry(lastRedTilt);
        addTiltTelemetry(lastBlueTilt);
        telemetry.addLine();
        telemetry.addData("Turret auto aim", autoAim);
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
