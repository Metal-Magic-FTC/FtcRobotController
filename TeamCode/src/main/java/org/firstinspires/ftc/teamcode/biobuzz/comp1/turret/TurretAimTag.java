package org.firstinspires.ftc.teamcode.biobuzz.comp1.turret;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry;

import java.util.ArrayDeque;

/**
 * Keeps the turret pointed at AprilTag TARGET_TAG_ID no matter where the robot is moved / rotated.
 *
 * HOW IT WORKS
 *   We don't know the starting pose, so the Pinpoint (through Pedro's PinpointLocalizer, not the Follower - the
 *   drive motors are never touched) starts at (0, 0, 0) wherever the robot is at INIT. That "odometry frame" is
 *   all we need: whenever the fixed Limelight sees the tag we compute where the tag is IN THAT FRAME
 *   (robot pose at the moment the photo was taken + tag offset from the camera). The tag doesn't move, so from
 *   then on odometry alone is enough to aim, even when the tag is out of the camera's view (e.g. behind the robot).
 *   Every new sighting refines the tag position and cancels odometry drift.
 *
 *   Aim:  turretAngle = atan2(tagY - turretY, tagX - turretX) - robotHeading, wrapped into the turret limits.
 *
 * SETUP
 *   - Turret facing robot FRONT at INIT (that's encoder zero). Robot must be still during INIT (Pinpoint IMU reset).
 *   - Camera mount numbers come from HiveTagGeometry (CAMERA_FORWARD_IN etc.) - measure them.
 *   - Limelight pipeline TAG_PIPELINE: AprilTag, 36h11, "Full 3D" on, ID filter must allow TARGET_TAG_ID, and the
 *     tag size must match your printed tag 24 (wrong size = wrong distance; the direction is still right).
 *
 * gamepad1:
 *   A            - toggle auto aim (on at start)
 *   left stick X - manual turret while auto aim is off
 *   B            - forget the tag (re-acquire on next sighting)
 *   X            - reset odometry to (0,0,0) and forget the tag
 */
@TeleOp(name = "Turret Aim Tag 24", group = "BioBuzz Tests")
public class TurretAimTag extends LinearOpMode {

    // ======================= TARGET / VISION =======================
    private static final int TARGET_TAG_ID = 24;
    private static final int TAG_PIPELINE = HiveTagGeometry.APRILTAG_PIPELINE;
    private static final long MAX_STALENESS_MS = 100;
    private static final double MAX_TAG_RANGE_IN = 150.0;
    /** How much each new sighting moves the stored tag position (0..1). */
    private static final double TAG_FILTER_GAIN = 0.25;
    /** A sighting this far from the stored tag is ignored ... */
    private static final double OUTLIER_IN = 18.0;
    /** ... unless it keeps happening this many frames in a row (then the stored position was wrong: re-lock). */
    private static final int OUTLIER_RELOCK_FRAMES = 15;
    private static final double POSE_HISTORY_SEC = 1.0;

    private PinpointLocalizer localizer;
    private TurretController turret;
    private Limelight3A limelight;
    private final ElapsedTime clock = new ElapsedTime();

    /** {time sec, x, y, heading} so a sighting can use the pose from when the photo was actually taken. */
    private final ArrayDeque<double[]> poseHistory = new ArrayDeque<>();

    private boolean tagLocked = false;
    private double tagX, tagY;
    private int outlierFrames = 0;
    private boolean tagVisible = false;
    private double lastTagRangeIn = 0;
    private double lastImageAgeMs = 0;

    @Override
    public void runOpMode() {
        turret = new TurretController(hardwareMap);

        telemetry.addLine("Resetting Pinpoint - keep the robot STILL");
        telemetry.update();
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        resetOdometry();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(TAG_PIPELINE);
        limelight.start();

        // Run localization during INIT too, so the tag can be locked before START (turret stays unpowered).
        while (opModeInInit()) {
            Pose pose = updatePose();
            updateTag(pose);
            telemetry.addLine("Turret facing FRONT? Press START to aim at tag " + TARGET_TAG_ID);
            addTelemetry(pose, Double.NaN);
            telemetry.update();
        }

        boolean autoAim = true;
        while (opModeIsActive()) {
            Pose pose = updatePose();
            updateTag(pose);

            if (gamepad1.aWasPressed()) autoAim = !autoAim;
            if (gamepad1.bWasPressed()) tagLocked = false;
            if (gamepad1.xWasPressed()) {
                resetOdometry();
                pose = localizer.getPose();
            }

            double targetDeg = Double.NaN;
            if (autoAim && tagLocked) {
                targetDeg = turret.aimAt(pose.getX(), pose.getY(), pose.getHeading(), tagX, tagY);
            } else if (!autoAim) {
                turret.manual(-gamepad1.left_stick_x); // stick left = counter-clockwise
            } else {
                turret.stop(); // no tag yet: hold still
            }

            telemetry.addData("Auto aim (A)", autoAim);
            addTelemetry(pose, targetDeg);
            telemetry.update();
        }

        turret.stop();
        limelight.stop();
    }

    // ======================= ODOMETRY =======================
    private void resetOdometry() {
        localizer.resetIMU(); // resetPosAndIMU + 300 ms wait, robot must be still
        localizer.setPose(new Pose(0, 0, 0));
        poseHistory.clear();
        tagLocked = false;
        turret.resetHeadingTracking();
    }

    private Pose updatePose() {
        localizer.update();
        Pose pose = localizer.getPose();
        double now = clock.seconds();

        poseHistory.addLast(new double[]{now, pose.getX(), pose.getY(), pose.getHeading()});
        while (now - poseHistory.peekFirst()[0] > POSE_HISTORY_SEC) poseHistory.removeFirst();

        turret.updateHeading(pose.getHeading());
        return pose;
    }

    /** Closest recorded pose to a past time. */
    private double[] poseAt(double time) {
        double[] best = poseHistory.peekLast();
        for (double[] p : poseHistory) {
            if (Math.abs(p[0] - time) < Math.abs(best[0] - time)) best = p;
        }
        return best;
    }

    // ======================= VISION =======================
    private void updateTag(Pose pose) {
        tagVisible = false;
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getStaleness() > MAX_STALENESS_MS) return;

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (tag.getFiducialId() != TARGET_TAG_ID) continue;
            Pose3D camPose = tag.getTargetPoseCameraSpace();
            if (camPose == null) continue;
            Position p = camPose.getPosition().toUnit(DistanceUnit.INCH);
            if (p.z <= 0) continue; // no 3D solve

            double[] robotRel = cameraToRobot(p);
            lastTagRangeIn = Math.hypot(robotRel[0], robotRel[1]);
            if (lastTagRangeIn > MAX_TAG_RANGE_IN) continue;
            tagVisible = true;

            // Robot pose when the frame was captured (robot may have turned since)
            lastImageAgeMs = result.getCaptureLatency() + result.getTargetingLatency() + result.getStaleness();
            double[] then = poseAt(clock.seconds() - lastImageAgeMs / 1000.0);
            double cos = Math.cos(then[3]), sin = Math.sin(then[3]);
            double seenX = then[1] + robotRel[0] * cos - robotRel[1] * sin;
            double seenY = then[2] + robotRel[0] * sin + robotRel[1] * cos;

            if (!tagLocked) {
                tagX = seenX;
                tagY = seenY;
                tagLocked = true;
                outlierFrames = 0;
            } else if (Math.hypot(seenX - tagX, seenY - tagY) > OUTLIER_IN) {
                if (++outlierFrames >= OUTLIER_RELOCK_FRAMES) tagLocked = false;
            } else {
                tagX += TAG_FILTER_GAIN * (seenX - tagX);
                tagY += TAG_FILTER_GAIN * (seenY - tagY);
                outlierFrames = 0;
            }
            return;
        }
    }

    /**
     * Limelight camera space (+x right, +y down, +z out of the lens) -> robot frame {forward, left} in inches,
     * using the camera mount from HiveTagGeometry.
     */
    private static double[] cameraToRobot(Position p) {
        double pitch = Math.toRadians(HiveTagGeometry.CAMERA_PITCH_DEG);
        double yaw = Math.toRadians(HiveTagGeometry.CAMERA_YAW_DEG);
        double levelForward = p.z * Math.cos(pitch) + p.y * Math.sin(pitch);
        double levelLeft = -p.x;
        double forward = HiveTagGeometry.CAMERA_FORWARD_IN + levelForward * Math.cos(yaw) - levelLeft * Math.sin(yaw);
        double left = HiveTagGeometry.CAMERA_LEFT_IN + levelForward * Math.sin(yaw) + levelLeft * Math.cos(yaw);
        return new double[]{forward, left};
    }

    // ======================= TELEMETRY =======================
    private void addTelemetry(Pose pose, double targetDeg) {
        telemetry.addLine("A auto aim | B forget tag | X reset odo | stick X manual");
        telemetry.addData("Robot (odo frame)", "(%.1f, %.1f) %.1f°",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.addData("Robot turn rate", "%.0f°/s", turret.robotOmegaDeg());
        telemetry.addLine();
        telemetry.addData("Tag " + TARGET_TAG_ID + " visible", tagVisible
                ? String.format("yes, %.1f in, image %.0f ms old", lastTagRangeIn, lastImageAgeMs) : "no");
        if (tagLocked) {
            double dx = pose.getX() - tagX, dy = pose.getY() - tagY;
            telemetry.addData("Tag (odo frame)", "(%.1f, %.1f)", tagX, tagY);
            telemetry.addData("Robot relative to tag", "(%.1f, %.1f)  dist %.1f in", dx, dy, Math.hypot(dx, dy));
        } else {
            telemetry.addData("Tag (odo frame)", "not locked - point the camera at tag " + TARGET_TAG_ID);
        }
        telemetry.addLine();
        double cur = turret.angleDeg();
        telemetry.addData("Turret", "%.1f°  (limits %.0f .. %.0f)", cur, TurretConstants.MIN_DEG, TurretConstants.MAX_DEG);
        if (!Double.isNaN(targetDeg)) {
            telemetry.addData("Turret target", "%.1f°  error %+.1f°", targetDeg, targetDeg - cur);
        }
    }
}
