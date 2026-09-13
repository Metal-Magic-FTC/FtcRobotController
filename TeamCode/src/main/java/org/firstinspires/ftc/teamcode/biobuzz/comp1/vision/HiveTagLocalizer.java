package org.firstinspires.ftc.teamcode.biobuzz.comp1.vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.Hive;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.HiveTilt;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.TagObservation;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedroPathing.Constants;

import java.util.ArrayDeque;
import java.util.List;

/**
 * Program 2: re-localizes the Pedro Pathing pose from the HIVE AprilTags, accounting for which way each HIVE is
 * tipped (or the measured angle while it is swinging), and can turn the robot to face the field center.
 *
 * The robot starts at START_POSE (bottom-right corner) until you scan tags.
 *
 * gamepad1:
 *   sticks  - robot-centric drive
 *   X       - snap pose to vision (hold still; averages the last SNAP_WINDOW_SEC of readings)
 *   Y       - toggle continuous vision correction
 *   A       - turn to face the field center
 *   B       - cancel the turn (moving the sticks also cancels)
 *   BACK    - reset pose to START_POSE
 */
@TeleOp(name = "HIVE Tag Localizer", group = "BioBuzz")
public class HiveTagLocalizer extends LinearOpMode {

    /**
     * Bottom-right corner: back against the audience wall, right side against the blue wall, facing away from
     * the audience (+Y, toward the HIVE). Uses the 18 x 18 in robot size from Constants.
     */
    private static final double ROBOT_HALF_SIZE_IN = 9.0;
    private static final Pose START_POSE =
            new Pose(144.0 - ROBOT_HALF_SIZE_IN, ROBOT_HALF_SIZE_IN, Math.toRadians(90));

    /** false = keep Pinpoint heading and only fix X/Y (recommended). true = also take heading from a tag pair. */
    private static final boolean USE_VISION_HEADING = false;
    private static final double MAX_TAG_RANGE_IN = 120.0;
    private static final double CONTINUOUS_GAIN = 0.15;
    private static final double SNAP_WINDOW_SEC = 0.5;
    private static final int SNAP_MIN_SAMPLES = 3;

    private static final double TURN_TOLERANCE_DEG = 2.0;
    private static final int TURN_SETTLE_LOOPS = 5;
    private static final double STICK_CANCEL = 0.3;

    private Follower follower;
    private final ArrayDeque<double[]> recentEstimates = new ArrayDeque<>(); // {x, y, heading, time}

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(START_POSE);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(HiveTagGeometry.APRILTAG_PIPELINE);
        limelight.start();

        telemetry.addLine("HIVE Tag Localizer ready");
        telemetry.addData("Start pose", "(%.1f, %.1f) %.0f°",
                START_POSE.getX(), START_POSE.getY(), Math.toDegrees(START_POSE.getHeading()));
        telemetry.update();
        waitForStart();

        follower.startTeleopDrive();
        ElapsedTime clock = new ElapsedTime();

        boolean continuous = false;
        boolean turning = false;
        double turnTarget = 0;
        int settleLoops = 0;
        boolean prevX = false, prevY = false, prevA = false, prevBack = false;

        while (opModeIsActive()) {
            follower.update();
            double now = clock.seconds();
            Pose pose = follower.getPose();

            // ---------- vision ----------
            List<TagObservation> tags = HiveTagGeometry.readTags(limelight.getLatestResult());
            HiveTilt redTilt = HiveTagGeometry.measureHive(tags, Hive.RED);
            HiveTilt blueTilt = HiveTagGeometry.measureHive(tags, Hive.BLUE);

            double visionHeading = HiveTagGeometry.headingFromTagPair(tags);
            double headingForVision = USE_VISION_HEADING && !Double.isNaN(visionHeading)
                    ? visionHeading : pose.getHeading();
            double[] estimate = estimateRobotXY(tags, redTilt, blueTilt, headingForVision);

            if (estimate != null) {
                recentEstimates.addLast(new double[]{estimate[0], estimate[1], headingForVision, now});
            }
            while (!recentEstimates.isEmpty() && now - recentEstimates.peekFirst()[3] > SNAP_WINDOW_SEC) {
                recentEstimates.removeFirst();
            }

            // ---------- buttons ----------
            // Pose edits are blocked while turning: turnTo holds the X/Y it started with, so moving the pose
            // mid-turn would make the robot drive to "correct" itself.
            if (gamepad1.x && !prevX && !turning) snapToVision(pose);
            if (gamepad1.y && !prevY) continuous = !continuous;
            if (gamepad1.back && !prevBack && !turning) follower.setPose(START_POSE);

            if (continuous && estimate != null && !turning) {
                double heading = USE_VISION_HEADING
                        ? pose.getHeading() + CONTINUOUS_GAIN
                                * HiveTagGeometry.normalizeAngle(headingForVision - pose.getHeading())
                        : pose.getHeading();
                follower.setPose(new Pose(
                        pose.getX() + CONTINUOUS_GAIN * (estimate[0] - pose.getX()),
                        pose.getY() + CONTINUOUS_GAIN * (estimate[1] - pose.getY()),
                        heading));
            }

            // ---------- face field center ----------
            pose = follower.getPose();
            double centerHeading = HiveTagGeometry.headingToFieldCenter(pose.getX(), pose.getY());
            double turnNeeded = HiveTagGeometry.normalizeAngle(centerHeading - pose.getHeading());

            if (gamepad1.a && !prevA && !turning) {
                turnTarget = centerHeading;
                follower.turnTo(turnTarget);
                turning = true;
                settleLoops = 0;
            }

            if (turning) {
                double error = HiveTagGeometry.normalizeAngle(turnTarget - pose.getHeading());
                settleLoops = Math.abs(error) < Math.toRadians(TURN_TOLERANCE_DEG) ? settleLoops + 1 : 0;
                boolean driverOverride = gamepad1.b
                        || Math.abs(gamepad1.left_stick_x) > STICK_CANCEL
                        || Math.abs(gamepad1.left_stick_y) > STICK_CANCEL
                        || Math.abs(gamepad1.right_stick_x) > STICK_CANCEL;
                if (settleLoops >= TURN_SETTLE_LOOPS || driverOverride) {
                    follower.startTeleopDrive();
                    turning = false;
                }
            } else {
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            }

            prevX = gamepad1.x;
            prevY = gamepad1.y;
            prevA = gamepad1.a;
            prevBack = gamepad1.back;

            // ---------- telemetry ----------
            telemetry.addLine("X snap | Y continuous | A face center | B cancel | BACK reset");
            telemetry.addData("Pedro pose", "(%.1f, %.1f) %.1f°",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
            telemetry.addData("Continuous vision", continuous);
            telemetry.addData("Heading to center", "%.1f°  (turn %+.1f°)",
                    Math.toDegrees(centerHeading), Math.toDegrees(turnNeeded));
            telemetry.addData("Turning", turning);

            telemetry.addLine();
            if (estimate != null) {
                telemetry.addData("Vision XY", "(%.1f, %.1f) from %d tag(s)", estimate[0], estimate[1], tags.size());
            } else {
                telemetry.addData("Vision XY", "no usable tags");
            }
            telemetry.addData("Vision heading (tag pair)",
                    Double.isNaN(visionHeading) ? "need 2 tags on one cell" : String.format("%.1f°", Math.toDegrees(visionHeading)));
            addHiveTelemetry(redTilt);
            addHiveTelemetry(blueTilt);
            telemetry.update();
        }

        limelight.stop();
    }

    /** Range-weighted average of the robot position implied by every visible tag. */
    private double[] estimateRobotXY(List<TagObservation> tags, HiveTilt redTilt, HiveTilt blueTilt, double heading) {
        double sumX = 0, sumY = 0, sumW = 0;
        for (TagObservation t : tags) {
            if (t.rangeIn > MAX_TAG_RANGE_IN) continue;
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

    private void snapToVision(Pose current) {
        if (recentEstimates.size() < SNAP_MIN_SAMPLES) return;
        double sumX = 0, sumY = 0, sumSin = 0, sumCos = 0;
        for (double[] e : recentEstimates) {
            sumX += e[0];
            sumY += e[1];
            sumSin += Math.sin(e[2]);
            sumCos += Math.cos(e[2]);
        }
        int n = recentEstimates.size();
        double heading = USE_VISION_HEADING ? Math.atan2(sumSin, sumCos) : current.getHeading();
        follower.setPose(new Pose(sumX / n, sumY / n, heading));
    }

    private void addHiveTelemetry(HiveTilt tilt) {
        if (tilt.tagCount == 0) {
            telemetry.addData(tilt.hive + " HIVE", tilt.state.label);
            return;
        }
        telemetry.addData(tilt.hive + " HIVE", "%s | measured %.1f° | used %.1f°",
                tilt.state.label, tilt.tiltDeg, HiveTagGeometry.tiltForLocalizationDeg(tilt));
    }
}
