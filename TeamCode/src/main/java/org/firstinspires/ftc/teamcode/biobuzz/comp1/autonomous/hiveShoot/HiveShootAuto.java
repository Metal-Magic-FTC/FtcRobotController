package org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.hiveShoot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.v1.PathsV1;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.Hive;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.HiveTilt;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.TiltState;

/**
 * Reads which way our HIVE is tipped from the AprilTags, then drives to the matching shooting spot.
 *
 * - AUDIENCE cell up (cell facing away from the top FLOWER, the match-start setup per manual 10.3.1)
 *     -> shoot from the bottom spot
 * - SCORING cell up (HIVE has been tipped)
 *     -> shoot from the top spot
 *
 * Scans during init and for SCAN_AFTER_START_SEC after start. If the tags were never read cleanly it falls
 * back to the match-start state (audience cell up).
 */
@Autonomous(name = "BioBuzz Hive Shoot Auto", group = "BioBuzz")
public class HiveShootAuto extends LinearOpMode {

    /** Our alliance HIVE (the start pose is on the red side of the field). */
    private static final Hive MY_HIVE = Hive.RED;

    private static final Pose START_POSE = PathsV1.START_POSE;
    /** Audience cell up (initial state). */
    private static final double SHOOT_BOTTOM_X = 55.4255751014885, SHOOT_BOTTOM_Y = 7.808525033829501;
    /** Scoring cell up (tipped). */
    private static final double SHOOT_TOP_X = 42.97970230040596, SHOOT_TOP_Y = 110.05615696887686;
    /** Keeps the top path left of the HIVE frame (frame base spans X 47-97, Y 52-92). */
    private static final Pose TOP_PATH_CONTROL = new Pose(26.0, 60.0);

    /** A stable reading must repeat this many frames in a row before we trust it. */
    private static final int CONFIRM_FRAMES = 5;
    private static final double SCAN_AFTER_START_SEC = 0.5;
    private static final double PATH_TIMEOUT_SEC = 6.0;
    private static final double SHOOT_TIME_SEC = 3.0;

    private Follower follower;
    private Limelight3A limelight;

    private TiltState confirmedState = null;
    private TiltState candidateState = null;
    private int candidateFrames = 0;
    private HiveTilt lastTilt = null;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(HiveTagGeometry.APRILTAG_PIPELINE);
        limelight.start();

        while (opModeInInit()) {
            scanHive();
            telemetry.addLine("BioBuzz Hive Shoot Auto - scanning " + MY_HIVE + " HIVE");
            addScanTelemetry();
            telemetry.update();
        }
        if (isStopRequested()) {
            limelight.stop();
            return;
        }

        ElapsedTime scanTimer = new ElapsedTime();
        while (opModeIsActive() && scanTimer.seconds() < SCAN_AFTER_START_SEC) {
            follower.update();
            scanHive();
        }
        limelight.stop();

        TiltState decision = confirmedState != null ? confirmedState : TiltState.AUDIENCE_UP;
        boolean shootTop = decision == TiltState.SCORING_UP;
        Pose shootPose = shootTop
                ? facingHive(SHOOT_TOP_X, SHOOT_TOP_Y)
                : facingHive(SHOOT_BOTTOM_X, SHOOT_BOTTOM_Y);

        PathChain toShoot = shootTop
                ? follower.pathBuilder()
                        .addPath(new BezierCurve(START_POSE, TOP_PATH_CONTROL, shootPose))
                        .setLinearHeadingInterpolation(START_POSE.getHeading(), shootPose.getHeading())
                        .build()
                : follower.pathBuilder()
                        .addPath(new BezierLine(START_POSE, shootPose))
                        .setLinearHeadingInterpolation(START_POSE.getHeading(), shootPose.getHeading())
                        .build();

        String spot = shootTop ? "TOP" : "BOTTOM";
        String source = confirmedState != null ? "tags" : "default (tags not read)";
        runPath(toShoot, "Driving to " + spot + " shoot spot", decision, source);
        shoot(decision, spot, source);

        while (opModeIsActive()) {
            follower.update();
            addStatusTelemetry("Done", decision, spot, source);
            telemetry.update();
        }
    }

    /** Aim the robot front at the center of our HIVE: heading = atan2(dy, dx). */
    private Pose facingHive(double x, double y) {
        double hiveX = MY_HIVE == Hive.RED ? HiveTagGeometry.RED_HIVE_X_IN : HiveTagGeometry.BLUE_HIVE_X_IN;
        double heading = Math.atan2(HiveTagGeometry.PIVOT_Y_IN - y, hiveX - x);
        return new Pose(x, y, heading);
    }

    private void scanHive() {
        lastTilt = HiveTagGeometry.measureHive(
                HiveTagGeometry.readTags(limelight.getLatestResult()), MY_HIVE);
        TiltState state = lastTilt.state;
        if (state != TiltState.SCORING_UP && state != TiltState.AUDIENCE_UP) return;

        if (state == candidateState) {
            candidateFrames++;
        } else {
            candidateState = state;
            candidateFrames = 1;
        }
        if (candidateFrames >= CONFIRM_FRAMES) confirmedState = state;
    }

    private void runPath(PathChain path, String status, TiltState decision, String source) {
        follower.followPath(path, true);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && follower.isBusy() && timer.seconds() < PATH_TIMEOUT_SEC) {
            follower.update();
            addStatusTelemetry(status, decision, "", source);
            telemetry.update();
        }
    }

    /** TODO: replace the wait with the launcher once the shooter subsystem exists. */
    private void shoot(TiltState decision, String spot, String source) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < SHOOT_TIME_SEC) {
            follower.update();
            addStatusTelemetry("Shooting", decision, spot, source);
            telemetry.update();
        }
    }

    private void addScanTelemetry() {
        telemetry.addData("Confirmed", confirmedState == null ? "none yet" : confirmedState.label);
        if (lastTilt != null && lastTilt.tagCount > 0) {
            telemetry.addData("Live", "%s (%.1f°, %d tags)", lastTilt.state.label, lastTilt.tiltDeg, lastTilt.tagCount);
        } else {
            telemetry.addData("Live", "no " + MY_HIVE + " tags visible");
        }
        boolean top = confirmedState == TiltState.SCORING_UP;
        telemetry.addData("Will shoot from", top ? "TOP (%.1f, %.1f)" : "BOTTOM (%.1f, %.1f)",
                top ? SHOOT_TOP_X : SHOOT_BOTTOM_X, top ? SHOOT_TOP_Y : SHOOT_BOTTOM_Y);
    }

    private void addStatusTelemetry(String status, TiltState decision, String spot, String source) {
        Pose pose = follower.getPose();
        telemetry.addData("Status", status);
        telemetry.addData("HIVE", "%s (from %s)", decision.label, source);
        if (!spot.isEmpty()) telemetry.addData("Shoot spot", spot);
        telemetry.addData("Pose", "(%.1f, %.1f) %.1f°", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }
}
