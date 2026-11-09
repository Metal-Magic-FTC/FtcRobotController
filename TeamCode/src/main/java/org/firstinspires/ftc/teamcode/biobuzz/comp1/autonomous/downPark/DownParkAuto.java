package org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.downPark;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.Hive;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.HiveTilt;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.TagObservation;
import org.firstinspires.ftc.teamcode.biobuzz.util.BulkReader;

import java.util.List;

/**
 * Down Park auto (RED), paths from Down_Park.pp.json (see PathsDownPark):
 *   1. Wait for our HIVE to tip: its AprilTags become visible to the Limelight (same tag filter as BioBuzzTeleOp).
 *   2. Shoot preloads        (modelled as a SHOOT_SEC wait)
 *   3. Drive to the FLOWER   (Path 1)
 *   4. Shoot FLOWER pollen   (modelled as a SHOOT_SEC wait)
 *   5. Drive to park         (Path 2)
 *
 * If the tags are never seen within TAG_WAIT_TIMEOUT_SEC the auto carries on anyway so it still parks.
 *
 * BEFORE PRESSING INIT: robot at PathsDownPark.start, (31.2, 132.3) facing 270 deg (toward the audience / the red
 * HIVE), and still during INIT (Pinpoint IMU calibrates). Telemetry shows live tag sightings so you can check
 * the Limelight can see the HIVE from the start spot.
 */
@Autonomous(name = "BioBuzz Down Park Auto", group = "BioBuzz")
public class DownParkAuto extends LinearOpMode {

    private static final Hive MY_HIVE = Hive.RED;

    // Same vision filter as BioBuzzTeleOp.updateVision()
    private static final long MAX_STALENESS_MS = 100;
    private static final double MAX_TAG_RANGE_IN = 110.0;

    /** Frames in a row with our HIVE's tags before we believe it has tipped (ignores single-frame false hits). */
    private static final int CONFIRM_FRAMES = 5;
    /** Give up waiting for the tags after this long and run the rest anyway (leaves time to park). */
    private static final double TAG_WAIT_TIMEOUT_SEC = 15.0;
    /** Placeholder for the shooter until the subsystem exists. */
    private static final double SHOOT_SEC = 2.0;
    /** Give up on a path after this long so a stuck robot still tries the rest. */
    private static final double PATH_TIMEOUT_SEC = 6.0;

    private BulkReader bulk;
    private Follower follower;
    private Limelight3A limelight;
    private final ElapsedTime matchTimer = new ElapsedTime();

    private int tagFrames = 0;
    private HiveTilt lastTilt;

    @Override
    public void runOpMode() {
        bulk = new BulkReader(hardwareMap);
        telemetry.addLine("Calibrating Pinpoint - keep the robot STILL");
        telemetry.update();
        follower = Constants.create(hardwareMap);
        follower.setPose(PathsDownPark.start);
        PathsDownPark paths = new PathsDownPark();
        Path toFlower = paths.toFlower();
        Path toPark = paths.toPark();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(HiveTagGeometry.APRILTAG_PIPELINE);
        limelight.start();

        while (opModeInInit()) {
            update();
            updateTags();
            telemetry.addLine("Down Park Auto ready. Start at (31.2, 132.3) facing 270° (toward the HIVE).");
            addTelemetry("Waiting for START");
            telemetry.update();
        }
        if (isStopRequested()) {
            limelight.stop();
            return;
        }
        matchTimer.reset();

        // 1. Wait for the HIVE to tip (tags visible), holding the start pose.
        tagFrames = 0;
        ElapsedTime waitTimer = new ElapsedTime();
        while (opModeIsActive() && tagFrames < CONFIRM_FRAMES && waitTimer.seconds() < TAG_WAIT_TIMEOUT_SEC) {
            update();
            updateTags();
            addTelemetry(String.format("Waiting for %s HIVE to tip (%.1f / %.0f s)",
                    MY_HIVE, waitTimer.seconds(), TAG_WAIT_TIMEOUT_SEC));
            telemetry.update();
        }
        String trigger = tagFrames >= CONFIRM_FRAMES ? "tags seen" : "timed out, no tags";
        limelight.stop();

        // 2-5. Shoot, drive, shoot, park.
        hold(SHOOT_SEC, "Shooting preloads (" + trigger + ")");
        runPath(toFlower, "Driving to FLOWER");
        hold(SHOOT_SEC, "Shooting FLOWER pollen");
        runPath(toPark, "Driving to park");

        while (opModeIsActive()) {
            update();
            addTelemetry("Parked (" + trigger + ")");
            telemetry.update();
        }
    }

    /** One loop of hardware: fresh bulk read first, then odometry + path following. */
    private void update() {
        bulk.update();
        follower.update();
    }

    /** Counts consecutive frames that show at least one of our HIVE's tags. */
    private void updateTags() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getStaleness() > MAX_STALENESS_MS) {
            tagFrames = 0;
            return;
        }
        List<TagObservation> tags = HiveTagGeometry.readTags(result);
        tags.removeIf(t -> t.rangeIn > MAX_TAG_RANGE_IN || t.info.hive != MY_HIVE);
        if (tags.isEmpty()) {
            tagFrames = 0;
            return;
        }
        tagFrames++;
        lastTilt = HiveTagGeometry.measureHive(tags, MY_HIVE);
    }

    private void runPath(Path path, String status) {
        follower.follow(path);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && follower.isBusy() && timer.seconds() < PATH_TIMEOUT_SEC) {
            update();
            addTelemetry(status);
            telemetry.update();
        }
    }

    /** Holds the current pose (the follower keeps holding the end of the last path) for a while. */
    private void hold(double seconds, String status) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            update();
            addTelemetry(status);
            telemetry.update();
        }
    }

    private void addTelemetry(String status) {
        Pose pose = follower.pose();
        telemetry.addData("Status", status);
        if (opModeIsActive()) telemetry.addData("Match time", "%.1f s", matchTimer.seconds());
        telemetry.addData("Pose", "(%.1f, %.1f) %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData(MY_HIVE + " tags", tagFrames > 0 ? tagFrames + " frames in a row" : "not visible");
        if (lastTilt != null && lastTilt.tagCount > 0) {
            telemetry.addData("Last HIVE reading", "%s (%.1f°, %d tags)",
                    lastTilt.state.label, lastTilt.tiltDeg, lastTilt.tagCount);
        }
    }
}
