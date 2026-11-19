package org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.v4;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;

// Limelight HIVE wait - re-enable once the Limelight is on the new robot (also uncomment the blocks below).
// import com.qualcomm.hardware.limelightvision.Limelight3A;
// import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry;
// import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.Hive;
// import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.TagObservation;

/**
 * Auto V4 (RED, new robot): DRIVE ONLY. No intake / turret / shooter / transfer code; the pauses between paths are
 * where those actions will go later.
 *
 *   (Limelight HIVE wait - commented out until the Limelight is mounted)
 *   1. Pause (shoot preloads)
 *   2. Path 1 -> far wall FLOWER, pause (pick up + shoot)
 *   3. Path 2 -> red wall FLOWER, pause (pick up)
 *   4. Path 3 -> corner, pause (shoot + pick up)
 *   5. Path 4 -> park
 *
 * BEFORE PRESSING INIT: robot at PathsV4.start (29.0, 133.7) facing the audience (heading 270), still during INIT.
 */
@Autonomous(name = "BioBuzz Auto V4 (drive only)", group = "BioBuzz")
public class AutoV4 extends LinearOpMode {

    /** Give up on a path after this long so a stuck robot still runs the rest of the auto. */
    private static final double PATH_TIMEOUT_SEC = 6.0;

    /** Placeholder pauses (seconds) where mechanism actions will go. Set to 0 to skip. */
    private static final double PRELOAD_PAUSE_SEC = 1.0;
    private static final double[] PAUSE_AFTER_PATH_SEC = {1.0, 1.0, 1.0, 0.0};

    /** Per-path speed, fraction of top speed (1.0 = full). Lower one if that path is jerky. */
    private static final double[] PATH_SPEED = {1.0, 1.0, 1.0, 1.0};

    // private static final int RED_TAG_CONFIRM_FRAMES = 5;
    // private Limelight3A limelight;

    private Follower follower;

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        follower.setPose(PathsV4.start);

        // limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // limelight.setPollRateHz(100);
        // limelight.pipelineSwitch(HiveTagGeometry.APRILTAG_PIPELINE);
        // limelight.start();

        // Build everything during init so start is instant
        PathsV4 paths = new PathsV4();
        Path[] sequence = {paths.path1(), paths.path2(), paths.path3(), paths.path4()};
        String[] names = {"1 to far wall FLOWER", "2 to red wall FLOWER", "3 to corner", "4 park"};

        // Odometry runs during INIT so you can check the start pose.
        while (opModeInInit()) {
            follower.update();
            telemetry.addLine("BioBuzz Auto V4 (drive only) ready.");
            addTelemetry("Waiting for start");
            telemetry.update();
        }
        if (isStopRequested()) {
            // limelight.stop();
            return;
        }

        // waitForRedHive();
        // limelight.stop();

        holdFor(PRELOAD_PAUSE_SEC, "Pause (shoot preloads)");

        for (int i = 0; i < sequence.length && opModeIsActive(); i++) {
            runPath(sequence[i], "Path " + names[i], PATH_SPEED[i]);
            holdFor(PAUSE_AFTER_PATH_SEC[i], "Pause after " + names[i]);
        }

        while (opModeIsActive()) {
            follower.update();
            addTelemetry("Done");
            telemetry.update();
        }
    }

    // /** Waits until the red HIVE tips: from the far wall the red tags only face the camera after it comes down. */
    // private void waitForRedHive() {
    //     ElapsedTime timer = new ElapsedTime();
    //     int frames = 0;
    //     while (opModeIsActive() && frames < RED_TAG_CONFIRM_FRAMES) {
    //         frames = seesRedTag() ? frames + 1 : 0;
    //         follower.update();
    //         addTelemetry(String.format("Waiting for red HIVE (%.1f s, red tag frames %d/%d)",
    //                 timer.seconds(), frames, RED_TAG_CONFIRM_FRAMES));
    //         telemetry.update();
    //     }
    // }
    //
    // private boolean seesRedTag() {
    //     for (TagObservation tag : HiveTagGeometry.readTags(limelight.getLatestResult())) {
    //         if (tag.info.hive == Hive.RED) return true;
    //     }
    //     return false;
    // }

    /** speedFraction: 0..1 of top speed for just this path, restored afterwards. */
    private void runPath(Path path, String status, double speedFraction) {
        Constants.foresightConfig.maxPathSpeed.set(
                speedFraction >= 1.0 ? com.pedropathing.algorithm.ForesightConfig.Constraint.NONE : speedFraction);
        try {
            follower.follow(path);
            ElapsedTime timer = new ElapsedTime();
            while (opModeIsActive() && follower.isBusy() && timer.seconds() < PATH_TIMEOUT_SEC) {
                follower.update();
                addTelemetry(status);
                telemetry.update();
            }
        } finally {
            Constants.foresightConfig.maxPathSpeed.set(com.pedropathing.algorithm.ForesightConfig.Constraint.NONE);
        }
    }

    /** Holds the end of the last path (the follower keeps holding) for a while. */
    private void holdFor(double seconds, String status) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            follower.update();
            addTelemetry(status);
            telemetry.update();
        }
    }

    private void addTelemetry(String status) {
        Pose pose = follower.pose();
        telemetry.addData("Status", status);
        telemetry.addData("Pose", "(%.1f, %.1f) %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }
}
