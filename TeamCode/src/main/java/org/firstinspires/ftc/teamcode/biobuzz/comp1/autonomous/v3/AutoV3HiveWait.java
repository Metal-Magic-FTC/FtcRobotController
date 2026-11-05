package org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.v3;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.RedHiveGoal;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.TurretController;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.Hive;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.TagObservation;

/**
 * Auto V3 (RED, far wall start). The turret aims at the red HIVE the whole time.
 *   1. Wait for the red HIVE to tip: from the far wall the red tags face away from the camera until it tips, so the
 *      first red tag seen means the HIVE came down. Blue tags are ignored (blue's up cell faces the camera from START).
 *   2. Shoot 4 preloads.
 *   3. Go to the far wall FLOWER.   4. Pick up and shoot those 4.
 *   5. Go to the red wall FLOWER and pick up.   6. Go to the shooting position and shoot them.
 *   7. Go to the corner and pick up 4.   8. Shoot those.   9. Park.
 *
 * shoot() and pickUp() are timed placeholders until the shooter and intake subsystems exist.
 *
 * BEFORE PRESSING INIT
 *   1. Robot at PathsV3.start facing the audience (heading 270).
 *   2. Turret pointing straight out the robot FRONT (toward the audience). That is turret 0 deg.
 *   3. Robot still during INIT (Pinpoint IMU calibrates).
 */
@Autonomous(name = "BioBuzz Auto V3 Hive Wait", group = "BioBuzz")
public class AutoV3HiveWait extends LinearOpMode {

    /** Give up on a path after this long so a stuck robot still runs the rest of the auto. */
    private static final double PATH_TIMEOUT_SEC = 6.0;
    /** Frames in a row that must show a red tag before we trust it. */
    private static final int RED_TAG_CONFIRM_FRAMES = 5;
    /** Placeholder durations until the real mechanisms replace them. */
    private static final double SHOOT_SEC = 2.5;
    private static final double PICK_UP_SEC = 1.0;
    /**
     * Path 1 packs a 181 deg turn into ~35 in, so drive it at this fraction of top speed to let the heading keep up.
     * Raise toward 1.0 if it's smooth, lower (e.g. 0.4) if it still jerks.
     */
    private static final double PATH1_SPEED = 0.55;

    private Follower follower;
    private TurretController turret;
    private Limelight3A limelight;
    private RedHiveGoal goal;

    @Override
    public void runOpMode() {
        turret = new TurretController(hardwareMap);
        follower = Constants.create(hardwareMap);
        follower.setPose(PathsV3.start);
        goal = new RedHiveGoal(PathsV3.start.y());

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(HiveTagGeometry.APRILTAG_PIPELINE);
        limelight.start();

        // Build everything during init so start is instant
        PathsV3 paths = new PathsV3();
        Path toFarFlower = paths.path1();
        Path toRedWallFlower = paths.path2();
        Path toShootSpot = paths.path3();
        Path toCorner = paths.path4();
        Path toPark = paths.path5();

        // Odometry runs during INIT so you can check the start pose; the turret stays unpowered until START.
        while (opModeInInit()) {
            follower.update();
            telemetry.addLine("BioBuzz Auto V3 ready. Turret facing robot FRONT?");
            addTelemetry("Waiting for start", Double.NaN);
            telemetry.update();
        }
        if (isStopRequested()) {
            limelight.stop();
            return;
        }

        waitForRedHive();                 // 1
        limelight.stop();
        shoot("Shooting preloads");       // 2

        runPath(toFarFlower, "To far wall FLOWER", PATH1_SPEED); // 3
        pickUp("Picking up at far wall FLOWER");          // 4
        shoot("Shooting far wall FLOWER POLLEN");

        runPath(toRedWallFlower, "To red wall FLOWER");   // 5
        pickUp("Picking up at red wall FLOWER");

        runPath(toShootSpot, "To shooting position");     // 6
        shoot("Shooting red wall FLOWER POLLEN");

        runPath(toCorner, "To corner");                   // 7
        pickUp("Picking up in corner");
        shoot("Shooting corner POLLEN");                  // 8

        runPath(toPark, "Parking");                       // 9

        while (opModeIsActive()) {
            double target = updateRobot();
            addTelemetry("Done", target);
            telemetry.update();
        }
        turret.stop();
    }

    /** TODO: alternate auto if the HIVE has not tipped within 8 seconds. For now this waits as long as it takes. */
    private void waitForRedHive() {
        ElapsedTime timer = new ElapsedTime();
        int frames = 0;
        while (opModeIsActive() && frames < RED_TAG_CONFIRM_FRAMES) {
            frames = seesRedTag() ? frames + 1 : 0;
            double target = updateRobot();
            addTelemetry(String.format("Waiting for red HIVE (%.1f s, red tag frames %d/%d)",
                    timer.seconds(), frames, RED_TAG_CONFIRM_FRAMES), target);
            telemetry.update();
        }
    }

    private boolean seesRedTag() {
        for (TagObservation tag : HiveTagGeometry.readTags(limelight.getLatestResult())) {
            if (tag.info.hive == Hive.RED) return true;
        }
        return false;
    }

    /** TODO: replace the wait with the shooter once that subsystem exists. */
    private void shoot(String status) {
        holdFor(SHOOT_SEC, status);
    }

    /** TODO: replace the wait with the intake once that subsystem exists. */
    private void pickUp(String status) {
        holdFor(PICK_UP_SEC, status);
    }

    private void runPath(Path path, String status) {
        runPath(path, status, 1.0);
    }

    /** speedFraction: 0..1 of top speed for just this path (Foresight's maxPathSpeed), restored afterwards. */
    private void runPath(Path path, String status, double speedFraction) {
        Constants.foresightConfig.maxPathSpeed.set(speedFraction >= 1.0 ? ForesightConfig.Constraint.NONE : speedFraction);
        try {
            followPath(path, status);
        } finally {
            Constants.foresightConfig.maxPathSpeed.set(ForesightConfig.Constraint.NONE);
        }
    }

    private void followPath(Path path, String status) {
        follower.follow(path);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && follower.isBusy() && timer.seconds() < PATH_TIMEOUT_SEC) {
            double target = updateRobot();
            addTelemetry(status, target);
            telemetry.update();
        }
    }

    /** Holds the end of the last path (the follower keeps holding) while the turret keeps aiming. */
    private void holdFor(double seconds, String status) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            double target = updateRobot();
            addTelemetry(status, target);
            telemetry.update();
        }
    }

    /** One loop: drive, then aim the turret from the new pose. Returns the turret target angle. */
    private double updateRobot() {
        follower.update();
        Pose pose = follower.pose();
        turret.updateHeading(pose.heading());
        goal.update(pose.y());
        return turret.aimAt(pose.x(), pose.y(), pose.heading(), goal.x(), goal.y());
    }

    private void addTelemetry(String status, double turretTargetDeg) {
        Pose pose = follower.pose();
        telemetry.addData("Status", status);
        telemetry.addData("Pose", "(%.1f, %.1f) %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Aiming at", "%s cell (%.1f, %.1f)", goal.aimingTop() ? "TOP" : "BOTTOM", goal.x(), goal.y());
        double cur = turret.angleDeg();
        telemetry.addData("Turret", "%.1f°  (limits %.0f .. %.0f)", cur, TurretConstants.MIN_DEG, TurretConstants.MAX_DEG);
        if (!Double.isNaN(turretTargetDeg)) {
            telemetry.addData("Turret target", "%.1f°  error %+.1f°", turretTargetDeg, turretTargetDeg - cur);
        }
    }
}
