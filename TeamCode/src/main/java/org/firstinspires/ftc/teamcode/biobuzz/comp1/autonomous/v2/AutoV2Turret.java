package org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.v2;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.RedHiveGoal;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.turret.TurretController;

/**
 * Auto V2 (RED): drives the 5 visualizer paths in PathsV2 while the turret keeps aiming at the red HIVE
 * (bottom cell in the bottom half of the field, top cell in the top half) the whole time, using the follower's pose.
 *
 * BEFORE PRESSING INIT
 *   1. Robot at PathsV2.start: (38.2, 8.1) facing 180 deg = facing the RED wall, left side against the audience wall.
 *   2. Turret pointing straight out the robot FRONT (toward the red wall). That is turret 0 deg.
 *   3. Robot still during INIT (Pinpoint IMU calibrates).
 *
 * Add mechanism actions (shooting, intake) in the waits between paths (WAIT_AFTER_SEC).
 */
@Autonomous(name = "BioBuzz Auto V2 + Turret", group = "BioBuzz")
public class AutoV2Turret extends LinearOpMode {

    /** Give up on a path after this long so a stuck robot still runs the rest of the auto. */
    private static final double PATH_TIMEOUT_SEC = 6.0;
    /** Pause at the end of each path, seconds (one per path, same order). The robot holds position meanwhile. */
    private static final double[] WAIT_AFTER_SEC = {0, 0, 0, 0, 0.0};

    private Follower follower;
    private TurretController turret;
    private RedHiveGoal goal;

    @Override
    public void runOpMode() {
        turret = new TurretController(hardwareMap);
        follower = Constants.create(hardwareMap);
        follower.setPose(PathsV2.start);
        goal = new RedHiveGoal(PathsV2.start.y());
        PathsV2 paths = new PathsV2();

        // Build everything during init so start is instant
        Path[] sequence = {
                paths.path11Shootpreloadsthensweepgarden(),
                paths.path22Runtofarwallflower(),
                paths.path33Toleftwallflower(),
                paths.path44Dropintonearband(),
                paths.path55Parkinloadingzone()
        };
        String[] names = {"1 shoot preloads, sweep garden", "2 run to far wall flower", "3 to left wall flower",
                "4 drop into near band", "5 park in loading zone"};

        // Odometry runs during INIT so you can check the start pose; the turret stays unpowered until START.
        while (opModeInInit()) {
            follower.update();
            telemetry.addLine("BioBuzz Auto V2 + Turret ready. Turret facing robot FRONT?");
            addTelemetry("Waiting for start", Double.NaN);
            telemetry.update();
        }
        if (isStopRequested()) return;

        for (int i = 0; i < sequence.length && opModeIsActive(); i++) {
            runPath(sequence[i], "Path " + names[i]);
            waitAt(WAIT_AFTER_SEC[i], "Waiting after " + names[i]);
        }

        while (opModeIsActive()) {
            double target = updateRobot();
            addTelemetry("Done", target);
            telemetry.update();
        }
        turret.stop();
    }

    private void runPath(Path path, String status) {
        follower.follow(path);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && follower.isBusy() && timer.seconds() < PATH_TIMEOUT_SEC) {
            double target = updateRobot();
            addTelemetry(status, target);
            telemetry.update();
        }
    }

    /** Holds the end of the last path (the follower keeps holding) while the turret keeps aiming. */
    private void waitAt(double seconds, String status) {
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
