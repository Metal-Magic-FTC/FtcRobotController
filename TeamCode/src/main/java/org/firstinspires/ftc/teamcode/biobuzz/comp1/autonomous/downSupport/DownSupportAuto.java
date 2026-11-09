package org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.downSupport;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.util.BulkReader;

/**
 * Down Support auto (RED), paths in PathsDownSupport:
 *   1. Drive across the field, shoot preloads      (modelled as a SHOOT_SEC wait)
 *   2. Ram into the corner to pick up more pollen
 *   3. Drive back to the starting side, shoot       (SHOOT_SEC wait)
 *   4. Drive to the FLOWER, shoot                   (SHOOT_SEC wait)
 *   5. Park
 *
 * BEFORE PRESSING INIT: robot at PathsDownSupport.start, (31.0, 132.2) facing 270 deg (toward the audience), and
 * still during INIT (Pinpoint IMU calibrates).
 */
@Autonomous(name = "BioBuzz Down Support Auto", group = "BioBuzz")
public class DownSupportAuto extends LinearOpMode {

    /** Placeholder for the shooter until the subsystem exists. */
    private static final double SHOOT_SEC = 2.0;
    /** Give up on a path after this long so a stuck robot still tries the rest. */
    private static final double PATH_TIMEOUT_SEC = 6.0;
    /**
     * The corner path ends against the walls, so Pedro may never call it finished. Stop pushing after this long
     * instead of burning the full PATH_TIMEOUT_SEC.
     */
    private static final double RAM_TIMEOUT_SEC = 2.5;

    private BulkReader bulk;
    private Follower follower;
    private final ElapsedTime matchTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        bulk = new BulkReader(hardwareMap);
        telemetry.addLine("Calibrating Pinpoint - keep the robot STILL");
        telemetry.update();
        follower = Constants.create(hardwareMap);
        follower.setPose(PathsDownSupport.start);
        PathsDownSupport paths = new PathsDownSupport();
        Path toFarShoot = paths.toFarShoot();
        Path toCorner = paths.toCorner();
        Path toHomeShoot = paths.toHomeShoot();
        Path toFlower = paths.toFlower();
        Path toPark = paths.toPark();

        while (opModeInInit()) {
            update();
            telemetry.addLine("Down Support Auto ready. Start at (31.0, 132.2) facing 270°.");
            addTelemetry("Waiting for START");
            telemetry.update();
        }
        if (isStopRequested()) return;
        matchTimer.reset();

        runPath(toFarShoot, "Driving across the field", PATH_TIMEOUT_SEC);
        hold(SHOOT_SEC, "Shooting preloads");
        runPath(toCorner, "Ramming the corner for pollen", RAM_TIMEOUT_SEC);
        runPath(toHomeShoot, "Driving back to the starting side", PATH_TIMEOUT_SEC);
        hold(SHOOT_SEC, "Shooting corner pollen");
        runPath(toFlower, "Driving to FLOWER", PATH_TIMEOUT_SEC);
        hold(SHOOT_SEC, "Shooting FLOWER pollen");
        runPath(toPark, "Driving to park", PATH_TIMEOUT_SEC);

        while (opModeIsActive()) {
            update();
            addTelemetry("Parked");
            telemetry.update();
        }
    }

    /** One loop of hardware: fresh bulk read first, then odometry + path following. */
    private void update() {
        bulk.update();
        follower.update();
    }

    private void runPath(Path path, String status, double timeoutSec) {
        follower.follow(path);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && follower.isBusy() && timer.seconds() < timeoutSec) {
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
    }
}
