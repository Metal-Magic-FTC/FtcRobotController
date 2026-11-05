package org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.hiveShoot;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.api.Paths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;

@Autonomous(name = "Auto V1 Drive Four Points", group = "BioBuzz")
public class AutoV1CollectAndShoot extends LinearOpMode {

    private static final double PATH_TIMEOUT_SEC = 7.0;

    // Point 1: Start facing left (180 degrees)
    private static final Pose START =
            new Pose(55.819, 8.357, Math.PI);

    // Point 2: Collection location
    private static final Pose COLLECT =
            new Pose(6.242, 6.828);

    // Point 3: Shooting location
    private static final Pose SHOOT =
            new Pose(36.715, 15.663);

    // Point 4: Parking location
    private static final Pose PARK =
            new Pose(8.829, 96.696);

    private Follower follower;

    @Override
    public void runOpMode() {

        follower = Constants.create(hardwareMap);
        follower.setPose(START);

        // Build paths between the four points.
        Path path1 = Paths.line(START, COLLECT);
        Path path2 = Paths.line(COLLECT, SHOOT);
        Path path3 = Paths.line(SHOOT, PARK);

        while (opModeInInit()) {
            follower.update();

            telemetry.addLine("Auto V1 - Four Point Path");
            telemetry.addData("Start", "(%.1f, %.1f)",
                    START.x(), START.y());
            telemetry.addData("Start heading", "180 degrees - facing left");
            telemetry.addLine("Ready to start");
            telemetry.update();
        }

        if (isStopRequested()) {
            return;
        }

        runPath(path1, "Point 1 -> Point 2");
        runPath(path2, "Point 2 -> Point 3");
        runPath(path3, "Point 3 -> Point 4");

        while (opModeIsActive()) {
            follower.update();

            Pose pose = follower.pose();
            telemetry.addLine("Finished route");
            telemetry.addData("Final pose", "(%.1f, %.1f, %.1f°)",
                    pose.x(),
                    pose.y(),
                    Math.toDegrees(pose.heading()));
            telemetry.update();
        }
    }

    private void runPath(Path path, String status) {
        follower.follow(path);

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()
                && follower.isBusy()
                && timer.seconds() < PATH_TIMEOUT_SEC) {

            follower.update();

            Pose pose = follower.pose();

            telemetry.addData("Status", status);
            telemetry.addData("Pose", "(%.1f, %.1f, %.1f°)",
                    pose.x(),
                    pose.y(),
                    Math.toDegrees(pose.heading()));
            telemetry.update();
        }
    }
}