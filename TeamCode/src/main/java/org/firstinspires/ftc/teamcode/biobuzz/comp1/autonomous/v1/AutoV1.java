package org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.v1;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedroPathing.Constants;

/**
 * Auto V1: drives the 6 paths from my_path2.pp in order. Add mechanism actions between runPath calls.
 */
@Autonomous(name = "BioBuzz Auto V1", group = "BioBuzz")
public class AutoV1 extends LinearOpMode {

    /** Give up on a path after this long so a stuck robot still runs the rest of the auto. */
    private static final double PATH_TIMEOUT_SEC = 6.0;

    private Follower follower;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PathsV1.START_POSE);
        PathsV1 paths = new PathsV1(follower);

        // Build everything during init so start is instant
        PathChain[] sequence = {
                paths.path1(), paths.path2(), paths.path3(),
                paths.path4(), paths.path5(), paths.path6()
        };

        telemetry.addLine("BioBuzz Auto V1 ready");
        telemetry.addData("Start pose", "(%.1f, %.1f) %.0f°", PathsV1.START_POSE.getX(),
                PathsV1.START_POSE.getY(), Math.toDegrees(PathsV1.START_POSE.getHeading()));
        telemetry.addData("Visualizer reverse headings", PathsV1.FOLLOW_VISUALIZER_REVERSE);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        for (int i = 0; i < sequence.length && opModeIsActive(); i++) {
            runPath(sequence[i], i + 1);
        }

        while (opModeIsActive()) {
            follower.update();
            addPoseTelemetry("Done");
            telemetry.update();
        }
    }

    private void runPath(PathChain path, int number) {
        follower.followPath(path, true);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && follower.isBusy() && timer.seconds() < PATH_TIMEOUT_SEC) {
            follower.update();
            addPoseTelemetry("Path " + number + " / 6");
            telemetry.update();
        }
    }

    private void addPoseTelemetry(String status) {
        Pose pose = follower.getPose();
        telemetry.addData("Status", status);
        telemetry.addData("Pose", "(%.1f, %.1f) %.1f°", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }
}
