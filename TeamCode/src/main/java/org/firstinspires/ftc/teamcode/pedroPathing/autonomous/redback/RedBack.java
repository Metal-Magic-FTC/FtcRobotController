package org.firstinspires.ftc.teamcode.pedroPathing.autonomous.redback;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RedBack Auto", group = "Auto")
public class RedBack extends LinearOpMode {

    private Follower follower;
    private GeneratedPathsRedBack paths;

    private void runPath(PathChain path, int stopDelayMs) {
        follower.followPath(path);

        // Run until the path is finished
        while (opModeIsActive() && !isStopRequested() && follower.isBusy()) {
            follower.update();
        }

        // ✅ Explicit stop
        follower.breakFollowing();  // or follower.stop() if your API uses that

        // ✅ Optional short pause
        if (stopDelayMs > 0) {
            sleep(stopDelayMs);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);

        // ✅ Apply the start pose from GeneratedPathsRedBack
        follower.setPose(GeneratedPathsRedBack.START_POSE);

        // Load paths
        paths = new GeneratedPathsRedBack(follower);

        telemetry.addLine("Ready to start RedBack Auto");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Sequence of autonomous (each with a stop + 250ms pause)
        runPath(paths.shoot(), 250);
        runPath(paths.toIntake1(), 250);
        runPath(paths.intake1(), 250);
        runPath(paths.shoot2(), 250);
        runPath(paths.toIntake2(), 250);
        runPath(paths.intake2(), 250);
        runPath(paths.shoot3(), 250);

        // End of auto
        telemetry.addLine("RedBack Auto Finished");
        telemetry.update();
    }
}
