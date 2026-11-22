package org.firstinspires.ftc.teamcode.biobuzz.comp1.extraPaths;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;

@Autonomous(name = "wait for the airball mate", group = "BioBuzz")
public class mat8AirballAuto extends LinearOpMode {
    private static final double WAIT_FOR_PARTNER_SEC = 10.0;
    private static final double PATH_TIMEOUT_SEC = 8.0;
    private static final double SETTLE_SEC = 0.3;
    private static final double SHOOT_SEC = 3.0;

    private Follower follower;

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        follower.setPose(mat8AirballPath.START_POSE);
        Path toShoot = new mat8AirballPath().driveToShoot();

        telemetry.addLine("BioBuzz Auto V2 (red, far-wall start) ready");
        telemetry.addData("Start pose", "(%.1f, %.1f) %.0f°", mat8AirballPath.START_POSE.x(),
                mat8AirballPath.START_POSE.y(), Math.toDegrees(mat8AirballPath.START_POSE.heading()));
        telemetry.addData("Shoot pose", "(%.1f, %.1f) %.0f°", mat8AirballPath.SHOOT_POSE.x(),
                mat8AirballPath.SHOOT_POSE.y(), Math.toDegrees(mat8AirballPath.SHOOT_POSE.heading()));
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        hold(WAIT_FOR_PARTNER_SEC, "Waiting for partner");
        runPath(toShoot, "Driving to shoot spot");
        hold(SETTLE_SEC, "Settling");
        shootPreloads();

        while (opModeIsActive()) {
            follower.update();
            addPoseTelemetry("Done");
            telemetry.update();
        }
    }
    private void shootPreloads() {
        hold(SHOOT_SEC, "Shooting");
    }

    private void runPath(Path path, String status) {
        follower.follow(path);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && follower.isBusy() && timer.seconds() < PATH_TIMEOUT_SEC) {
            follower.update();
            addPoseTelemetry(status);
            telemetry.update();
        }
    }

    /** Stand still for a fixed time, keeping the localizer updating. */
    private void hold(double seconds, String status) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            follower.update();
            addPoseTelemetry(status);
            telemetry.addData("Timer", "%.1f / %.1f s", timer.seconds(), seconds);
            telemetry.update();
        }
    }

    private void addPoseTelemetry(String status) {
        Pose pose = follower.pose();
        telemetry.addData("Status", status);
        telemetry.addData("Pose", "(%.1f, %.1f) %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }
}