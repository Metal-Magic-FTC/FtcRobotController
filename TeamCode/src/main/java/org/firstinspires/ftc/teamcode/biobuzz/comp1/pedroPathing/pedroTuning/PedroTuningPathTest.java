package org.firstinspires.ftc.teamcode.biobuzz.comp1.pedroPathing.pedroTuning;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedroPathing.Constants;

@Autonomous(name = "BioBuzz PedroTuning Path Test", group = "Auto")
public class PedroTuningPathTest extends LinearOpMode {

    private Follower follower;
    private PedroTuningPaths paths;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(PedroTuningPaths.START_POSE);
        paths = new PedroTuningPaths(follower);

        brakeDriveMotors();

        telemetry.addLine("Ready to start PedroTuning Path Test");
        telemetry.addData("Start Pose", "(%.2f, %.2f) @ %.0f deg",
                PedroTuningPaths.START_POSE.getX(),
                PedroTuningPaths.START_POSE.getY(),
                Math.toDegrees(PedroTuningPaths.START_POSE.getHeading()));
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // (103.880, 135.920) @ 90 deg  ->  (86.535, 108.632) @ 180 deg
        runPath(paths.path1(), 100, 0.8);

        // (86.535, 108.632) @ 180 deg  ->  (99.207, 59.568) @ 0 deg
        runPath(paths.path2(), 100, 0.8);

        telemetry.addLine("PedroTuning Path Test finished");
        telemetry.addData("End Pose", "(%.2f, %.2f) @ %.0f deg",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    // -----------------------------
    // PATH HELPERS
    // -----------------------------
    private void runPath(PathChain path, int stopDelayMs, double speed) {
        follower.setMaxPower(speed);
        follower.followPath(path);

        while (opModeIsActive() && !isStopRequested() && follower.isBusy()) {
            follower.update();

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }

        follower.breakFollowing();
        stopDriveMotors();

        if (stopDelayMs > 0) sleep(stopDelayMs);
    }

    private void stopDriveMotors() {
        for (String m : new String[]{"frontLeft", "frontRight", "backLeft", "backRight"}) {
            DcMotor motor = hardwareMap.get(DcMotor.class, m);
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void brakeDriveMotors() {
        for (String m : new String[]{"frontLeft", "frontRight", "backLeft", "backRight"}) {
            hardwareMap.get(DcMotor.class, m).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}
