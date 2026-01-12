package org.firstinspires.ftc.teamcode.decode.pedroPathing.autov2;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.pedroPathing.autov2.blueback.GeneratedPathsBlueBackV4;
import org.firstinspires.ftc.teamcode.decode.pedroPathing.autov2.blueback.BlueBackV4;
import org.firstinspires.ftc.teamcode.decode.teleOp.CustomMecanumDrive;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "!!! Leave Either Color")
public class Leave extends LinearOpMode {

    // ---------------- DRIVE ----------------
    private Follower follower;
    private GeneratedPathsLeave paths;
    private CustomMecanumDrive drivetrain;

    // ---------------- RUN ----------------
    @Override
    public void runOpMode() throws InterruptedException {


        initHardware();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(GeneratedPathsLeave.START_POSE);
        paths = new GeneratedPathsLeave(follower);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        runPath(paths.exit(), 250, 0.5);

        telemetry.addLine("Finished");
        telemetry.update();
    }

    // ---------------- PATH HELPERS ----------------
    private void runPath(PathChain path, int stopDelay, double speed) {
        follower.setMaxPower(speed);
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) follower.update();
        follower.breakFollowing();
        if (stopDelay > 0) sleep(stopDelay);
    }

    // ---------------- INIT ----------------
    private void initHardware() {
        drivetrain = new CustomMecanumDrive(hardwareMap);
    }
}