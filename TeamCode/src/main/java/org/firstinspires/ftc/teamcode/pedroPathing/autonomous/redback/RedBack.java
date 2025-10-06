package org.firstinspires.ftc.teamcode.pedroPathing.autonomous.redback;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RedBack Auto", group = "Auto")
public class RedBack extends LinearOpMode {

    private Follower follower;
    private GeneratedPathsRedBack paths;

    DcMotor intakeMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        if (isStopRequested()) return;

        intakeMotor.setPower(1);

        // Sequence of autonomous (each with a stop + 250ms pause)
        runPath(paths.shoot(), 250);
        runPath(paths.toIntake1(), 250);
        runPath(paths.intake1(), 250);
        runPath(paths.shoot2(), 250);
        runPath(paths.toIntake2(), 250);
        runPath(paths.intake2(), 250);
        runPath(paths.shoot3(), 250);

        //intakeMotor.setPower(0);

        // End of auto
        telemetry.addLine("RedBack Auto Finished");
        telemetry.update();
    }

    private void initialize() {
        follower = Constants.createFollower(hardwareMap);

        // Apply the start pose from GeneratedPathsRedBack
        follower.setPose(GeneratedPathsRedBack.START_POSE);

        //follower = Constants.createFollower(hardwareMap);

        // Set motors to BRAKE to stop drift when idle
        hardwareMap.get(DcMotor.class, "frontLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "frontRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "backLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "backRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Load paths
        paths = new GeneratedPathsRedBack(follower);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Ready to start RedBack Auto");
        telemetry.update();
    }

    private void runPath(PathChain path, int stopDelayMs) {
        follower.followPath(path);

        // run until path done
        while (opModeIsActive() && !isStopRequested() && follower.isBusy()) {
            follower.update();
        }

        follower.setMaxPower(0.9); // 0.6 of normal power

        follower.breakFollowing();

        // Fully stop all drive motors
        hardwareMap.get(DcMotor.class, "frontLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "frontRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "backLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "backRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "frontLeft").setPower(0);
        hardwareMap.get(DcMotor.class, "frontRight").setPower(0);
        hardwareMap.get(DcMotor.class, "backLeft").setPower(0);
        hardwareMap.get(DcMotor.class, "backRight").setPower(0);

        if (stopDelayMs > 0) sleep(stopDelayMs);
    }

}
