package org.firstinspires.ftc.teamcode.decode.pedroPathing.autonomous.redmiddle;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

@Autonomous(name = "RedMiddle Auto", group = "Auto")
@Disabled
public class RedMiddle extends LinearOpMode {

    private Follower follower;
    private GeneratedPathsRedMiddle paths;

    DcMotor intakeMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        if (isStopRequested()) return;

        intakeMotor.setPower(0);

        // Sequence of autonomous (each with a stop + 250ms pause)
        runPath(paths.shoot(), 250, 0.75);

        runPath(paths.toIntake1(), 250, 0.75);
        intakeMotor.setPower(1);

        runPath(paths.intake1(), 250, 0.5);
        intakeMotor.setPower(0);

        runPath(paths.shoot2(), 250, 0.75);

        runPath(paths.toIntake2(), 250, 0.75);
        intakeMotor.setPower(1);

        runPath(paths.intake2(), 250, 0.5);
        intakeMotor.setPower(0);

        runPath(paths.shoot3(), 250, 0.75);

        //intakeMotor.setPower(0);

        // End of auto
        telemetry.addLine("RedMiddle Auto Finished");
        telemetry.update();
    }

    private void initialize() {
        follower = Constants.createFollower(hardwareMap);

        // Apply the start pose from GeneratedPathsRedMiddle
        follower.setPose(GeneratedPathsRedMiddle.START_POSE);

        //follower = Constants.createFollower(hardwareMap);

        // Set motors to BRAKE to stop drift when idle
        hardwareMap.get(DcMotor.class, "frontLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "frontRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "backLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "backRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Load paths
        paths = new GeneratedPathsRedMiddle(follower);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Ready to start RedMiddle Auto");
        telemetry.update();
    }

    private void runPath(PathChain path, int stopDelayMs, double speed) {

        follower.setMaxPower(speed); // 0.6 of normal power

        follower.followPath(path);

        // run until path done
        while (opModeIsActive() && !isStopRequested() && follower.isBusy()) {
            follower.update();
        }


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
