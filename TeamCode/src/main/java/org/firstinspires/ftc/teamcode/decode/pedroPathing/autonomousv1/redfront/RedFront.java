package org.firstinspires.ftc.teamcode.decode.pedroPathing.autonomousv1.redfront;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.teleOp.CustomMecanumDrive;


@Autonomous(name = "! v1 Red Far Auto", group = "Auto")
public class RedFront extends LinearOpMode {

    private Follower follower;
    private GeneratedPathsRedFront paths;
    private CustomMecanumDrive drivetrain;

    // CustomMecanumDrive drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();
        // Initialize path follower
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(GeneratedPathsRedFront.START_POSE);
        paths = new GeneratedPathsRedFront(follower);


        telemetry.addLine("Ready to start RedFront New Auto");
        telemetry.update();


        waitForStart();


        if (isStopRequested()) return;


        // ----------------------
        // 2. Move to shooting position
        // ----------------------
        runPath(paths.shoot(), 50, 1);

        // ----------------------
        // 4. Continue auto sequence
        // ----------------------
        runPath(paths.toIntake1(), 50, 0.75);

        runPath(paths.shoot2(), 50, 0.75);


        runPath(paths.toIntake2(), 50, 0.75);


        runPath(paths.shoot3(), 50, 0.75);

        // End of auto
        telemetry.addLine("Red Front Auto Finished");
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
        }


        follower.breakFollowing();
        stopDriveMotors();


        if (stopDelayMs > 0) sleep(stopDelayMs);
    }


    private void stopDriveMotors() {
        String[] driveMotors = {"frontLeft", "frontRight", "backLeft", "backRight"};
        for (String m : driveMotors) {
            DcMotor motor = hardwareMap.get(DcMotor.class, m);
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }


    private void initializeHardware() {

        drivetrain = new CustomMecanumDrive(hardwareMap);

    }

    private void initialize() {
        follower = Constants.createFollower(hardwareMap);

        // Apply the start pose from GeneratedPathsRedFront
        follower.setPose(GeneratedPathsRedFront.START_POSE);

        //follower = Constants.createFollower(hardwareMap);

        // Set motors to BRAKE to stop drift when idle
        hardwareMap.get(DcMotor.class, "frontLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "frontRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "backLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "backRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Load paths
        paths = new GeneratedPathsRedFront(follower);

        telemetry.addLine("Ready to start RedFront Auto");
        telemetry.update();
    }
}
