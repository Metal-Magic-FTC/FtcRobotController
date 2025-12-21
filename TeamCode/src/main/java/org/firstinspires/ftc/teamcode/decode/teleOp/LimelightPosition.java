package org.firstinspires.ftc.teamcode.decode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

@TeleOp(name="!Limelight Manual Test")
public class LimelightPosition extends LinearOpMode {

    // ts constants
    private static final double START_X = 116.6988847583643;
    private static final double START_Y = 128.83271375464685;
    private static final double START_HEADING =
            Math.toRadians(225);
    private CustomMecanumDrive drivetrain;

    private CustomTurret turret;

    private CustomLimelight limelight;
    private Follower follower;

    private Pose startPose = new Pose(
            START_X,
            START_Y,
            START_HEADING
    );

    private Pose target = new Pose(
            136,
            136,
            Math.toRadians(225)
    );

    private Pose target2 = new Pose(
            0,
            0,
            Math.toRadians(0)
    );

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(); // initializing everything

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        waitForStart(); // waiting until driver clicks play button

        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            drivetrain.driveMecanum(strafe, drive, turn);


            limelight.getLimelightPose();

            telemetry.addData("limelight position", limelight);


            follower.update();

            telemetry.update();
        }
    }

    public void initialize() {

        // fl - 0, fr - 1, bl - 2, br - 3
        drivetrain = new CustomMecanumDrive(hardwareMap);

        turret = new CustomTurret(hardwareMap);

        limelight = new CustomLimelight(hardwareMap);

    }

}
