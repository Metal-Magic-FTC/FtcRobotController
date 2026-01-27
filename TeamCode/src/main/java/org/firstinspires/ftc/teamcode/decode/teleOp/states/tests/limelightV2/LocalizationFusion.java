package org.firstinspires.ftc.teamcode.decode.teleOp.states.tests.limelightV2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.decode.teleOp.tests.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.GoBildaPinpointDriver;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

import java.util.List;
import java.util.function.Supplier;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name="!!Localization Fusion Test")

public class LocalizationFusion extends LinearOpMode {

    private CustomMecanumDrive drivetrain;

    private FusedPose fusedPose;
    private Pose startingPose;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(); // initializing everything

        waitForStart(); // waiting until driver clicks play button

        while (opModeIsActive()) {
            fusedPose.update();

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            drivetrain.driveMecanum(strafe, drive, turn);

            Follower follower = fusedPose.getFollower();

            telemetry.addLine("--- Follower Pose ---");
            telemetry.addData("  X", "%.3f in", (follower != null) ? follower.getPose().getX() : null);
            telemetry.addData("  Y", "%.3f in", (follower != null) ? follower.getPose().getY() : null);
            telemetry.addData("  H", "%.1f°", (follower != null) ? Math.toDegrees(follower.getPose().getHeading()) : null);

            Pose rawPose = fusedPose.getRobotPose(false);
            telemetry.addLine("--- Limelight Raw Pose ---");
            telemetry.addData("  X", "%.3f in", (rawPose != null) ? rawPose.getPose().getX() : null);
            telemetry.addData("  Y", "%.3f in", (rawPose != null) ? rawPose.getPose().getY() : null);
            telemetry.addData("  H", "%.1f°", (rawPose != null) ? Math.toDegrees(rawPose.getPose().getHeading()) : null);

            Pose convertedPose = fusedPose.getRobotPose(true);
            telemetry.addLine("--- Limelight Converted Pose ---");
            telemetry.addData("  X", "%.3f in", (convertedPose != null) ? convertedPose.getPose().getX() : null);
            telemetry.addData("  Y", "%.3f in", (convertedPose != null) ? convertedPose.getPose().getY() : null);
            telemetry.addData("  H", "%.1f°", (convertedPose != null) ? Math.toDegrees(convertedPose.getPose().getHeading()) : null);

            Pose mergedPose = fusedPose.mergePoses(true);
            telemetry.addLine("--- Merged Pose ---");
            telemetry.addData("  X", "%.3f in", (mergedPose != null) ? mergedPose.getPose().getX() : null);
            telemetry.addData("  Y", "%.3f in", (mergedPose != null) ? mergedPose.getPose().getY() : null);
            telemetry.addData("  H", "%.1f°", (mergedPose != null) ? Math.toDegrees(mergedPose.getPose().getHeading()) : null);

            telemetry.update();
        }
    }

    public void initialize() {

        // fl - 0, fr - 1, bl - 2, br - 3
        drivetrain = new CustomMecanumDrive(hardwareMap);

        startingPose = new Pose(
                116,
                128,
                Math.toRadians(225)
        );
        fusedPose = new FusedPose(hardwareMap, startingPose);

    }

}
