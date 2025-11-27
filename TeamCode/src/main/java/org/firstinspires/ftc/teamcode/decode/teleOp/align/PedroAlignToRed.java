package org.firstinspires.ftc.teamcode.decode.teleOp.align;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class PedroAlignToRed extends OpMode {
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedRotation;
    private Supplier<PathChain> rotationPath;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    // Fixed goal point on the field (adjust these coordinates as needed)
    private static final double GOAL_X = 130.0;
    private static final double GOAL_Y = 130.0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        // Lazy path generation for rotation - creates a point turn at current position
        rotationPath = () -> {
            Pose currentPose = follower.getPose();
            double targetHeading = calculateHeadingToGoal();

            return follower.pathBuilder()
                    .addPath(new Path(new BezierLine(
                            new Pose(currentPose.getX(), currentPose.getY()),
                            new Pose(currentPose.getX(), currentPose.getY())
                    )))
                    .setConstantHeadingInterpolation(targetHeading)
                    .build();
        };
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        if (!automatedRotation) {
            // Normal teleop driving
            if (!slowMode) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        true
                );
            } else {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * slowModeMultiplier,
                        -gamepad1.left_stick_x * slowModeMultiplier,
                        -gamepad1.right_stick_x * slowModeMultiplier,
                        true
                );
            }
        }

        // Rotate to face goal point when A is pressed
        if (gamepad1.aWasPressed()) {
            follower.followPath(rotationPath.get(), true);
            automatedRotation = true;
        }

        // Stop automated rotation if done or B is pressed
        if (automatedRotation && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedRotation = false;
        }

        // Slow Mode toggle
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        // Slow mode strength adjustment
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }
        if (gamepad2.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }

        // Standard FTC Telemetry
        Pose currentPose = follower.getPose();
        double targetHeading = calculateHeadingToGoal();

        telemetry.addData("--- Position ---", "");
        telemetry.addData("X", String.format("%.2f", currentPose.getX()));
        telemetry.addData("Y", String.format("%.2f", currentPose.getY()));
        telemetry.addData("Heading (deg)", String.format("%.2f", Math.toDegrees(currentPose.getHeading())));
        telemetry.addData("", "");
        telemetry.addData("--- Goal ---", "");
        telemetry.addData("Goal Point", String.format("(%.1f, %.1f)", GOAL_X, GOAL_Y));
        telemetry.addData("Target Heading (deg)", String.format("%.2f", Math.toDegrees(targetHeading)));
        telemetry.addData("Heading Error (deg)", String.format("%.2f",
                Math.toDegrees(normalizeAngle(targetHeading - currentPose.getHeading()))));
        telemetry.addData("", "");
        telemetry.addData("--- Status ---", "");
        telemetry.addData("Mode", automatedRotation ? "AUTO ROTATE" : "MANUAL");
        telemetry.addData("Slow Mode", slowMode);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("", "");
        telemetry.addData("--- Controls ---", "");
        telemetry.addData("A", "Rotate to Goal");
        telemetry.addData("B", "Cancel Rotation");
        telemetry.addData("RB", "Toggle Slow Mode");
        telemetry.update();
    }

    /**
     * Calculate the heading angle needed to face the goal point from current position
     */
    private double calculateHeadingToGoal() {
        Pose currentPose = follower.getPose();
        double deltaX = GOAL_X - currentPose.getX();
        double deltaY = GOAL_Y - currentPose.getY();
        return Math.atan2(deltaY, deltaX);
    }

    /**
     * Normalize angle to -PI to PI range
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}