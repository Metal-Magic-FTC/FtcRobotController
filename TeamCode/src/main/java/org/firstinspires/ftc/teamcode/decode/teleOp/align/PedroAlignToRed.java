package org.firstinspires.ftc.teamcode.decode.teleOp.align;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

@Configurable
@TeleOp
public class PedroAlignToRed extends OpMode {
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedRotation;
    private TelemetryManager telemetryM;
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
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

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
            double targetHeading = calculateHeadingToGoal();
            follower.turnTo(targetHeading);
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

        // Telemetry
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedRotation", automatedRotation);
        telemetryM.debug("targetHeading", Math.toDegrees(calculateHeadingToGoal()));
        telemetryM.debug("goalPoint", "(" + GOAL_X + ", " + GOAL_Y + ")");
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
}