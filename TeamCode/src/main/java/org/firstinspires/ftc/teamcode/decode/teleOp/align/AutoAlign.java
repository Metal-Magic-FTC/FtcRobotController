package org.firstinspires.ftc.teamcode.decode.teleOp.align;
import com.bylazar.configurables.annotations.Configurable;
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

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

import java.util.function.Supplier;

@TeleOp(name = "Auto Alignment", group = "Test")
public class AutoAlign extends OpMode {

    enum OdoState {
        IDLE,
        NAVIGATING,
        FACING_TARGET
    }

    OdoState odoState = OdoState.IDLE;

    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        // WHY DID I DEFINE THE STARTING POSE AFTER CREATING THE FOLLOWER??
        startingPose = new Pose(
                116,
                128,
                Math.toRadians(225)
        );
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(100, 100))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0), 0.8))
                .build();

    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetry.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }

        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
//            follower.followPath(pathChain.get());
            Pose curr = follower.getPose();
            double dx = 136 - curr.getX();
            double dy = 136 - curr.getY();
            double targetAngle = Math.atan2(dy, dx);
//            follower.followPath(new Path(new BezierPoint(new Pose(curr.getX(), curr.getY(), targetAngle))));
            follower.turnTo(targetAngle);
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }

        //Optional way to change slow mode strength
        if (gamepad2.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }

        telemetry.addLine("--- Follower Pose ---");
        telemetry.addData("  X", follower.getPose().getX());
        telemetry.addData("  Y", follower.getPose().getY());
        telemetry.addData("  Heading", Math.toDegrees(follower.getPose().getHeading()));
        double dx = 136 - follower.getPose().getX();
        double dy = 136 - follower.getPose().getY();
        double targetAngle = Math.atan2(dy, dx);
        telemetry.addData("angle", Math.toDegrees(targetAngle));
        telemetry.addData("automatedDrive", automatedDrive);
        if (automatedDrive) {
            telemetry.addData("heading error", follower.getHeadingError());
        }
    }
}