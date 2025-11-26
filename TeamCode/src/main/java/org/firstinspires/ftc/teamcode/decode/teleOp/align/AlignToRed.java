package org.firstinspires.ftc.teamcode.decode.teleOp.align;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.teleOp.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

@TeleOp(name="TeleOp – ALIGN ROTATE ONLY")
public class AlignToRed extends LinearOpMode {

    public static final Pose START_POSE = new Pose(
            116.6988847583643,
            128.83271375464685,
            Math.toRadians(225)
    );

    private CustomMecanumDrive drive;
    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        drive = new CustomMecanumDrive(hardwareMap);

        follower.setPose(START_POSE);

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ALIGN TO START ON 'A'
            if (gamepad1.a) {
                alignToStartPureRotate();
            }

            // REGULAR TELEOP
            drive.driveMecanum(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x
            );

            follower.update();
        }
    }

    private void alignToStartPureRotate() {

        while (opModeIsActive() && gamepad1.a) {

            Pose pose = follower.getPose();

            // vector from robot → start pos
            double dx = START_POSE.getX() - pose.getX();
            double dy = START_POSE.getY() - pose.getY();

            double desiredHeading = Math.atan2(dy, dx);
            double currentHeading = pose.getHeading();

            double error = normalize(desiredHeading - currentHeading);

            // DONE?
            if (Math.abs(error) < Math.toRadians(2)) {
                drive.driveMecanum(0, 0, 0);
                return;
            }

            // proportional turn speed
            double kP = 0.9;
            double turnPower = kP * error;

            // clamp to safe turn speed
            turnPower = Math.max(-0.6, Math.min(0.6, turnPower));

            // **PURE ROTATION ONLY**
            drive.driveMecanum(0, 0, turnPower);

            follower.update();
        }

        // stop if button released
        drive.driveMecanum(0, 0, 0);
    }

    private double normalize(double a) {
        while (a <= -Math.PI) a += 2 * Math.PI;
        while (a > Math.PI) a -= 2 * Math.PI;
        return a;
    }
}