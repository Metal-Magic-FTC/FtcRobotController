package org.firstinspires.ftc.teamcode.decode.teleOp.align;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.teleOp.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

@TeleOp(name="!!!!!!!TeleOp – ALIGN TO MY RED LOOKING AHH")
public class AlignToRed extends LinearOpMode {

    public static final Pose START_POSE = new Pose(
            116.6988847583643,
            128.83271375464685,
            Math.toRadians(225)
    );

    private CustomMecanumDrive drivetrain;
    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        drivetrain = new CustomMecanumDrive(hardwareMap);

        follower.setPose(START_POSE);

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --------------------------
            // RUN ALIGNMENT ON 'A'
            // --------------------------
            if (gamepad1.a) {
                runAlignmentRoutine();
            }

            // --------------------------
            // NORMAL TELEOP DRIVE
            // --------------------------
            double drive  = -gamepad1.left_stick_y;
            double strafe =  gamepad1.left_stick_x;
            double turn   =  gamepad1.right_stick_x;

            drivetrain.driveMecanum(strafe, drive, turn);

            follower.update();
        }
    }

    /**
     * Blocks teleop until robot is facing the START_POSE point
     */
    private void runAlignmentRoutine() {
        Pose current = follower.getPose();

        double dx = START_POSE.getX() - current.getX();
        double dy = START_POSE.getY() - current.getY();

        double desiredHeading = Math.atan2(dy, dx);
        double currentHeading = current.getHeading();

        double delta = normalizeAngle(desiredHeading - currentHeading);
        boolean turnLeft = delta > 0;
        double angleToTurn = Math.abs(delta);

        // stop teleop control immediately
        drivetrain.driveMecanum(0, 0, 0);

        // ---- BLOCKING TURN (Pedro v2) ----
        follower.turn(angleToTurn, turnLeft);

        // ---- WAIT UNTIL WITHIN ±3° ----
        while (opModeIsActive()) {
            double err = normalizeAngle(desiredHeading - follower.getPose().getHeading());
            if (Math.abs(err) < Math.toRadians(3)) break;

            follower.update();
        }

        // teleop returns automatically after method ends
    }

    private double normalizeAngle(double ang) {
        while (ang <= -Math.PI) ang += 2.0 * Math.PI;
        while (ang > Math.PI)   ang -= 2.0 * Math.PI;
        return ang;
    }
}