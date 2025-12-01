package org.firstinspires.ftc.teamcode.decode.teleOp.align;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.decode.teleOp.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@TeleOp(name="TeleOp – Odo Distance Pivot")
public class TeleOdoPivot extends LinearOpMode {

    // Start pose same as your AlignToRed code
    public static final Pose START_POSE = new Pose(
            116.6988847583643,
            128.83271375464685,
            Math.toRadians(225)
    );

    private Follower follower;             // Pedro v2 localizer
    private CustomMecanumDrive drive;      // Manual driving
    private Servo pivotServo;

    // Servo positions
    private static final double POS_CLOSE = 0.6;
    private static final double POS_MED   = 0.7028;
    private static final double POS_FAR   = 0.735;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        drive = new CustomMecanumDrive(hardwareMap);

        pivotServo = hardwareMap.get(Servo.class, "launchServo");

        // Set Pedro’s odometry reference origin
        follower.setPose(START_POSE);

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            follower.update();   // REQUIRED for odometry

            Pose pose = follower.getPose();

            // ----------------------------
            // Compute distance from START
            // ----------------------------
            double dx = pose.getX() - START_POSE.getX();
            double dy = pose.getY() - START_POSE.getY();

            // Pedro uses inches → convert to cm afterward
            double distIn = Math.sqrt(dx*dx + dy*dy);
            double distCm = distIn * 2.54;

            // ----------------------------
            // Auto pivot servo based on odometry
            // ----------------------------
            if (distCm <= 80) {
                pivotServo.setPosition(POS_CLOSE);
            } else if (distCm <= 200) {
                pivotServo.setPosition(POS_MED);
            } else {
                pivotServo.setPosition(POS_FAR);
            }

            // ----------------------------
            // Manual driving
            // ----------------------------
            drive.driveMecanum(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x
            );

            // ----------------------------
            // Telemetry
            // ----------------------------
            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Dist Cm", distCm);
            telemetry.addData("Pivot Servo", pivotServo.getPosition());
            telemetry.update();
        }
    }
}