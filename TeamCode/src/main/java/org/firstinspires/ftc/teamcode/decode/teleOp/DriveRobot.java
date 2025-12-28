package org.firstinspires.ftc.teamcode.decode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="!Drivetrain Manual Test")

public class DriveRobot extends LinearOpMode {

    private CustomMecanumDrive drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(); // initializing everything

        waitForStart(); // waiting until driver clicks play button

        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            drivetrain.driveMecanum(strafe, drive, turn);

            telemetry.update();
        }
    }

    public void initialize() {

        // fl - 0, fr - 1, bl - 2, br - 3
        drivetrain = new CustomMecanumDrive(hardwareMap);

    }

}
