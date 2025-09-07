package org.firstinspires.ftc.teamcode.decode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Wheel directions")
public class DrivetrainDirections extends LinearOpMode {
    DcMotor leftFront = null;
    DcMotor leftBack = null;
    DcMotor rightFront = null;
    DcMotor rightBack = null;

    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotor.class, "backLeft");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        rightBack = hardwareMap.get(DcMotor.class, "backRight");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                leftFront.setPower(0.5);
            } else {
                leftFront.setPower(0);
            }

            if (gamepad1.b) {
                leftBack.setPower(0.5);
            } else {
                leftBack.setPower(0);
            }

            if (gamepad1.x) {
                rightFront.setPower(0.5);
            } else {
                rightFront.setPower(0);
            }

            if (gamepad1.y) {
                rightBack.setPower(0.5);
            } else {
                rightBack.setPower(0);
            }
        }
    }
}
