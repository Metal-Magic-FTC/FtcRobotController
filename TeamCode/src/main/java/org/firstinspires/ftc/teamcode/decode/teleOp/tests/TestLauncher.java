package org.firstinspires.ftc.teamcode.decode.teleOp.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled@TeleOp
public class TestLauncher extends LinearOpMode {

    DcMotorEx launchMotor = null;

//    public Servo launchGate = null;

    double energy = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(); // initializing everything
        waitForStart(); // waiting until driver clicks play button

        while (opModeIsActive()) {


//            if (gamepad1.a) {
//                launchMotor.setPower(1);
//            } else if (gamepad1.b) {
//                launchMotor.setPower(-1);
//            } else {
//                launchMotor.setPower(0);
//            }

//            if (gamepad1.a) {
//                launchMotor.setVelocity(200);
//            } else if (gamepad1.b) {
//                launchMotor.setVelocity(100);
//            } else {
//                launchMotor.setVelocity(0);
//            }

            if (gamepad1.right_bumper) {
                energy += 1;
            }

            if (gamepad1.left_bumper) {
                energy -= 1;
            }

            if (gamepad1.a) {
                launchMotor.setVelocity(energy);
            } else {
                launchMotor.setVelocity(0);
            }

            telemetry.addData("energy", energy);
            telemetry.addData("speed", launchMotor.getVelocity());
            telemetry.addData("power", launchMotor.getPower());

            telemetry.update();
        }
    }

    public void initialize() {

        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");

        launchMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        launchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

//        launchGate = hardwareMap.servo.get("leftGate");
//        launchGate.setPosition(1);

    }

}
