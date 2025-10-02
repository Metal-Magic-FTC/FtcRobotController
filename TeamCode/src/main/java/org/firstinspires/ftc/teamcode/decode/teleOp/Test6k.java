package org.firstinspires.ftc.teamcode.decode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="!Test 6k rpm")
public class Test6k extends LinearOpMode {

    DcMotor leftLaunch = null;
    DcMotor rightLaunch = null;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(); // initializing everything

        waitForStart(); // waiting until driver clicks play button

        double speed = 1.0;

        boolean upWasPressed = false;
        boolean downWasPressed = false;

        boolean upPressed = false;
        boolean downPressed = false;

        while (opModeIsActive()) {

            if (gamepad1.a) {
                leftLaunch.setPower(speed);
                rightLaunch.setPower(speed);
            } else if (gamepad1.b) {
                leftLaunch.setPower(-1*speed);
                rightLaunch.setPower(-1*speed);
            } else {
                leftLaunch.setPower(0);
                rightLaunch.setPower(0);
            }

//            if (gamepad1.dpad_up) {
//                speed += 0.00001;
//            }
//
//            if (gamepad1.dpad_down) {
//                speed -= 0.00001;
//            }

            upPressed = (gamepad1.dpad_up);
            downPressed = (gamepad1.dpad_down);

            if (upPressed && !upWasPressed) {
                speed += 0.01;
            }

            if (downPressed && !downWasPressed) {
                speed -= 0.01;
            }

            upWasPressed = (gamepad1.dpad_up);
            downWasPressed = (gamepad1.dpad_down);


            if (speed < 0) { speed = 0; }
            if (speed > 1) { speed = 1; }

            telemetry.addData("speed", speed);
            telemetry.update();

        }
    }

    public void initialize() {

        leftLaunch = hardwareMap.get(DcMotor.class, "leftLaunch");
        rightLaunch = hardwareMap.get(DcMotor.class, "rightLaunch");

        leftLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLaunch.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLaunch.setDirection(DcMotorSimple.Direction.FORWARD);

    }

}
