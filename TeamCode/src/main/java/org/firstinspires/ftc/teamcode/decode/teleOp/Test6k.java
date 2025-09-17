package org.firstinspires.ftc.teamcode.decode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="!Test 6k rpm")
public class Test6k extends LinearOpMode {

    DcMotor leftLaunch = null;
    DcMotor rightLaunch = null;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(); // initializing everything

        waitForStart(); // waiting until driver clicks play button

        while (opModeIsActive()) {


            if (gamepad1.a) {
                leftLaunch.setPower(1);
                rightLaunch.setPower(1);
            } else if (gamepad1.b) {
                leftLaunch.setPower(-1);
                rightLaunch.setPower(-1);
            } else {
                leftLaunch.setPower(0);
                rightLaunch.setPower(0);
            }

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
