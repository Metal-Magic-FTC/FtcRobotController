package org.firstinspires.ftc.teamcode.decode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="spindexter")
public class SpindexterTest extends LinearOpMode {

    DcMotor spinDexter = null;
    int currentPosition = 0;
    double spinRadians = ((2 * Math.PI) / 3);
\

    public void initialize() {

        spinDexter = hardwareMap.get(DcMotor.class, "spinDexter");

        spinDexter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spinDexter.setDirection(DcMotorSimple.Direction.REVERSE);

//        launchGate = hardwareMap.servo.get("leftGate");
//        launchGate.setPosition(1);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(); // initializing everything
        // waitForStart(); // waiting until driver clicks play button

        boolean aWasPressed = true;
        boolean aIsPressed;
        boolean oneTime = true;

        while (opModeIsActive()) {
        if (gamepad1.a) {
            spinDexter.setPower(1);
        }
        telemetry.addData("Spindexter position: ", spinDexter.getCurrentPosition());

//            aIsPressed = (gamepad1.a);
//            if (!aWasPressed && aIsPressed) {
//                if (oneTime) {
//
//                }
//
//                if (spinDexter.getCurrentPosition()<=(currentPosition+spinRadians)) {
//                        spinDexter.setPower(1);
//                }
//                } else {
//                    spinDexter.setPower(0);
//                    oneTime = true;
//                }
//            }
//
//            aWasPressed = (gamepad1.a);


        }
    }
}




