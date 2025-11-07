package org.firstinspires.ftc.teamcode.decode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="TEST THE SCRUMPTIOUS LAUNCHER FAIL NNN WITH TS")
public class TestLauncher extends LinearOpMode {

    DcMotor launchMotor = null;

//    public Servo launchGate = null;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(); // initializing everything
        waitForStart(); // waiting until driver clicks play button

        while (opModeIsActive()) {


            if (gamepad1.a) {
                launchMotor.setPower(1);
            } else if (gamepad1.b) {
                launchMotor.setPower(-1);
            } else {
                launchMotor.setPower(0);
            }

        }
    }

    public void initialize() {

        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");

        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launchMotor.setDirection(DcMotorSimple.Direction.FORWARD);

//        launchGate = hardwareMap.servo.get("leftGate");
//        launchGate.setPosition(1);

    }

}
