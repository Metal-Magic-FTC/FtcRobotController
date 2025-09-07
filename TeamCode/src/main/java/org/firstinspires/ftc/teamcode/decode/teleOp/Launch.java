package org.firstinspires.ftc.teamcode.decode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;

@TeleOp(name="launcher")
public class Launch extends LinearOpMode {

    DcMotor leftLaunch = null;
    DcMotor rightLaunch = null;

    public Servo leftGate = null;

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

            if (gamepad1.left_bumper) {
                leftGate.setPosition(0);
            } else if (gamepad1.right_bumper) {
                leftGate.setPosition(1);
            } else if (gamepad1.y) {
                leftGate.setPosition(leftGate.getPosition()+0.01);
            } else if (gamepad1.x) {
                leftGate.setPosition(leftGate.getPosition()-0.01);
            } else if (gamepad1.left_trigger >= 0.3F) {
                leftGate.setPosition(leftGate.getPosition()+0.1);
            } else if (gamepad1.right_trigger >= 0.3F) {
                leftGate.setPosition(leftGate.getPosition()-0.1);
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

        leftGate = hardwareMap.servo.get("leftGate");

    }

}
