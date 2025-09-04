package org.firstinspires.ftc.teamcode.summerCamp.teleOp.demo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;

@TeleOp(name="Summer Camp Servo Test")
public class ServoTest extends LinearOpMode {

    DcMotor leftFrontDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightBackDrive = null;

    Servo pivotServo = null;
    Servo clawServo = null;

    /**
     * Main section of code -- like 'main' method
     * @throws InterruptedException - just in case
     */
    @Override
    public void runOpMode() throws InterruptedException {

        initialize(); // initializing everything

        waitForStart(); // waiting until driver clicks play button

        while (opModeIsActive()) {

            if (gamepad1.a) {
                clawServo.setPosition(clawServo.getPosition() + 0.05);
            }

            if (gamepad1.b) {
                clawServo.setPosition(clawServo.getPosition() - 0.05);
            }

            if (gamepad1.dpad_up) {
                pivotServo.setPosition(pivotServo.getPosition() + 0.05);
            }

            if (gamepad1.dpad_down) {
                clawServo.setPosition(clawServo.getPosition() - 0.05);
            }

        }

    }

    /**
     * code to initialize everything
     */
    public void initialize() {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBackDrive.setDirection(UtilityValues.compLeftBackDirection);
        leftFrontDrive.setDirection(UtilityValues.compLeftFrontDirection);
        rightBackDrive.setDirection(UtilityValues.compRightBackDirection);
        rightFrontDrive.setDirection(UtilityValues.compRightFrontDirection);

        pivotServo = hardwareMap.servo.get("pivotServo");
        clawServo = hardwareMap.servo.get("clawServo");

        pivotServo.setPosition(0);
        clawServo.setPosition(0);

    }


}
