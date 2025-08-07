package org.firstinspires.ftc.teamcode.summerCamp.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;

@TeleOp(name="TEAM 2 Run this - This is your TeleOp Mode")
public class SummerCampTeleOp2 extends LinearOpMode {

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

            /*
             * ===============
             * YOUR CODE BELOW
             * ===============
             */

            /*
             * Hints:
             * 1. set the x y and rx (rotation) based on the gamepad input
             */

            // you will need to change the 0s. they are placeholders.
            double x = 0;
            double y = 0; // hint, the y stick on the controller is reversed.
            double rx = 0;

            // Denominator is the largest motor power (abs value) or 1. It makes sure no more than 1 power is delivered.
            // This makes sure that the ratio stays the same
            // but only when at least one is out of range [-1, 1]
            // the maximum (Math.max(value1, value2)) of either:
            //      1. the sum of the absolute values (Math.abs(value)) of x, y, and rx
            //      2. 1
            // thix sets the denominator to the highest value if it is the sum or if it 1
            double denominator = 0;
            /*
             * frontLeftPower should be the sum of y, x, and rx all divided by denominator
             * backLeftPower should be y minus x plus rx all divided by denominator
             * frontRightPower should be y minus x minus rx all divided by denominator
             * backRightPower should be y plus x minus rx all divided by denominator
             */
            double frontLeftPower = 0;
            double backLeftPower = 0;
            double frontRightPower = 0;
            double backRightPower = 0;

            // set power to each of the motors now

            /*
             * ===============
             * YOUR CODE ENDS
             * ===============
             */

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

    }


}
