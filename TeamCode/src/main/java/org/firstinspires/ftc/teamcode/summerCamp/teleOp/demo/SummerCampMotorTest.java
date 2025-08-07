package org.firstinspires.ftc.teamcode.summerCamp.teleOp.demo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;

@TeleOp(name="MotorTest")
public class SummerCampMotorTest extends LinearOpMode {

    DcMotor leftFrontDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightBackDrive = null;

    /**
     * Main section of code -- like 'main' method
     * @throws InterruptedException - just in case
     */
    @Override
    public void runOpMode() throws InterruptedException {



        initialize(); // initializing everything

        waitForStart(); // waiting until driver clicks play button

        while (opModeIsActive()) {

            double moveCoefficient = gamepad1.right_trigger;

            rightFrontDrive.setPower(moveCoefficient);
            leftFrontDrive.setPower(moveCoefficient);
            rightBackDrive.setPower(moveCoefficient);
            leftBackDrive.setPower(moveCoefficient);

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


    }


}
