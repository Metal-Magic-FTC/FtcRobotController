package org.firstinspires.ftc.teamcode.decode.teleOp.tests;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.lang.Math;

public class CustomMecanumDrive {
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    public CustomMecanumDrive(HardwareMap hardwareMap) {

        // Declares all four motors for robot
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set motor directions as needed for your robot's configuration
//        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void setMode(DcMotor.ZeroPowerBehavior mode) {
        frontLeft.setZeroPowerBehavior(mode);
        frontRight.setZeroPowerBehavior(mode);
        backLeft.setZeroPowerBehavior(mode);
        backRight.setZeroPowerBehavior(mode);
    }

    public DcMotor.ZeroPowerBehavior getMode() {
        return frontLeft.getZeroPowerBehavior();
    }


    public void updateFrontLeft(boolean isForward) {
        if (isForward) {
            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void updateFrontRight(boolean isForward) {
        if (isForward) {
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void updateBackLeft(boolean isForward) {
        if (isForward) {
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void updateBackRight(boolean isForward) {
        if (isForward) {
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void powerOff() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void driveMecanum(double x, double y, double turn) {
        double frontLeftPower = y + x + turn;
        double frontRightPower = y - x - turn;
        double backLeftPower = y - x + turn;
        double backRightPower = y + x - turn;

        
        // Normalize powers to keep them between -1.0 and 1.0
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }
}
