package org.firstinspires.ftc.teamcode.summerCamp.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;

@Autonomous(name="TEAM 2 - Autonomous run this")
public class SummerCampAutonomous2 extends LinearOpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    Servo pivotServo = null;
    Servo clawServo = null;

    static final double MOTOR_TICK_COUNTS = UtilityValues.motorTicks; // goBILDA 5203 series Yellow Jacket
    // figure out how many times we need to turn the wheels to go a certain distance
    // the distance you drive with one turn of the wheel is the circumference of the wheel
    // The wheel's Diameter is 96mm. To convert mm to inches, divide by 25.4
    static final double WHEEL_DIAMETER_INCHES = UtilityValues.wheelDiameter / 25.4; // in Inches
    static final double CIRCUMFERENCE_INCHES = Math.PI * WHEEL_DIAMETER_INCHES; // pi * the diameter of the wheels in inches

    static final double DEGREES_MOTOR_MOVES_IN_1_REV = 45.0;

    static final double SPEED = UtilityValues.SPEED; // Motor Power setting

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        /*
         * ============================
         * THIS IS THE ACTUAL DRIVING
         * ============================
         */

        //drive code...
        moveStraightLine(40);
        clawOpen();
        clawDown();
        clawClose();
        clawUp();
        strafe(50);
        clawDown();
        clawOpen();
        clawUp();
        moveStraightLine(-40);
        strafe(-50);


        strafe(24);
        moveStraightLine(38);
        strafe(-24);
        clawOpen();
        clawDown();
        clawClose();
        clawUp();
        strafe(50);
        clawDown();
        clawOpen();
        clawUp();
        rotate(45);
        moveStraightLine(-57);
        rotate(-45);


        rotate(855);
        moveStraightLine(10);
        moveStraightLine(-5);
        moveStraightLine(10);
        moveStraightLine(-5);
        moveStraightLine(10);
        moveStraightLine(-5);
        moveStraightLine(35);
        clawOpen();
        clawDown();
        clawClose();
        clawUp();
        strafe(50);
        clawDown();
        clawOpen();
        clawUp();
        clawClose();
        rotate(-45);
        moveStraightLine(-48);
        rotate(45);



        moveStraightLine(50);
        clawOpen();
        clawDown();
        clawClose();
        clawUp();
        strafe(50);
        clawDown();
        clawOpen();
        clawUp();
        rotate(360);
        strafe(-50);
        moveStraightLine(-50);
        rotate(360);
        clawOpen();
        clawClose();
        clawUp();
        clawDown();

    }

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

    public void drive(double speed, double leftFrontRevs, double leftBackRevs, double rightFrontRevs, double rightBackRevs) {

        int LFdrivetarget = (int) (leftFrontRevs * MOTOR_TICK_COUNTS) + leftFrontDrive.getCurrentPosition();
        int LBdrivetarget = (int) (leftBackRevs * MOTOR_TICK_COUNTS) + leftBackDrive.getCurrentPosition();
        int RFdrivetarget = (int) (rightFrontRevs * MOTOR_TICK_COUNTS) + rightFrontDrive.getCurrentPosition();
        int RBdrivetarget = (int) (rightBackRevs * MOTOR_TICK_COUNTS) + rightBackDrive.getCurrentPosition();

        leftFrontDrive.setTargetPosition(LFdrivetarget);
        leftBackDrive.setTargetPosition(LBdrivetarget);
        rightFrontDrive.setTargetPosition(RFdrivetarget);
        rightBackDrive.setTargetPosition(RBdrivetarget);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        rightBackDrive.setPower(speed);
        //
        // while (leftFrontDrive.isBusy() || leftBackDrive.isBusy() ||
        // rightFrontDrive.isBusy() || rightBackDrive.isBusy()) {
        //
        // }

        while (tolerance(leftFrontDrive) || tolerance(leftBackDrive) || tolerance(rightFrontDrive)
                || tolerance(rightBackDrive)) {

        }

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        sleep(20);
    }

    public boolean tolerance(DcMotor motor) {
        return Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) > 10;
    }

    private void moveStraightLine(double movementInInches) {
        double moveInRevs = movementInInches / CIRCUMFERENCE_INCHES;
        telemetry.addData("Moving ", "%.3f inches", movementInInches);
        telemetry.update();
        drive(SPEED, moveInRevs, moveInRevs, moveInRevs, moveInRevs);
    }

    private void strafe(double strafeInches) {
        // We assume that strafing right means positive
        double strafeRevs = Math.abs(strafeInches / CIRCUMFERENCE_INCHES);
        if (strafeInches >= 0) {
            telemetry.addData("Strafing towards right by ", "%.3f inches", strafeInches);

            drive(SPEED,
                    1 * strafeRevs,
                    -1 * strafeRevs,
                    -1 * strafeRevs,
                    1 * strafeRevs);
        } else {
            telemetry.addData("Strafing towards Left by ", "%.3f inches", Math.abs(strafeInches));

            drive(SPEED,
                    -1 * strafeRevs,
                    1 * strafeRevs,
                    1 * strafeRevs,
                    -1 * strafeRevs);
        }
    }

    public void rotate(double degrees) {

        double robotSpeed = SPEED;
        // Assume positive degrees means moving towards the right
        double movementOfWheelsInRevs = Math.abs(degrees / DEGREES_MOTOR_MOVES_IN_1_REV);

        if (degrees >= 0) {
            drive(robotSpeed,
                    1.0 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs);
        } else {
            // Moving negative means rotating left
            drive(robotSpeed,
                    -1 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs);
        }
    }
    public void clawDown() {
        pivotServo.setPosition(.63);
        sleep(1000);
    }
    public void clawUp() {
        pivotServo.setPosition(1);
        sleep(1000);
    }
    public void clawOpen() {
        clawServo.setPosition(1);
        sleep(1000);
    }
    public void clawClose() {
        clawServo.setPosition(.8);
        sleep(1000);
    }

}
