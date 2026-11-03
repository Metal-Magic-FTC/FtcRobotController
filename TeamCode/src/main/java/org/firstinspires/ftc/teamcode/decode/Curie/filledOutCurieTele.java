package org.firstinspires.ftc.teamcode.decode.Curie;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * BasicTeleOp.java
 *
 * Drives a 4-motor mecanum robot with a gamepad, and controls two servos
 * (pivot + claw) with buttons.
 *
 * Robot configuration names needed:
 *   Motors: "leftFrontDrive", "rightFrontDrive", "leftBackDrive", "rightBackDrive"
 *   Servos: "pivotServo", "clawServo"
 *
 *
 * // NUMBER - PIVOT UP: 1      PIVOT DOWN: 0.88      CLAW_OPEN : 1     CLAW_CLOSE: 0
 *
 *
 * Controls:
 *   Left stick  -> drive forward/backward + strafe
 *   Right stick -> turn
 *   A / B       -> move pivot servo
 *   X / Y       -> open/close claw servo
 *
 * This file has // TODO markers, numbered 1-9, where you need to fill in
 * a line (or a few lines) of code yourself. Everything you need was covered
 * in the slides - look for the matching slide name in each TODO if you get stuck.
 */
@TeleOp(name = "hopefully working good nice maybe good curie TeleOp")
public class filledOutCurieTele extends LinearOpMode {

    // Declare our motors and servos. We connect them to the real
    // hardware later, inside initialize().
    DcMotor leftFrontDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightBackDrive = null;

    Servo pivotServo = null;
    Servo clawServo = null;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();   // set up all motors and servos
        waitForStart(); // wait until the driver presses PLAY

        /*
         * ===========================================
         * THIS IS THE ACTUAL DRIVING
         * ===========================================
         */
        while (opModeIsActive()) {

            // TODO 4 (slide "x, y, rx"): read gamepad1's left stick and right
            // stick into three variables named y, x, and rx.
            // Remember: y needs a minus sign in front of it!
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            // TODO 5 (slide "Normalizing Motor Power"): create a variable called
            // denominator using Math.max(...) so no motor power ever goes above 1.0.
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            // TODO 6 (slide "Mecanum Wheel Math"): create the four power variables
            // - frontLeftPower, backLeftPower, frontRightPower, backRightPower -
            // using y, x, rx, and denominator.
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Max power. Anything lower will make sure the robot does not go fast, for safety
            double motorSpeed = 1;

            // TODO 7 (slide "Setting power"): call setPower() on all four drive
            // motors, multiplying each power variable by motorSpeed.
            leftFrontDrive.setPower(frontLeftPower * motorSpeed);
            leftBackDrive.setPower(backLeftPower * motorSpeed);
            rightFrontDrive.setPower(frontRightPower * motorSpeed);
            rightBackDrive.setPower(backRightPower * motorSpeed);

            // Servos move to a POSITION (0.0 to 1.0), not a power like motors.

            // TODO 8 (slide "Servos"): if gamepad1.a is pressed, set the pivot
            // servo's position to 0.0. If gamepad1.b is pressed, set it to 1.0.
            if (gamepad1.a) {
                pivotServo.setPosition(0.0);
            } else if (gamepad1.b) {
                pivotServo.setPosition(1.0);
            }

            // TODO 9 (slide "Servos"): do the same thing for the claw servo,
            // using gamepad1.x (closed, 0.0) and gamepad1.y (open, 1.0).
            if (gamepad1.x) {
                clawServo.setPosition(0.0);
            } else if (gamepad1.y) {
                clawServo.setPosition(1.0);
            }

            telemetry.addData("Pivot Position", pivotServo.getPosition());
            telemetry.addData("Claw Position", clawServo.getPosition());
            telemetry.update();
        }
    }

    // Sets up all the motors and servos before the match starts.
    public void initialize() {

        // TODO 1 (slide "Declaring and Initializing Motors"): connect all four
        // motor variables to hardwareMap.get(DcMotor.class, "..."), using the
        // configuration names listed at the top of this file.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        // TODO 2 (slide "Servos"): connect pivotServo and clawServo to
        // hardwareMap.get(Servo.class, "...") the same way.
        pivotServo = hardwareMap.get(Servo.class, "pivotServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        // Brake instead of coast when power is 0, for more control
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO 3 (slide "More motor statements"): set direction on all four
        // motors using setDirection(DcMotorSimple.Direction...) - REVERSE the
        // left side motors, FORWARD the right side motors.
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Ready! Press PLAY to start.");
        telemetry.update();
    }
}