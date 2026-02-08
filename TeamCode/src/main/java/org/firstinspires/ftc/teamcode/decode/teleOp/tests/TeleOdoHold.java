package org.firstinspires.ftc.teamcode.decode.teleOp.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.DriveToPoint;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.GoBildaPinpointDriver;

import java.util.Locale;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@TeleOp(name = "TeleOp Odo Hold")

public class TeleOdoHold extends LinearOpMode {

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    private Position cameraPosition =  new Position(DistanceUnit.INCH, 6, -3.5, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    static final double cameraOffsetY = -127.0;
    static final double cameraOffsetX = -203.2;
    double motorSpeed;

    GoBildaPinpointDriver odo;
    DriveToPoint nav = new DriveToPoint(this);

    enum StateMachine {
        WAITING_FOR_START,
        AT_TARGET,
        DRIVE_TO_TARGET_BASKET,
        DRIVE_TO_TARGET_SUB_WAYPOINT,
        DRIVE_TO_TARGET_SUBMERSIBLE
    }

    static final Pose2D startingPos = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    static final Pose2D BASKET_TARGET = new Pose2D(DistanceUnit.MM, -450, 110, AngleUnit.DEGREES, 40);
    static final Pose2D SUBMERSIBLE_TARGET = new Pose2D(DistanceUnit.MM, 300, 1569, AngleUnit.DEGREES, 0);
    static final Pose2D SUB_WAYPOINT_TARGET = new Pose2D(DistanceUnit.MM, -254, 1424, AngleUnit.DEGREES, 0);

    boolean atTarget = false;

    // ODOMETRY HOLD VARIABLES
    static final double STICK_DEADBAND = 0.05;
    Pose2D holdPose = null;
    boolean holdingPosition = false;

    @Override
    public void runOpMode() {

        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFront = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBack = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBack = hardwareMap.get(DcMotor.class, "rightBackDrive");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(UtilityValues.compLeftBackDirection);
        leftFront.setDirection(UtilityValues.compLeftFrontDirection);
        rightBack.setDirection(UtilityValues.compRightBackDirection);
        rightFront.setDirection(UtilityValues.compRightFrontDirection);

        // Initialize odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(0, 130);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        odo.resetPosAndIMU();
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine = StateMachine.WAITING_FOR_START;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update();

            switch (stateMachine) {
                case WAITING_FOR_START:
                    odo.setPosition(startingPos);
                    stateMachine = StateMachine.AT_TARGET;
                    break;

                case DRIVE_TO_TARGET_BASKET:
                    if (nav.driveTo(odo.getPosition(), BASKET_TARGET, 0.65, 0.5)) {
                        telemetry.addLine("at basket!");
                        stateMachine = StateMachine.AT_TARGET;
                        setAllMotors(0);
                    }
                    atTarget = false;
                    break;

                case DRIVE_TO_TARGET_SUB_WAYPOINT:
                    if (nav.driveTo(odo.getPosition(), SUB_WAYPOINT_TARGET, 1, 0.5)) {
                        telemetry.addLine("at sub waypoint!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_SUBMERSIBLE;
                        setAllMotors(0);
                    }
                    atTarget = false;
                    break;

                case DRIVE_TO_TARGET_SUBMERSIBLE:
                    if (nav.driveTo(odo.getPosition(), SUBMERSIBLE_TARGET, 0.3, 0.5)) {
                        telemetry.addLine("at submersible!");
                        stateMachine = StateMachine.AT_TARGET;
                        setAllMotors(0);
                    }
                    atTarget = false;
                    break;

                case AT_TARGET:
                    atTarget = true;
                    break;
            }

            // =============================
            // TELEOP DRIVE + ODO HOLD
            // =============================
            double y = -gamepad2.left_stick_y - gamepad1.left_stick_y / 2;
            double x = gamepad2.left_stick_x + gamepad1.left_stick_x / 2;
            double rx = gamepad2.right_stick_x + gamepad1.right_stick_x / 2;

            boolean sticksIdle =
                    Math.abs(y) < STICK_DEADBAND &&
                            Math.abs(x) < STICK_DEADBAND &&
                            Math.abs(rx) < STICK_DEADBAND;

            if (sticksIdle) {

                // SNAP HOLD
                if (!holdingPosition) {
                    holdPose = odo.getPosition();
                    holdingPosition = true;
                }

                nav.driveTo(odo.getPosition(), holdPose, 0.4, 0.6);

                leftFront.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
                rightFront.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
                leftBack.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
                rightBack.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

                telemetry.addLine("ODO HOLD ACTIVE");

            } else {
                // MANUAL DRIVE
                holdingPosition = false;

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                motorSpeed = 1;

                leftFront.setPower(frontLeftPower * motorSpeed);
                leftBack.setPower(backLeftPower * motorSpeed);
                rightFront.setPower(frontRightPower * motorSpeed);
                rightBack.setPower(backRightPower * motorSpeed);
            }

            // BUTTON NAV OVERRIDE
            if (gamepad2.x) {
                stateMachine = StateMachine.DRIVE_TO_TARGET_BASKET;
                atTarget = false;
                holdingPosition = false;
            }
            if (gamepad2.y) {
                stateMachine = StateMachine.DRIVE_TO_TARGET_SUB_WAYPOINT;
                atTarget = false;
                holdingPosition = false;
            }

            telemetry.addData("Current State", stateMachine);
            Pose2D pos = odo.getPosition();
            telemetry.addData("Position", String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                    pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES)));

            telemetry.update();
        }
    }

    private void setAllMotors(double power) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }
}
