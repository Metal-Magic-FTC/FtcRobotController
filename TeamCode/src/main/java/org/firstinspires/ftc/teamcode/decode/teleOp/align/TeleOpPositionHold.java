package org.firstinspires.ftc.teamcode.decode.teleOp.align;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.DriveToPoint;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.GoBildaPinpointDriver;

@TeleOp(name = "TeleOp Position Hold")
public class TeleOpPositionHold extends LinearOpMode {

    DcMotor lf, rf, lb, rb;

    GoBildaPinpointDriver odo;
    DriveToPoint nav;

    Pose2D holdPose = null;
    boolean holding = false;

    static final double DEADBAND = 0.05;

    @Override
    public void runOpMode() {

        lf = hardwareMap.get(DcMotor.class, "frontLeft");
        lb = hardwareMap.get(DcMotor.class, "backLeft");
        rf = hardwareMap.get(DcMotor.class, "frontRight");
        rb = hardwareMap.get(DcMotor.class, "backRight");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-178, 0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        odo.resetPosAndIMU();

        nav = new DriveToPoint(this);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        waitForStart();

        while (opModeIsActive()) {

            odo.update();

            double y = -gamepad1.left_stick_y; // forward
            double x = gamepad1.left_stick_x;  // strafe
            double turn = gamepad1.right_stick_x;

            boolean driverActive =
                    Math.abs(x) > DEADBAND ||
                            Math.abs(y) > DEADBAND ||
                            Math.abs(turn) > DEADBAND;

            if (driverActive) {
                // driver control
                holding = false;
                holdPose = null;

                lf.setPower(y + x + turn);
                rf.setPower(y - x - turn);
                lb.setPower(y - x + turn);
                rb.setPower(y + x - turn);

            } else {
                // position hold
                if (!holding) {
                    holdPose = odo.getPosition();
                    holding = true;
                }

                nav.driveTo(
                        odo.getPosition(),
                        holdPose,
                        0.6,   // max speed
                        0.05   // tight hold tolerance
                );

                lf.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
                rf.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
                lb.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
                rb.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));
            }

            Pose2D p = odo.getPosition();
            telemetry.addData("Mode", holding ? "HOLDING" : "DRIVER");
            telemetry.addData("X", p.getX(DistanceUnit.MM));
            telemetry.addData("Y", p.getY(DistanceUnit.MM));
            telemetry.addData("H", p.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}