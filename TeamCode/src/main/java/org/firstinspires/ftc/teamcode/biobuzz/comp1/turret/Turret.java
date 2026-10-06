package org.firstinspires.ftc.teamcode.biobuzz.comp1.turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.GoBildaPinpointDriver;

@TeleOp(name="!TURRET TEST")
public class Turret extends LinearOpMode {

    DcMotor turretMotor = null;

    private GoBildaPinpointDriver odometry;

    private Pose2D target = new Pose2D(DistanceUnit.INCH, 12.0, -24.0, AngleUnit.DEGREES, 0);
    private Pose2D start = new Pose2D(DistanceUnit.INCH, 12.0, -24.0, AngleUnit.DEGREES, 0);

    private Pose2D pos = new Pose2D(DistanceUnit.INCH, start.getX(DistanceUnit.INCH), start.getY(DistanceUnit.INCH), AngleUnit.DEGREES, start.getHeading(AngleUnit.DEGREES));

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while (opModeIsActive()) {

            double power = gamepad1.right_trigger - gamepad1.left_trigger;

            if (gamepad1.x) {
                if (power > 0.1) {
                    power = 0.1;
                }

                if (power < -0.1) {
                    power = -0.1;
                }
            }

            turretMotor.setPower(power);

//            if (gamepad1.a) {
//                turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//            }
//
//            if (gamepad1.b) {
//                turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
            telemetry.addData("Odometry x", odometry.getPosX());
            telemetry.addData("Odometry y", odometry.getPosY());

            telemetry.addData("encoder for turret", turretMotor.getCurrentPosition());
            telemetry.addData("speed", power);
            telemetry.update();

        }

    }

    public void initialize() {

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Configure odometry - Pinpoint at center of robot with swingarm pods
        odometry.setOffsets(
                -5, 0); // Center of robot
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometry.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        odometry.resetPosAndIMU();

        telemetry.addData("Pinpoint IMU", "Available (press A to enable)");
        telemetry.addData("Odometry Config", "Center mount, swingarm pods");


    }

}