package org.firstinspires.ftc.teamcode.decode.teleOp.align.turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="!TEST TS TURRET")
public class TurretTest extends LinearOpMode {

    DcMotor turretMotor = null;


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

            if (gamepad1.a) {
                turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            if (gamepad1.b) {
                turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

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

    }

}
