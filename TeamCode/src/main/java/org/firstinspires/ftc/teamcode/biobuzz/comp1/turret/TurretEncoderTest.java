package org.firstinspires.ftc.teamcode.biobuzz.comp1.turret;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Reads the turret encoder so you can find the turret's limits. See TurretConstants for the angle convention.
 *
 * Point the turret straight out the robot FRONT before pressing INIT - that becomes 0 deg.
 * Turn it by hand (motor is in FLOAT) or with the stick, and write down the min/max it shows.
 *
 * gamepad1:
 *   left stick X - drive the turret slowly (let go = no power, FLOAT so you can push it by hand)
 *   A            - re-zero here (turret must be facing robot front)
 *   B            - clear the recorded min/max
 */
@TeleOp(name = "Turret Encoder Test", group = "BioBuzz Tests")
public class TurretEncoderTest extends OpMode {

    private static final double MAX_MANUAL_POWER = 0.25;

    private DcMotorEx turret;
    private double minSeenDeg = 0, maxSeenDeg = 0;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, TurretConstants.TURRET_MOTOR_NAME);
        turret.setDirection(TurretConstants.TURRET_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        zero();
        telemetry.addLine("Turret should be facing robot FRONT now. That is 0 deg.");
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) zero();
        if (gamepad1.bWasPressed()) {
            minSeenDeg = maxSeenDeg = angleDeg();
        }

        double stick = -gamepad1.left_stick_x; // stick left = counter-clockwise = positive
        turret.setPower(Math.abs(stick) > 0.05 ? stick * MAX_MANUAL_POWER : 0);

        int ticks = turret.getCurrentPosition();
        double deg = angleDeg();
        minSeenDeg = Math.min(minSeenDeg, deg);
        maxSeenDeg = Math.max(maxSeenDeg, deg);

        telemetry.addLine("0 = robot front, + = counter-clockwise from above");
        telemetry.addData("Raw ticks", ticks);
        telemetry.addData("Angle continuous", "%.1f°   <- use this for limits", deg);
        telemetry.addData("Angle -180..180", "%.1f°", TurretConstants.wrap180(deg));
        telemetry.addData("Angle 0..360", "%.1f°", TurretConstants.wrap360(deg));
        telemetry.addLine("-----------------------------");
        telemetry.addData("Min seen", "%.1f°  (%d ticks)", minSeenDeg, degToTicks(minSeenDeg));
        telemetry.addData("Max seen", "%.1f°  (%d ticks)", maxSeenDeg, degToTicks(maxSeenDeg));
        telemetry.addData("Ticks per turret rev (config)", "%.1f", TurretConstants.TICKS_PER_TURRET_REV);
        telemetry.addData("Velocity", "%.0f ticks/s", turret.getVelocity());
        telemetry.addLine("Stick X drive | A re-zero | B clear min/max");
    }

    @Override
    public void stop() {
        turret.setPower(0);
    }

    private void zero() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        minSeenDeg = maxSeenDeg = 0;
    }

    private double angleDeg() {
        return TurretConstants.ticksToDeg(turret.getCurrentPosition());
    }

    private static int degToTicks(double deg) {
        return (int) Math.round(deg / 360.0 * TurretConstants.TICKS_PER_TURRET_REV);
    }
}
