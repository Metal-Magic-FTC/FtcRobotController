package org.firstinspires.ftc.teamcode.biobuzz.comp1.turret;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Test for the turret's Axon MAX in CONTINUOUS ROTATION mode.
 *
 * ROBOT CONFIG
 *   - "turretServo"   : Continuous Rotation Servo (the Axon's 3-wire plug, on a servo port)
 *   - "turretEncoder" : Analog Input (OPTIONAL - the Axon's 4th wire, on an analog port). Skipped if not configured.
 *
 * SAFETY: nothing here stops the turret at a limit. It only moves while you hold the stick or a button, and power
 * starts capped at 20%. Watch the cables.
 *
 * gamepad1:
 *   left stick X    - turn the turret (stick left = positive power), scaled by the power cap
 *   A / B (hold)    - run at +cap / -cap (steady speed, good for timing a full turn)
 *   dpad up / down  - power cap +/- 5%
 *   X               - flip servo direction (use this until stick LEFT turns the turret COUNTER-CLOCKWISE from above)
 *   Y               - zero the angle readout here
 */
@TeleOp(name = "Turret Servo Test (Axon CR)", group = "BioBuzz Tests")
public class TurretServoTest extends OpMode {

    private static final String SERVO_NAME = "turretServo";
    private static final String ENCODER_NAME = "turretEncoder";

    /** Servo turns per turret turn: 2.75:1 turret ring / 3.2x overdrive. Only used for the turret angle readout. */
    private static final double SERVO_REVS_PER_TURRET_REV = 2.75 * 3.2;
    /** Axon analog output swings 0 .. 3.3 V over one servo turn. */
    private static final double ENCODER_MAX_VOLTS = 3.3;

    private CRServo servo;
    private AnalogInput encoder; // null if not configured

    private boolean reversed = false;
    private double powerCap = 0.20;

    // Unwrapping the analog reading: it wraps every servo turn, so count the wraps.
    private double lastServoDeg = Double.NaN;
    private double totalServoDeg = 0;
    private double minVolts = Double.MAX_VALUE, maxVolts = 0;

    @Override
    public void init() {
        servo = hardwareMap.get(CRServo.class, SERVO_NAME);
        servo.setPower(0);
        encoder = hardwareMap.tryGet(AnalogInput.class, ENCODER_NAME);

        telemetry.addLine("Turret Servo Test ready. Nothing moves until you use the stick / A / B.");
        telemetry.addData("Analog encoder", encoder != null ? "found" : "not configured (\"" + ENCODER_NAME + "\")");
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) powerCap = Math.min(1.0, powerCap + 0.05);
        if (gamepad1.dpadDownWasPressed()) powerCap = Math.max(0.05, powerCap - 0.05);
        if (gamepad1.xWasPressed()) {
            reversed = !reversed;
            servo.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        }
        if (gamepad1.yWasPressed()) totalServoDeg = 0;

        double power;
        if (gamepad1.a) power = powerCap;
        else if (gamepad1.b) power = -powerCap;
        else {
            double stick = -gamepad1.left_stick_x; // stick left = positive
            power = Math.abs(stick) > 0.05 ? stick * powerCap : 0;
        }
        servo.setPower(power);

        telemetry.addData("Power", "%+.2f   (cap %.0f%%)", power, powerCap * 100);
        telemetry.addData("Direction", reversed ? "REVERSE" : "FORWARD");

        if (encoder != null) {
            double volts = encoder.getVoltage();
            minVolts = Math.min(minVolts, volts);
            maxVolts = Math.max(maxVolts, volts);
            double servoDeg = volts / ENCODER_MAX_VOLTS * 360.0;
            if (!Double.isNaN(lastServoDeg)) {
                double delta = servoDeg - lastServoDeg;
                if (delta > 180) delta -= 360;   // wrapped backward past 0
                if (delta < -180) delta += 360;  // wrapped forward past 360
                totalServoDeg += delta;
            }
            lastServoDeg = servoDeg;

            telemetry.addLine("----- analog encoder -----");
            telemetry.addData("Voltage", "%.3f V   (seen %.3f .. %.3f)", volts, minVolts, maxVolts);
            telemetry.addData("Servo angle", "%.1f°  (wraps every servo turn)", servoDeg);
            telemetry.addData("Servo total", "%.1f°  = %.2f turns", totalServoDeg, totalServoDeg / 360.0);
            telemetry.addData("Turret angle (est.)", "%.1f°", totalServoDeg / SERVO_REVS_PER_TURRET_REV);
        } else {
            telemetry.addLine("No analog encoder: add Analog Input \"" + ENCODER_NAME + "\" to see position.");
        }

        telemetry.addLine("Stick X turn | A/B hold = +/- cap | dpad cap | X flip | Y zero");
    }

    @Override
    public void stop() {
        servo.setPower(0);
    }
}
