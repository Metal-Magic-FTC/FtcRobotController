package org.firstinspires.ftc.teamcode.biobuzz.comp1.turret;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Reads the Axon turret encoder ("TurretEncoder"). Turn the turret BY HAND.
 *
 * If "TurretServo" (Continuous Rotation Servo) is in the config, it is sent power 0 = STOP. That does not move it,
 * it just switches the servo's PWM on so the Axon is awake and its encoder wire outputs a position.
 *
 * gamepad1:
 *   B - clear the recorded min / max
 */
@TeleOp(name = "Turret Encoder Readout (Axon)", group = "BioBuzz Tests")
public class TurretEncoderReadout extends OpMode {

    private AxonEncoder encoder;
    private CRServo servo; // null if "TurretServo" is not configured
    private double minVolts, maxVolts, minDeg, maxDeg;

    @Override
    public void init() {
        servo = hardwareMap.tryGet(CRServo.class, "TurretServo");
        if (servo != null) servo.setPower(0); // stopped, but PWM on so the Axon wakes up
        encoder = new AxonEncoder(hardwareMap);
        clearMinMax();
    }

    @Override
    public void init_loop() {
        readAndShow();
    }

    @Override
    public void loop() {
        readAndShow();
    }

    private void readAndShow() {
        encoder.update();
        if (gamepad1.bWasPressed()) clearMinMax();

        double v = encoder.volts();
        double deg = encoder.turretDeg();
        minVolts = Math.min(minVolts, v);
        maxVolts = Math.max(maxVolts, v);
        minDeg = Math.min(minDeg, deg);
        maxDeg = Math.max(maxDeg, deg);

        telemetry.addData("TurretServo", servo != null ? "found, held at power 0 (awake)"
                : "NOT in config - Axon may be asleep / unpowered");
        telemetry.addData("Raw voltage", "%.3f V", v);
        telemetry.addData("Voltage min / max seen", "%.3f / %.3f", minVolts, maxVolts);
        telemetry.addData("Turret angle (absolute)", "%.1f°   0 = front, + = left", deg);
        telemetry.addData("Turret min / max seen", "%.1f° / %.1f°", minDeg, maxDeg);
        telemetry.addLine("B clear min/max");
    }

    private void clearMinMax() {
        encoder.update();
        minVolts = maxVolts = encoder.volts();
        minDeg = maxDeg = encoder.turretDeg();
    }
}
