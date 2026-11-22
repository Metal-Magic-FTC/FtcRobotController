package org.firstinspires.ftc.teamcode.biobuzz.comp1.turret;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Reads the Axon turret encoder ("TurretEncoder"). Nothing is powered: turn the turret BY HAND.
 *
 * Point the turret out the robot FRONT before INIT - that becomes turret 0 deg (or press A later to re-zero).
 *
 * gamepad1:
 *   A - zero here (turret facing robot front)
 *   B - clear the recorded min / max
 */
@TeleOp(name = "Turret Encoder Readout (Axon)", group = "BioBuzz Tests")
public class TurretEncoderReadout extends OpMode {

    private AxonEncoder encoder;
    private double minVolts, maxVolts, minTurretDeg, maxTurretDeg;

    @Override
    public void init() {
        encoder = new AxonEncoder(hardwareMap);
        encoder.zero();
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
        if (gamepad1.aWasPressed()) {
            encoder.zero();
            clearMinMax();
        }
        if (gamepad1.bWasPressed()) clearMinMax();

        double v = encoder.volts();
        double turret = encoder.turretDeg();
        minVolts = Math.min(minVolts, v);
        maxVolts = Math.max(maxVolts, v);
        minTurretDeg = Math.min(minTurretDeg, turret);
        maxTurretDeg = Math.max(maxTurretDeg, turret);

        telemetry.addLine("Turn the turret BY HAND. 0 = robot front.");
        telemetry.addData("TURRET angle", "%.1f°", turret);
        telemetry.addData("Turret min / max seen", "%.1f° / %.1f°  (span %.1f°)",
                minTurretDeg, maxTurretDeg, maxTurretDeg - minTurretDeg);
        telemetry.addLine("-----------------------------");
        telemetry.addData("Voltage", "%.3f V  (hub max %.3f V)", v, encoder.maxVolts());
        telemetry.addData("Voltage min / max seen", "%.3f / %.3f V", minVolts, maxVolts);
        telemetry.addData("Servo shaft (this turn)", "%.1f°", encoder.rawServoDeg());
        telemetry.addData("Servo shaft total", "%.1f° = %.2f turns  (wraps %d)",
                encoder.servoDeg(), encoder.servoDeg() / 360.0, encoder.wraps());
        telemetry.addData("Servo turns per turret turn (config)", "%.2f", AxonEncoder.SERVO_REVS_PER_TURRET_REV);
        telemetry.addLine("A zero here | B clear min/max");
    }

    private void clearMinMax() {
        double v = encoder.volts();
        minVolts = maxVolts = v;
        minTurretDeg = maxTurretDeg = encoder.turretDeg();
    }
}
