package org.firstinspires.ftc.teamcode.biobuzz.comp1.turret;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The Axon MAX's built-in encoder (its 4th, analog wire). Reads 0 .. ~3.3 V over ONE servo-shaft turn.
 *
 * The servo shaft turns several times per turret turn (overdrive + turret ring), so the reading wraps several times
 * per turret turn. update() counts the wraps to give a continuous angle. Call update() EVERY loop: if the servo moves
 * more than half a turn between two calls, a wrap is missed.
 *
 * Robot Configuration: "TurretEncoder" = Analog Input, on an analog port.
 */
public class AxonEncoder {

    public static final String ENCODER_NAME = "TurretEncoder";

    /**
     * Servo-shaft turns per turret turn: goBILDA 1:3.2 overdrive x 2.75:1 turret ring = 8.8.
     * CHECK: zero the readout, turn the turret exactly one full turn by hand, "servo turns" should read this number.
     */
    public static final double SERVO_REVS_PER_TURRET_REV = 3.2 * 2.75;

    /** Flip if turning the turret COUNTER-CLOCKWISE (seen from above) makes the turret angle go DOWN. */
    public static final boolean REVERSED = false;

    private final AnalogInput input;

    private double volts;
    private double lastServoDeg = Double.NaN;
    private double totalServoDeg = 0;
    private double zeroServoDeg = 0;
    private int wraps = 0;

    public AxonEncoder(HardwareMap hardwareMap) {
        input = hardwareMap.get(AnalogInput.class, ENCODER_NAME);
        update();
    }

    /** Read the encoder and count wraps. Call once per loop. */
    public void update() {
        volts = input.getVoltage();
        double servoDeg = rawServoDeg();
        if (!Double.isNaN(lastServoDeg)) {
            double delta = servoDeg - lastServoDeg;
            if (delta > 180) { delta -= 360; wraps--; }  // went backward past 0
            if (delta < -180) { delta += 360; wraps++; } // went forward past 360
            totalServoDeg += delta;
        }
        lastServoDeg = servoDeg;
    }

    /** Make the current position turret 0 deg (turret pointing out the robot front). */
    public void zero() {
        zeroServoDeg = totalServoDeg;
    }

    public double volts() {
        return volts;
    }

    public double maxVolts() {
        return input.getMaxVoltage();
    }

    /** Servo shaft angle inside the current turn, 0 .. 360 (wraps). */
    public double rawServoDeg() {
        return volts / input.getMaxVoltage() * 360.0;
    }

    /** Continuous servo shaft angle since zero, degrees. */
    public double servoDeg() {
        return (totalServoDeg - zeroServoDeg) * (REVERSED ? -1 : 1);
    }

    /** Continuous turret angle since zero, degrees (0 = robot front, + = counter-clockwise if REVERSED is right). */
    public double turretDeg() {
        return servoDeg() / SERVO_REVS_PER_TURRET_REV;
    }

    public int wraps() {
        return wraps;
    }
}
