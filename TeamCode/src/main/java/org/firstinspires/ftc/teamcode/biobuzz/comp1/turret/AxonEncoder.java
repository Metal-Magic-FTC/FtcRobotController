package org.firstinspires.ftc.teamcode.biobuzz.comp1.turret;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The Axon MAX's built-in encoder (its 4th, analog wire) as an ABSOLUTE turret angle.
 *
 * Gearing: servo -> 1:3.2 overdrive -> 2.75:1 turret ring, so the turret turns 3.2 / 2.75 = 1.164 times per servo
 * turn. The turret's whole travel (about -180 .. +180) is only ~310 deg of servo shaft, i.e. LESS than one servo turn,
 * so one voltage = one turret angle. No zeroing, no wrap counting: the turret can be anywhere at INIT.
 *
 * Measured (Turret Encoder Readout): turret facing robot FRONT = 2.731 V. Turning the turret RIGHT (clockwise from
 * above) makes the voltage go UP. The voltage wraps 3.3 -> 0 inside the travel; that is handled here.
 *
 * Robot Configuration: "TurretEncoder" = Analog Input (on the analog port number the Axon wire actually lands on).
 */
public class AxonEncoder {

    public static final String ENCODER_NAME = "TurretEncoder";

    /** Turret turns per servo-shaft turn (3.2x overdrive, then 2.75:1 down at the ring). */
    public static final double TURRET_REVS_PER_SERVO_REV = 3.2 / 2.75;
    /** Voltage with the turret pointing straight out the robot front. Re-measure if the servo is ever re-splined. */
    public static final double FRONT_VOLTS = 2.731;
    /** One full servo turn spans this many volts. */
    public static final double FULL_TURN_VOLTS = 3.3;
    /** true because turret RIGHT (clockwise) made the voltage go UP, and we want + = counter-clockwise (left). */
    public static final boolean REVERSED = true;

    private final AnalogInput input;
    private double volts;

    public AxonEncoder(HardwareMap hardwareMap) {
        input = hardwareMap.get(AnalogInput.class, ENCODER_NAME);
        update();
    }

    /** Read the encoder. Call once per loop. */
    public void update() {
        volts = input.getVoltage();
    }

    /** Raw voltage from the Axon wire. */
    public double volts() {
        return volts;
    }

    /** Servo shaft angle from the FRONT position, -180 .. +180 (the cut at +-180 falls outside the turret's travel). */
    public double servoDegFromFront() {
        double deg = (volts - FRONT_VOLTS) / FULL_TURN_VOLTS * 360.0;
        deg %= 360.0;
        if (deg > 180.0) deg -= 360.0;
        if (deg <= -180.0) deg += 360.0;
        return deg;
    }

    /** Absolute turret angle: 0 = robot front, + = counter-clockwise from above (toward robot left). */
    public double turretDeg() {
        return servoDegFromFront() * TURRET_REVS_PER_SERVO_REV * (REVERSED ? -1 : 1);
    }
}
