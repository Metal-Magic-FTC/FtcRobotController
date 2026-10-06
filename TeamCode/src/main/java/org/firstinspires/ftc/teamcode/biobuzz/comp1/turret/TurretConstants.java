package org.firstinspires.ftc.teamcode.biobuzz.comp1.turret;

/**
 * Shared turret numbers used by TurretEncoderTest and TurretAimTag.
 *
 * TURRET ANGLE CONVENTION (same as Pedro heading)
 *   - 0 deg   = turret pointing out the FRONT of the robot (where it must be when you press INIT)
 *   - +deg    = counter-clockwise when looking DOWN on the robot from above (toward robot left)
 *   - wrapped display range is -180 .. +180
 *   - limits below are CONTINUOUS degrees from zero (not wrapped), so a turret that can do 270 deg total
 *     can have e.g. MIN = -135, MAX = +135, and a turret that can do more than a full turn can go past 180.
 */
public final class TurretConstants {

    private TurretConstants() {}

    public static final String TURRET_MOTOR_NAME = "turret";

    /**
     * Flip this until turning the turret COUNTER-CLOCKWISE (seen from above) makes the angle go UP in
     * TurretEncoderTest.
     */
    public static final boolean TURRET_REVERSED = false;

    /**
     * Encoder ticks for ONE full turret revolution = motor encoder ticks per motor rev * gear ratio.
     * e.g. goBILDA 5203 312 rpm = 537.7 ticks/rev, with a 100:20 turret ring -> 537.7 * 5 = 2688.5.
     * To measure: in TurretEncoderTest zero it facing front, rotate exactly 180 deg, then ticks * 2.
     */
    public static final double TICKS_PER_TURRET_REV = 537.7 * 5.0;

    /** Soft limits in continuous degrees (fill these in from TurretEncoderTest; keep a few degrees of margin). */
    public static final double MIN_DEG = -170.0;
    public static final double MAX_DEG = 170.0;

    /** Turret rotation center relative to robot center, inches (forward = robot front, left = robot left). */
    public static final double TURRET_FORWARD_IN = 0.0;
    public static final double TURRET_LEFT_IN = 0.0;

    public static double ticksToDeg(double ticks) {
        return ticks / TICKS_PER_TURRET_REV * 360.0;
    }

    /** Wraps to -180 .. +180. */
    public static double wrap180(double deg) {
        deg %= 360.0;
        if (deg > 180.0) deg -= 360.0;
        if (deg <= -180.0) deg += 360.0;
        return deg;
    }

    /** Wraps to 0 .. 360. */
    public static double wrap360(double deg) {
        deg %= 360.0;
        if (deg < 0) deg += 360.0;
        return deg;
    }
}
