package org.firstinspires.ftc.teamcode.decode.AarushImprovements.util;

/**
 * Hardware-specific constants for the Magic Machine V4. These were scattered
 * across the competition code as magic numbers — collecting them here makes
 * the auto and teleop easier to tune and audit.
 */
public final class MagicConstants {

    private MagicConstants() {
        // utility
    }

    // -------- Spindexer --------
    /** Encoder ticks per revolution of the spindexer. */
    public static final int SPINDEXER_TICKS_PER_REV = 750;

    // -------- Turret --------
    /** Encoder ticks per revolution of the turret. */
    public static final double TURRET_TICKS_PER_REV = 555.0;

    // -------- Drivetrain (Pedro Pathing) --------
    /** Field coordinate of the red close start pose (X, Y, heading rad). */
    public static final double RED_START_X = 113.8154613466334;
    public static final double RED_START_Y = 135.5561097256858;
    public static final double RED_START_HEADING_DEG = 180.0;
}
