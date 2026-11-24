package org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher;

/**
 * Every number the launcher needs, in one place. Anything marked MEASURE or TUNE is a placeholder until it has been
 * checked on the real robot.
 *
 * The launcher is FIXED to the chassis (no turret yet): it shoots out the robot front, so the driver aims by turning
 * the robot. One flywheel motor; the hood is set BY HAND and its angle is entered in LauncherTest.
 */
public final class LauncherConstants {

    private LauncherConstants() {}

    // ======================= HARDWARE (must match the Robot Configuration) =======================
    /** Same motor name as FlywheelTuning. Its motor TYPE in the configuration must be right (see Launcher). */
    public static final String MOTOR_NAME = "launchMotor";
    /** Flip so that a positive velocity pushes the ball OUT. */
    public static final boolean MOTOR_REVERSED = true;

    // ======================= FLYWHEEL (MEASURE / TUNE) =======================
    /**
     * true  = the ball is squeezed between two wheels (both driven by this motor) -> leaves at ~ wheel surface speed
     * false = one wheel rolls the ball along the hood                             -> leaves at ~ half the surface speed
     */
    public static final boolean DUAL_WHEEL = true;
    /** Flywheel diameter, inches (96 mm goBILDA wheel = 3.78 in). MEASURE. */
    public static final double WHEEL_DIAMETER_IN = 3.78;
    /** Motor revolutions per flywheel revolution (1.0 = direct drive, 0.5 = flywheel spins 2x the motor). */
    public static final double MOTOR_REVS_PER_WHEEL_REV = 1.0;
    /**
     * Ball speed / ideal ball speed (slip + ball squish). 0.8 - 0.9 is typical. MEASURE: film a shot at 240 fps
     * next to a tape measure and compare with LauncherTest's "measured" ball speed.
     */
    public static final double ETA = 0.85;
    /** Free speed of the motor (goBILDA 5203 bare = 6000 rpm). Targets are capped at 90% of this. */
    public static final double MOTOR_FREE_RPM = 6000.0;
    public static final double MAX_MOTOR_RAD_PER_SEC = 0.9 * MOTOR_FREE_RPM * 2.0 * Math.PI / 60.0;

    /**
     * Hub velocity PIDF (ticks/sec units, the SDK default style). TUNE with FlywheelTuning; F ~ 32767 / max ticks per
     * sec (6000 rpm, 28 ticks/rev -> 2800 tps -> F ~ 11.7).
     */
    public static final double VEL_P = 15.8; // tuned on the robot
    public static final double VEL_I = 0.0;
    public static final double VEL_D = 0.0;
    public static final double VEL_F = 13.5; // tuned on the robot
    /** Flywheel counts as "at speed" within this fraction of the target. */
    public static final double VELOCITY_TOLERANCE = 0.03;

    // ======================= HOOD (set by hand) =======================
    /**
     * The hood angle entered in LauncherTest is the BALL EXIT ANGLE above horizontal, not the plate angle: after
     * setting the hood, film a shot from the side once and measure the first few frames of the arc.
     */
    public static final double HOOD_MIN_DEG = 40.0;
    public static final double HOOD_MAX_DEG = 80.0;
    public static final double HOOD_DEFAULT_DEG = 60.0;

    // ======================= MOUNT (MEASURE) =======================
    /** Ball exit point relative to robot center, inches (forward / left) and height above the tiles. */
    public static final double LAUNCHER_FORWARD_IN = 0.0;
    public static final double LAUNCHER_LEFT_IN = 0.0;
    public static final double EXIT_HEIGHT_IN = 15.0;
    /** Direction the launcher shoots, relative to the robot front, + = toward robot left. */
    public static final double LAUNCHER_YAW_DEG = 0.0;

    // ======================= SHOT CHOICE =======================
    /**
     * Best descent angle at the CELL opening. The opening is tilted 30 deg up, so a ball falling at 30 deg meets it
     * square-on and sees the full 14 in. Used for the "suggested hood" hint. Usable window is roughly 0 .. 60 deg.
     */
    public static final double ENTRY_DESCENT_DEG = 30.0;
    public static final double MIN_ENTRY_DESCENT_DEG = 0.0;
    public static final double MAX_ENTRY_DESCENT_DEG = 60.0;
    /** Robot heading must be within this of the target to count as aimed (20 in opening, 2.8 in ball). */
    public static final double AIM_TOLERANCE_DEG = 2.5;

    /**
     * Drag correction: commanded ball speed = physics (no drag) speed * scale, interpolated by horizontal distance.
     * Starting guess 1.08; TUNE each entry at that distance with LauncherTest (dpad left/right, then copy the number).
     */
    public static final double[] SPEED_SCALE_DIST_IN = {36, 60, 84, 108, 132};
    public static final double[] SPEED_SCALE = {1.08, 1.08, 1.08, 1.08, 1.08};
}
