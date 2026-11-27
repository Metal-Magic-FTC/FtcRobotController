package org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher;

/**
 * Launcher wheel (72 mm traction) speed vs. horizontal distance from the TURRET to the target cell.
 *
 * Speeds are hub velocity units (encoder ticks per second, what DcMotorEx.setVelocity uses), the same units as the
 * FullLaunch test. Anchored on the measured point: 1650 against the wall facing the HIVE (~55 in). The other rows are
 * a physics estimate (fixed launch angle ~67 deg, ~42 in rise) scaled to that point - TUNE each row with real shots:
 * park at that distance (telemetry shows "Distance"), adjust until it scores, write the number here.
 */
public final class FlywheelSpeedTable {

    private FlywheelSpeedTable() {}

    /** Horizontal turret-to-cell distance, inches (must be increasing). */
    public static final double[] DISTANCE_IN = {40, 55, 70, 85, 100, 120};
    /**
     * Launcher velocity at each distance, ticks/s (BEFORE SPEED_SCALE). Physics: 72 mm hogback wheel, 1:1 belt, ball
     * leaves at 70 deg, needs to rise ~42 in to the cell, no drag; scaled so 55 in gives the measured wall shot
     * (1650 * 0.92 = 1518 actually commanded). Ball ends up at ~0.44 x wheel surface speed (single wheel on a hood).
     * Replace rows with real data points from "Launch Speed Picker (Red)" as they come in.
     */
    public static final double[] VELOCITY_TPS = {1521, 1650, 1789, 1925, 2054, 2217};

    /** Used before the robot knows where it is (not localized yet): the measured wall shot. */
    public static final double DEFAULT_TPS = 1650;

    /** Overall trim on every speed above. 1.0 = table as written; 0.95 = 5% slower. */
    public static final double SPEED_SCALE = 0.92; // was 0.88 - raised ~4% (slightly short everywhere)

    /** Speed for this distance, with SPEED_SCALE applied. */
    public static double velocityFor(double distanceIn) {
        return SPEED_SCALE * tableVelocity(distanceIn);
    }

    /** Straight-line interpolation between rows; flat beyond the ends. */
    private static double tableVelocity(double distanceIn) {
        if (Double.isNaN(distanceIn)) return DEFAULT_TPS;
        if (distanceIn <= DISTANCE_IN[0]) return VELOCITY_TPS[0];
        int last = DISTANCE_IN.length - 1;
        if (distanceIn >= DISTANCE_IN[last]) return VELOCITY_TPS[last];
        for (int i = 0; i < last; i++) {
            if (distanceIn <= DISTANCE_IN[i + 1]) {
                double t = (distanceIn - DISTANCE_IN[i]) / (DISTANCE_IN[i + 1] - DISTANCE_IN[i]);
                return VELOCITY_TPS[i] + t * (VELOCITY_TPS[i + 1] - VELOCITY_TPS[i]);
            }
        }
        return VELOCITY_TPS[last];
    }
}
