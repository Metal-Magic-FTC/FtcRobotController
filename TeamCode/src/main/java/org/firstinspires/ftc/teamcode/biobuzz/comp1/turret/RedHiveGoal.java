package org.firstinspires.ftc.teamcode.biobuzz.comp1.turret;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry;

/**
 * Which part of the RED HIVE the turret aims at, shared by TurretAimGoal and the autos.
 *   robot in the BOTTOM half of the field (Y < 72, audience side) -> BOTTOM (audience) cell
 *   robot in the TOP half (Y > 72)                                -> TOP (scoring) cell
 * The robot must cross the half line by SWITCH_HYSTERESIS_IN before the target switches, so it does not flip-flop.
 */
public class RedHiveGoal {

    /** Field line that splits "bottom half" from "top half" (the HIVE pivot line). */
    public static final double HALF_LINE_Y = HiveTagGeometry.PIVOT_Y_IN;
    /**
     * Aim at the CENTER of each cell: X = HIVE center line (59.25), Y = 72 -/+ 15.4 (56.6 bottom, 87.4 top).
     * (Was the CAD point 57.8, 54.2 / 89.8 for a while.)
     */
    public static final double GOAL_X = HiveTagGeometry.RED_HIVE_X_IN;
    public static final double CELL_OFFSET_Y = 15.4;
    /**
     * Bottom (audience) cell, tags 34-37: aim point nudged this far to the RIGHT as seen from the red drivers
     * (toward the audience, -Y). Bigger = further right; negative = left.
     */
    public static final double BOTTOM_CELL_RIGHT_SHIFT_IN = 0.0; // was 2.0 - moved back to the cell center
    public static final double GOAL_BOTTOM_Y = HALF_LINE_Y - CELL_OFFSET_Y - BOTTOM_CELL_RIGHT_SHIFT_IN; // 56.6
    public static final double GOAL_TOP_Y = HALF_LINE_Y + CELL_OFFSET_Y;
    public static final double SWITCH_HYSTERESIS_IN = 3.0;

    private boolean aimTop;

    public RedHiveGoal(double startY) {
        aimTop = startY > HALF_LINE_Y;
    }

    /** Call every loop with the robot's field Y. */
    public void update(double robotY) {
        if (robotY > HALF_LINE_Y + SWITCH_HYSTERESIS_IN) aimTop = true;
        else if (robotY < HALF_LINE_Y - SWITCH_HYSTERESIS_IN) aimTop = false;
    }

    public boolean aimingTop() {
        return aimTop;
    }

    public double x() {
        return GOAL_X;
    }

    public double y() {
        return aimTop ? GOAL_TOP_Y : GOAL_BOTTOM_Y;
    }
}
