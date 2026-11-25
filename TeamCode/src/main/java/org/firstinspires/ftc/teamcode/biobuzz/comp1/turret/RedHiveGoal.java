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
    /** Measured in CAD: bottom cell aim point (57.8, 54.2). The top cell is the same point mirrored over Y = 72. */
    public static final double GOAL_X = 57.8;
    public static final double GOAL_BOTTOM_Y = 54.2;
    public static final double GOAL_TOP_Y = 2 * HALF_LINE_Y - GOAL_BOTTOM_Y; // 89.8
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
