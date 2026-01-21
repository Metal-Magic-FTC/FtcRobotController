package org.firstinspires.ftc.teamcode.decode.teleOp.tests;

public class CustomSpindexer {
    private static final double TICKS_PER_SLOT = 250;
    private static final double TICKS_PER_6 = 125;

    private static final int TARGET_TOL = 2;
    private static final double MAX_POWER = 0.35;
    private static final int PRESENCE_ALPHA_THRESHOLD = 5;
    // ===== TYPES =====
    public enum Ball {EMPTY, PURPLE, GREEN}

    // ===== HARDWARE =====
//    private final DcMotorEx spinMotor;
//    private final ColorSensor intakeColor;

    // ==== STATE =====
    private int zeroCount = 0;
    private final Ball[] slots = {Ball.EMPTY, Ball.EMPTY, Ball.EMPTY};
}
