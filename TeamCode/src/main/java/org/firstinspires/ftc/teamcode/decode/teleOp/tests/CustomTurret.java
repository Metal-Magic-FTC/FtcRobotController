package org.firstinspires.ftc.teamcode.decode.teleOp.tests;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.lang.Math;

public class CustomTurret {
    private DcMotor turretMotor;

    private static final int TURRET_MIN = 0;
    private static final int TURRET_MAX = 555;
    private static final double TICKS_PER_RADIAN =
            TURRET_MAX / (2 * Math.PI);

    public CustomTurret(HardwareMap hardwareMap) {

        // Declares motor to rotate turret
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public double normalizeRadians(double angle) {
        while (angle < 0) angle += 2 * Math.PI;
        while (angle >= 2 * Math.PI) angle -= 2 * Math.PI;
        return angle;
    }

    public int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }

    public void rotateTurret(int targetTurretPosition) {
        turretMotor.setTargetPosition(targetTurretPosition);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.5);
    }

    public void updateTurretAim(Pose robotPose, Pose targetPose) {

        // double dx = START_X - robotPose.getX();
        // double dy = START_Y - robotPose.getY();

        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();

        double fieldAngleToStart = Math.atan2(dy, dx);

        double turretAngle =
                normalizeRadians(fieldAngleToStart - robotPose.getHeading());

        int targetTicks =
                (int) Math.round(turretAngle * TICKS_PER_RADIAN);

        targetTicks = clamp(targetTicks, TURRET_MIN, TURRET_MAX);

        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.9);
    }

}
