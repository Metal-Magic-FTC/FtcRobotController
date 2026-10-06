package org.firstinspires.ftc.teamcode.biobuzz.comp1.turret;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Turret motor + the "point at a field position" math, shared by every turret OpMode so the gains live in one
 * place. Angle convention and limits are in TurretConstants.
 *
 * The encoder is zeroed in the constructor: the turret MUST be pointing out the robot FRONT at INIT.
 */
public class TurretController {

    // ======================= TUNE =======================
    public static final double KP = 0.02;          // power per degree of error
    public static final double KD = 0.0008;        // power per deg/s of turret speed (damping)
    public static final double KV = 0.0025;        // power per deg/s of robot rotation (feedforward)
    public static final double KS = 0.04;          // static friction kick
    public static final double TOLERANCE_DEG = 0.75;
    public static final double MAX_POWER = 0.6;
    public static final double MANUAL_POWER = 0.3;

    private final DcMotorEx motor;
    private final ElapsedTime clock = new ElapsedTime();
    private double prevHeading, prevTime;
    private boolean hasPrevHeading = false;
    private double robotOmegaDeg = 0;

    public TurretController(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, TurretConstants.TURRET_MOTOR_NAME);
        motor.setDirection(TurretConstants.TURRET_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Continuous turret angle, degrees (0 = robot front, + = counter-clockwise from above). */
    public double angleDeg() {
        return TurretConstants.ticksToDeg(motor.getCurrentPosition());
    }

    public double robotOmegaDeg() {
        return robotOmegaDeg;
    }

    /** Call once per loop with the robot heading (radians) so the rotation feedforward knows the turn rate. */
    public void updateHeading(double headingRad) {
        double now = clock.seconds();
        double dt = now - prevTime;
        if (hasPrevHeading && dt > 1e-4) {
            double omega = TurretConstants.wrap180(Math.toDegrees(headingRad - prevHeading)) / dt;
            robotOmegaDeg += 0.5 * (omega - robotOmegaDeg); // light low-pass
        }
        prevHeading = headingRad;
        prevTime = now;
        hasPrevHeading = true;
    }

    /** Forget the turn rate (call after the odometry pose is jumped / reset). */
    public void resetHeadingTracking() {
        hasPrevHeading = false;
        robotOmegaDeg = 0;
    }

    /**
     * Drives the turret toward the field point (targetX, targetY) given the robot pose (inches, radians).
     *
     * @return the turret angle it is driving to (continuous degrees)
     */
    public double aimAt(double robotX, double robotY, double headingRad, double targetX, double targetY) {
        double current = angleDeg();
        double target = chooseTarget(desiredDeg(robotX, robotY, headingRad, targetX, targetY), current);
        setPowerLimited(pdPower(target, current), current);
        return target;
    }

    /** stick: -1..1, positive = counter-clockwise. Soft limits still apply. */
    public void manual(double stick) {
        setPowerLimited(stick * MANUAL_POWER, angleDeg());
    }

    public void stop() {
        motor.setPower(0);
    }

    /** Turret angle (robot-relative, -180..180) that points the turret at the target. */
    public static double desiredDeg(double robotX, double robotY, double h, double targetX, double targetY) {
        double turretX = robotX + TurretConstants.TURRET_FORWARD_IN * Math.cos(h) - TurretConstants.TURRET_LEFT_IN * Math.sin(h);
        double turretY = robotY + TurretConstants.TURRET_FORWARD_IN * Math.sin(h) + TurretConstants.TURRET_LEFT_IN * Math.cos(h);
        double fieldAngle = Math.atan2(targetY - turretY, targetX - turretX);
        return TurretConstants.wrap180(Math.toDegrees(fieldAngle - h));
    }

    /**
     * Of all the equivalent angles (desired + k*360) inside the limits, take the one closest to where the turret is.
     * If none fit (target is in the dead zone), park at whichever limit is angularly closer to the target.
     */
    public static double chooseTarget(double desired, double current) {
        double best = Double.NaN;
        for (int k = -2; k <= 2; k++) {
            double c = desired + 360.0 * k;
            if (c < TurretConstants.MIN_DEG || c > TurretConstants.MAX_DEG) continue;
            if (Double.isNaN(best) || Math.abs(c - current) < Math.abs(best - current)) best = c;
        }
        if (!Double.isNaN(best)) return best;
        double toMin = Math.abs(TurretConstants.wrap180(desired - TurretConstants.MIN_DEG));
        double toMax = Math.abs(TurretConstants.wrap180(desired - TurretConstants.MAX_DEG));
        return toMin < toMax ? TurretConstants.MIN_DEG : TurretConstants.MAX_DEG;
    }

    private double pdPower(double targetDeg, double currentDeg) {
        double error = targetDeg - currentDeg;
        double turretVelDeg = TurretConstants.ticksToDeg(motor.getVelocity());
        // Robot turning CCW at w means the turret must turn CW at w relative to the robot to stay on target.
        double power = KP * error - KD * turretVelDeg + KV * -robotOmegaDeg;
        if (Math.abs(error) > TOLERANCE_DEG) power += Math.signum(error) * KS;
        else if (Math.abs(robotOmegaDeg) < 2.0) power = 0;
        return Range.clip(power, -MAX_POWER, MAX_POWER);
    }

    private void setPowerLimited(double power, double currentDeg) {
        if (currentDeg >= TurretConstants.MAX_DEG && power > 0) power = 0;
        if (currentDeg <= TurretConstants.MIN_DEG && power < 0) power = 0;
        motor.setPower(power);
    }
}
