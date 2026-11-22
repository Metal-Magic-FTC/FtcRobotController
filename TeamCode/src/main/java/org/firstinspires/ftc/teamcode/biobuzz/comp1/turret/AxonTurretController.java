package org.firstinspires.ftc.teamcode.biobuzz.comp1.turret;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;

/**
 * Axon MAX turret in CR mode ("TurretServo") + its built-in absolute encoder (AxonEncoder).
 * PD on the turret angle + feedforward that cancels the robot's own rotation, inside hard limits.
 *
 * Angles: 0 = robot front, + = counter-clockwise from above (toward robot left), degrees.
 */
public class AxonTurretController {

    public static final String SERVO_NAME = "TurretServo";

    // ======================= LIMITS (measured: 180 deg each way from front) =======================
    public static final double MIN_DEG = -180.0; // turret turned RIGHT (clockwise)
    public static final double MAX_DEG = 180.0;  // turret turned LEFT (counter-clockwise)
    /** When the target is behind the robot, stay on the current side unless it is this far past the back. */
    public static final double BACK_HYSTERESIS_DEG = 15.0;

    // ======================= MOUNT =======================
    /**
     * Turret center relative to the robot center, inches (forward = toward robot front, left = toward robot left).
     * Middle of the back half of the robot = a quarter of the robot length behind center. MEASURE and replace.
     */
    public static final double TURRET_FORWARD_IN = -Constants.ROBOT_LENGTH_IN / 4.0;
    public static final double TURRET_LEFT_IN = 0.0;

    // ======================= TUNE =======================
    /**
     * Flip if a POSITIVE power turns the turret RIGHT (clockwise). Check with the triggers in AxonTurretAimOdo:
     * the "Turret" telemetry should move toward + with left trigger.
     */
    public static final boolean SERVO_REVERSED = true; // right trigger turned it LEFT with false
    public static final double KP = 0.012;     // power per degree of error
    public static final double KD = 0.0004;    // power per deg/s of turret speed (damping)
    /** Full power ~ 500 deg/s at the turret (229 rpm overdrive / 2.75), so 1/500 power per deg/s of robot turn. */
    public static final double KV = 0.002;
    public static final double KS = 0.04;      // kick to get past friction / the CR deadband
    public static final double TOLERANCE_DEG = 1.0;
    public static final double MAX_POWER = 0.7;
    public static final double MANUAL_POWER = 0.3;
    /** Max change in power per second (keeps big swings from jerking the robot). */
    public static final double SLEW_PER_SEC = 4.0;

    private final CRServo servo;
    private final AxonEncoder encoder;

    private final ElapsedTime clock = new ElapsedTime();
    private double lastTime = 0;
    private double lastAngle = Double.NaN;
    private double turretVel = 0;     // deg/s, filtered
    private double prevHeading = Double.NaN;
    private double robotOmega = 0;    // deg/s, filtered
    private double lastPower = 0;
    private double angle = 0;
    private double loopDt = 0.02;
    /** +1 / -1: flips the servo command if positive power turns the turret the wrong way (see checkDirection). */
    private double directionSign = 1;

    public AxonTurretController(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, SERVO_NAME);
        servo.setDirection(SERVO_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        servo.setPower(0);
        encoder = new AxonEncoder(hardwareMap);
    }

    /** Read the encoder and the robot heading (radians). Call ONCE per loop before aimAt / manual / stop. */
    public void update(double robotHeadingRad) {
        double now = clock.seconds();
        double dt = now - lastTime;
        lastTime = now;
        loopDt = Range.clip(dt, 0.001, 0.1);

        encoder.update();
        angle = encoder.turretDeg();
        if (!Double.isNaN(lastAngle) && dt > 1e-4) {
            turretVel += 0.3 * ((angle - lastAngle) / dt - turretVel);
        }
        lastAngle = angle;

        double h = Math.toDegrees(robotHeadingRad);
        if (!Double.isNaN(prevHeading) && dt > 1e-4) {
            robotOmega += 0.5 * (wrap180(h - prevHeading) / dt - robotOmega);
        }
        prevHeading = h;
    }

    /** Forget the robot turn rate (call after the odometry pose is jumped / reset). */
    public void resetHeadingTracking() {
        prevHeading = Double.NaN;
        robotOmega = 0;
    }

    public double angleDeg() {
        return angle;
    }

    public double volts() {
        return encoder.volts();
    }

    public double robotOmegaDeg() {
        return robotOmega;
    }

    public double power() {
        return lastPower;
    }

    /**
     * Point at field point (targetX, targetY) from robot pose (inches, radians).
     * @return the turret angle it is driving to
     */
    public double aimAt(double robotX, double robotY, double headingRad, double targetX, double targetY) {
        double target = chooseTarget(desiredDeg(robotX, robotY, headingRad, targetX, targetY), angle);
        driveTo(target);
        return target;
    }

    /** Hold a robot-relative turret angle (0 = front, + = left), still cancelling robot rotation. */
    public void driveTo(double target) {
        target = Range.clip(target, MIN_DEG, MAX_DEG);
        double error = target - angle;
        // Robot turning CCW at w -> turret must turn CW at w (relative to the robot) to stay on target.
        double power = KP * error - KD * turretVel - KV * robotOmega;
        if (Math.abs(error) > TOLERANCE_DEG) power += Math.signum(error) * KS;
        else if (Math.abs(robotOmega) < 2.0) power = 0;
        setPower(Range.clip(power, -MAX_POWER, MAX_POWER));
    }

    /**
     * Raw open-loop power for the direction check (limits still apply, no slew). + should turn the turret LEFT.
     */
    public void rawPower(double power) {
        if (angle >= MAX_DEG && power > 0) power = 0;
        if (angle <= MIN_DEG && power < 0) power = 0;
        lastPower = power;
        servo.setPower(directionSign * power);
    }

    /** Flip the servo command direction (+1 normal, -1 flipped). */
    public void setDirectionSign(double sign) {
        directionSign = sign >= 0 ? 1 : -1;
    }

    public double directionSign() {
        return directionSign;
    }

    /** stick -1..1, + = counter-clockwise (left). Limits still apply. */
    public void manual(double stick) {
        setPower(stick * MANUAL_POWER);
    }

    public void stop() {
        lastPower = 0;
        servo.setPower(0);
    }

    /** Robot-relative turret angle that points at the target, -180 .. 180. */
    public static double desiredDeg(double robotX, double robotY, double h, double targetX, double targetY) {
        // Where the turret actually sits on the field (it is not at the robot center).
        double turretX = robotX + TURRET_FORWARD_IN * Math.cos(h) - TURRET_LEFT_IN * Math.sin(h);
        double turretY = robotY + TURRET_FORWARD_IN * Math.sin(h) + TURRET_LEFT_IN * Math.cos(h);
        double fieldAngle = Math.atan2(targetY - turretY, targetX - turretX);
        return wrap180(Math.toDegrees(fieldAngle - h));
    }

    /**
     * The limits are +-180, so every direction is reachable, but "straight behind" can be reached from either end.
     * Near the back, stay on the side the turret is already on (no 360 deg swing for a target wobbling around 180).
     */
    public static double chooseTarget(double desired, double current) {
        if (Math.abs(desired) > 180.0 - BACK_HYSTERESIS_DEG && Math.signum(desired) != Math.signum(current)
                && Math.abs(current) > 90.0) {
            double other = desired + Math.signum(current) * 360.0; // same direction, on the current side
            return Range.clip(other, MIN_DEG, MAX_DEG);
        }
        return Range.clip(desired, MIN_DEG, MAX_DEG);
    }

    private void setPower(double power) {
        if (angle >= MAX_DEG && power > 0) power = 0;
        if (angle <= MIN_DEG && power < 0) power = 0;
        double maxStep = SLEW_PER_SEC * loopDt;
        if (Math.abs(power) > Math.abs(lastPower) || Math.signum(power) != Math.signum(lastPower)) {
            power = Range.clip(power, lastPower - maxStep, lastPower + maxStep);
        }
        lastPower = power;
        servo.setPower(directionSign * power);
    }

    public static double wrap180(double deg) {
        deg %= 360.0;
        if (deg > 180.0) deg -= 360.0;
        if (deg <= -180.0) deg += 360.0;
        return deg;
    }
}
