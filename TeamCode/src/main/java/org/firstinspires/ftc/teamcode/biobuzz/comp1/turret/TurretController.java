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
    // Not final: TurretTuning changes these live. Copy the tuned numbers back here (as the starting values).
    // Tuned with TurretTuning (motor): KP 0.00891, KD 0.00090, KS 0.05567, tol 0.75, max 0.6, slew 3.0
    public static double KP = 0.00891;          // power per degree of error
    public static double KD = 0.00090;        // power per deg/s of turret speed (damping)
    public static final double KV = 0.0025;        // power per deg/s of robot rotation (feedforward)
    public static double KS = 0.05567;          // static friction kick
    // 0.5 = always settles on the encoder tick NEAREST the target (1 tick = 0.9 deg, 400 ticks/rev). Going lower
    // gains nothing (the encoder cannot see it) and makes it hunt between two ticks. Was 0.75.
    public static double TOLERANCE_DEG = 0.5;
    public static double MAX_POWER = 0.6;
    public static final double MANUAL_POWER = 0.3;
    /**
     * Max change in turret power per second. Stops the turret slamming from 0 to full power on big swings (e.g. the
     * wrap-around swing when it runs out of travel), whose reaction torque jerks the whole drivetrain.
     */
    public static double MAX_POWER_SLEW_PER_SEC = 3.0;
    /**
     * Integral: when the turret sits a little off target (friction stops KP + KS from finishing the move), push
     * harder the longer it stays off, until it gets there. Only within I_ZONE_DEG of the target, reset once on target
     * or when it crosses over. Power added = KI * (degrees x seconds), capped at I_MAX_POWER. 0 = off.
     */
    public static double KI = 0.03;
    public static double I_ZONE_DEG = 6.0;
    public static double I_MAX_POWER = 0.12;

    // ======================= ROBOT MOTION (aimAt) =======================
    /**
     * aimAt measures the robot's field velocity from the poses it is given and works out how fast the target's
     * direction is sweeping around the turret (driving past it AND turning). That rate is fed forward through KV,
     * and the aim leads by LEAD_SEC to cover loop + motor delay. 0 = no lead.
     */
    public static double LEAD_SEC = 0.05;
    /**
     * false = SIMPLE aiming (how it was on Oct 28): point at the target from the current pose, only the robot's TURN
     * rate is fed forward, no driving-velocity feedforward and no lead. true = the velocity-tracking version above.
     */
    public static boolean TRACK_ROBOT_VELOCITY = false;
    /**
     * Aim trim, degrees, added to every auto-aim angle (+ = more to the LEFT / counter-clockwise, - = more RIGHT).
     * For a constant left/right miss. Live in the TeleOp: gamepad2 dpad right (-0.5) / left (+0.5).
     */
    public static double AIM_TRIM_DEG = 0.0;
    /** Velocity readings above this are pose jumps, not driving (in/s). */
    public static final double MAX_ROBOT_SPEED_IN_S = 90.0;
    /** Low-pass on the measured velocity: 1 = raw, smaller = smoother but slower to react. */
    public static final double VELOCITY_FILTER = 0.35;

    private final DcMotorEx motor;
    private double prevTx, prevTy, prevAimTime;
    private boolean hasPrevAim = false;
    private double robotVx = 0, robotVy = 0;   // turret point's field velocity, in/s (filtered)
    private double targetRateDeg = 0;          // how fast the aim angle is changing, deg/s (robot-relative)
    private final ElapsedTime clock = new ElapsedTime();
    private double prevHeading, prevTime;
    private boolean hasPrevHeading = false;
    private double robotOmegaDeg = 0;
    private double lastPower = 0;
    private final ElapsedTime slewClock = new ElapsedTime();
    private final ElapsedTime iClock = new ElapsedTime();
    private double errorSum = 0, lastError = 0;

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

    /** Forget the turn rate and velocity (call after the odometry pose is jumped / reset). */
    public void resetHeadingTracking() {
        hasPrevHeading = false;
        robotOmegaDeg = 0;
        hasPrevAim = false;
        robotVx = robotVy = 0;
        targetRateDeg = 0;
    }

    /**
     * The pose was CORRECTED (e.g. by vision) by dx, dy, dHeading, not driven: shift the motion references by the
     * same amount so the correction is not mistaken for the robot moving or spinning.
     */
    public void notePoseShift(double dx, double dy, double dHeadingRad) {
        if (hasPrevAim) {
            prevTx += dx;
            prevTy += dy;
        }
        if (hasPrevHeading) prevHeading += dHeadingRad;
    }

    /**
     * Drives the turret toward the field point (targetX, targetY) given the robot pose (inches, radians), allowing
     * for the robot's current velocity and turn rate.
     *
     * @return the turret angle it is driving to (continuous degrees)
     */
    public double aimAt(double robotX, double robotY, double headingRad, double targetX, double targetY) {
        double h = headingRad;
        double tx = robotX + TurretConstants.TURRET_FORWARD_IN * Math.cos(h) - TurretConstants.TURRET_LEFT_IN * Math.sin(h);
        double ty = robotY + TurretConstants.TURRET_FORWARD_IN * Math.sin(h) + TurretConstants.TURRET_LEFT_IN * Math.cos(h);
        updateVelocity(tx, ty);

        // Direction to the target from the turret: phi = atan2(Dy, Dx). Driving changes it by
        // dphi/dt = (Dy * vx - Dx * vy) / r^2; turning the robot changes the ROBOT-RELATIVE angle by -omega.
        double dx = targetX - tx, dy = targetY - ty;
        double r2 = Math.max(dx * dx + dy * dy, 36.0); // closer than 6 in: don't let the rate blow up
        double bearingRateDeg = Math.toDegrees((dy * robotVx - dx * robotVy) / r2);
        targetRateDeg = TRACK_ROBOT_VELOCITY ? bearingRateDeg - robotOmegaDeg : -robotOmegaDeg;
        double lead = TRACK_ROBOT_VELOCITY ? targetRateDeg * LEAD_SEC : 0;

        double current = angleDeg();
        double desired = desiredDeg(robotX, robotY, headingRad, targetX, targetY) + lead
                + AIM_TRIM_DEG;
        double target = chooseTarget(TurretConstants.wrap180(desired), current);
        setPowerLimited(pdPower(target, current, targetRateDeg), current);
        return target;
    }

    /** Turret point's field velocity from successive aimAt poses (filtered, jump-proof). */
    private void updateVelocity(double tx, double ty) {
        double now = clock.seconds();
        if (hasPrevAim) {
            double dt = now - prevAimTime;
            if (dt > 1e-3 && dt < 0.25) {
                double vx = (tx - prevTx) / dt, vy = (ty - prevTy) / dt;
                if (Math.hypot(vx, vy) <= MAX_ROBOT_SPEED_IN_S) {
                    robotVx += VELOCITY_FILTER * (vx - robotVx);
                    robotVy += VELOCITY_FILTER * (vy - robotVy);
                }
            } else if (dt >= 0.25) { // long gap (e.g. aim was off): start over
                robotVx = robotVy = 0;
            }
        }
        prevTx = tx;
        prevTy = ty;
        prevAimTime = now;
        hasPrevAim = true;
    }

    /** Robot (turret point) speed measured by aimAt, in/s. */
    public double robotSpeedInPerSec() {
        return Math.hypot(robotVx, robotVy);
    }

    /** How fast the aim angle is changing because of robot motion, deg/s (what the feedforward is covering). */
    public double targetRateDeg() {
        return targetRateDeg;
    }

    /**
     * Hold a robot-relative turret angle (continuous degrees, clipped to MIN_DEG..MAX_DEG). Same PD as aimAt; the
     * robot-rotation feedforward only acts if updateHeading() is being called.
     */
    public void driveTo(double targetDeg) {
        double current = angleDeg();
        double target = Range.clip(targetDeg, TurretConstants.MIN_DEG, TurretConstants.MAX_DEG);
        setPowerLimited(pdPower(target, current, -robotOmegaDeg), current);
    }

    /** Raw power -1..1 (+ = counter-clockwise). Hard limits and the slew limit still apply. */
    public void openLoop(double power) {
        setPowerLimited(power, angleDeg());
    }

    /** Turret speed, deg/s (+ = counter-clockwise). */
    public double velocityDeg() {
        return TurretConstants.ticksToDeg(motor.getVelocity());
    }

    /** Raw encoder ticks since INIT. */
    public int ticks() {
        return motor.getCurrentPosition();
    }

    /** Last power actually sent to the motor. */
    public double power() {
        return lastPower;
    }

    /** stick: -1..1, positive = counter-clockwise. Soft limits still apply. */
    public void manual(double stick) {
        setPowerLimited(stick * MANUAL_POWER, angleDeg());
    }

    public void stop() {
        lastPower = 0;
        motor.setPower(0);
    }

    /**
     * FAILSAFE: "the turret is pointing out the robot FRONT right now" - makes this position 0 deg (resets the
     * encoder). Use when the angle reading has slipped (belt skip, bumped turret) and the limits / aim are off.
     */
    public void rezero() {
        stop();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetHeadingTracking();
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
     * If none fit (target is in the dead zone), stay at whichever limit the turret is already nearer. (Picking by the
     * target instead made it slam 355 deg between the two limits whenever the target sat in the dead zone.)
     */
    public static double chooseTarget(double desired, double current) {
        double best = Double.NaN;
        for (int k = -2; k <= 2; k++) {
            double c = desired + 360.0 * k;
            if (c < TurretConstants.MIN_DEG || c > TurretConstants.MAX_DEG) continue;
            if (Double.isNaN(best) || Math.abs(c - current) < Math.abs(best - current)) best = c;
        }
        if (!Double.isNaN(best)) return best;
        double toMin = Math.abs(current - TurretConstants.MIN_DEG);
        double toMax = Math.abs(current - TurretConstants.MAX_DEG);
        return toMin < toMax ? TurretConstants.MIN_DEG : TurretConstants.MAX_DEG;
    }

    /**
     * @param ffRateDeg how fast the turret must turn (robot-relative, deg/s) just to stay on target because the robot
     *                  is moving / turning. Fed forward through KV, and KD only damps speed BEYOND it (so damping does
     *                  not fight the tracking motion).
     */
    private double pdPower(double targetDeg, double currentDeg, double ffRateDeg) {
        double error = targetDeg - currentDeg;
        double turretVelDeg = TurretConstants.ticksToDeg(motor.getVelocity());
        double dt = Math.min(iClock.seconds(), 0.1);
        iClock.reset();
        // integral: build up only while a little off target; clear once on target or after crossing over
        if (Math.abs(error) <= TOLERANCE_DEG || Math.abs(error) > I_ZONE_DEG
                || Math.signum(error) != Math.signum(lastError)) {
            errorSum = 0;
        } else {
            errorSum += error * dt;
        }
        lastError = error;
        double iPower = KI > 0 ? Range.clip(KI * errorSum, -I_MAX_POWER, I_MAX_POWER) : 0;

        double power = KP * error + iPower - KD * (turretVelDeg - ffRateDeg) + KV * ffRateDeg;
        if (Math.abs(error) > TOLERANCE_DEG) power += Math.signum(error) * KS;
        else if (Math.abs(ffRateDeg) < 2.0) power = 0; // on target and nothing to track: rest (BRAKE holds it)
        return Range.clip(power, -MAX_POWER, MAX_POWER);
    }

    private void setPowerLimited(double power, double currentDeg) {
        // Never push past the HARD limits, and ease off as it gets close so momentum can't carry it over.
        double roomLeft = TurretConstants.HARD_MAX_DEG - currentDeg;   // degrees left before the + limit
        double roomRight = currentDeg - TurretConstants.HARD_MIN_DEG;  // degrees left before the - limit
        if (power > 0) power = Math.min(power, MAX_POWER * Range.clip(roomLeft / TurretConstants.LIMIT_SLOWDOWN_DEG, 0, 1));
        if (power < 0) power = Math.max(power, -MAX_POWER * Range.clip(roomRight / TurretConstants.LIMIT_SLOWDOWN_DEG, 0, 1));
        double dt = Math.min(slewClock.seconds(), 0.1);
        slewClock.reset();
        double maxStep = MAX_POWER_SLEW_PER_SEC * dt;
        // Speeding up is rate-limited; slowing down / stopping takes effect immediately. A reversal drops straight to
        // 0 and ramps up the other way (never keeps pushing the old way, e.g. into a limit).
        if (power != 0 && Math.signum(power) != Math.signum(lastPower)) {
            power = Range.clip(power, -maxStep, maxStep);
        } else if (Math.abs(power) > Math.abs(lastPower)) {
            power = Range.clip(power, lastPower - maxStep, lastPower + maxStep);
        }
        lastPower = power;
        motor.setPower(power);
    }
}
