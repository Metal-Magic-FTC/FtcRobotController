package org.firstinspires.ftc.teamcode.biobuzz.comp1.turret;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Turret driven by an Axon MK2 servo, closed loop on the Axon's analog position feedback wire.
 *
 * WIRING / SERVO SETUP
 *   - Axon MK2 programmed to CONTINUOUS (CR) mode with the Axon programmer.
 *   - Servo PWM lead  -> a servo port, configured as "Continuous Rotation Servo" named SERVO_NAME.
 *   - Axon feedback wire (the 4th, analog lead) -> an analog port, configured as "Analog Input" named ENCODER_NAME.
 *
 * ANGLE CONVENTION (same as TurretConstants / Pedro): 0 deg = turret pointing out the robot FRONT at INIT,
 * + = counter-clockwise seen from above. Angles are continuous (not wrapped), so gearing past one servo turn works.
 *
 * LIMIT CAPTURE (before lockLimits)
 *   captureDrive(stick) turns the turret at a fixed SLOW power (CAPTURE_POWER at full stick). Drive it to each end.
 *   The furthest angles reached each way are recorded. If the turret stops moving while being driven (it reached
 *   its physical end), power in that direction is cut until the stick is released, so it never grinds on the stop.
 *
 * HARD LIMITS (after lockLimits)
 *   lockLimits() pulls the captured extremes in by LIMIT_MARGIN_DEG and freezes them. After that there is no
 *   method to change, widen or clear them, and EVERY power command goes through output(), which refuses any power
 *   that pushes past a limit and slows the turret as it nears one. The only way to re-capture is to restart the
 *   OpMode.
 *
 * FAULTS (latch until restart, turret goes limp)
 *   - WRONG DIRECTION: positive power made the angle go down -> flip SERVO_REVERSED or ENCODER_REVERSED.
 *   - NO MOVEMENT (after lock only): power applied but the feedback did not move -> feedback wire unplugged / jam.
 */
public class AxonTurret {

    // ======================= HARDWARE =======================
    public static final String SERVO_NAME = "turretServo";
    public static final String ENCODER_NAME = "turretEncoder";
    /** Flip until positive power (stick left during capture) turns the turret counter-clockwise from above. */
    public static final boolean SERVO_REVERSED = false;
    /** Flip until the turret turning counter-clockwise (seen from above) makes the angle go UP. */
    public static final boolean ENCODER_REVERSED = false;
    /** Servo horn revolutions per turret revolution (1.0 = servo drives the turret directly). */
    public static final double SERVO_REVS_PER_TURRET_REV = 1.0;

    // ======================= LIMIT CAPTURE =======================
    /** Power at full stick while finding the limits. Fixed and slow on purpose. */
    public static final double CAPTURE_POWER = 0.10;
    /** Below this stick deflection the turret is not driven during capture. */
    public static final double CAPTURE_DEADBAND = 0.1;
    /** Driven for this long without moving END_STOP_MIN_MOVE_DEG = treat it as the physical end. */
    public static final double END_STOP_TIME_SEC = 0.4;
    public static final double END_STOP_MIN_MOVE_DEG = 1.5;

    // ======================= LIMITS =======================
    /** How far inside the captured extremes the hard limits sit. */
    public static final double LIMIT_MARGIN_DEG = 3.0;
    /** Refuse to lock limits that span less than this (catches "pressed lock without moving it"). */
    public static final double MIN_SPAN_DEG = 20.0;
    /** Within this distance of a limit, the allowed power toward it shrinks linearly. */
    public static final double SLOW_ZONE_DEG = 20.0;

    // ======================= SPEED (after lock) =======================
    /** Max power at the start of every run. Kept low on purpose. */
    public static final double START_MAX_POWER = 0.15;
    /** The speed cap can be raised in steps up to this, never further. */
    public static final double ABSOLUTE_MAX_POWER = 0.6;
    public static final double MIN_MAX_POWER = 0.05;
    /** Max change in power per second (acceleration only; slowing down is always immediate). */
    public static final double MAX_POWER_SLEW_PER_SEC = 1.5;

    // ======================= CONTROL =======================
    public static final double KP = 0.008;   // power per degree of error
    public static final double KD = 0.0004;  // power per deg/s of turret speed
    public static final double KS = 0.04;    // static friction kick (Axon CR deadband)
    public static final double TOLERANCE_DEG = 1.0;

    // ======================= FAULT DETECTION =======================
    private static final double FAULT_MIN_POWER = 0.08;
    private static final double CAPTURE_WATCH_MIN_POWER = 0.05;
    private static final double FAULT_TIME_SEC = 0.6;
    private static final double FAULT_MIN_MOVE_DEG = 1.5;

    private final CRServoImplEx servo;
    private final AnalogInput encoder;

    private double lastRawDeg;
    private double servoUnwrappedDeg = 0;
    private double angleDeg = 0;
    private double velDegPerSec = 0;
    private final ElapsedTime loopClock = new ElapsedTime();
    private final ElapsedTime slewClock = new ElapsedTime();

    // Limit capture
    private double seenMinDeg = 0, seenMaxDeg = 0;
    private int blockedSign = 0;        // direction cut off after hitting an end stop (0 = none)
    private double endStopPosDeg = Double.NaN, endStopNegDeg = Double.NaN;
    private boolean locked = false;
    private double minDeg = Double.NaN, maxDeg = Double.NaN;

    private boolean pwmOn = true; // forced off in constructor
    private boolean limp = false;
    private double maxPower = START_MAX_POWER;
    private double lastPower = 0;
    private String fault = null;

    // Movement watch window (fault detection after lock, end-stop detection during capture)
    private final ElapsedTime watchClock = new ElapsedTime();
    private double watchStartAngle = 0;
    private int watchSign = 0;

    public AxonTurret(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServoImplEx.class, SERVO_NAME);
        servo.setDirection(SERVO_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        encoder = hardwareMap.get(AnalogInput.class, ENCODER_NAME);
        disablePwm();

        lastRawDeg = rawServoDeg(); // turret angle 0 = wherever it is right now
        loopClock.reset();
        slewClock.reset();
    }

    // ======================= SENSOR =======================

    /** Servo horn angle from the feedback wire, 0..360. */
    private double rawServoDeg() {
        double deg = encoder.getVoltage() / encoder.getMaxVoltage() * 360.0;
        return ENCODER_REVERSED ? 360.0 - deg : deg;
    }

    public double feedbackVoltage() {
        return encoder.getVoltage();
    }

    /** Call exactly once per loop, before anything else on the turret. */
    public void update() {
        double raw = rawServoDeg();
        servoUnwrappedDeg += TurretConstants.wrap180(raw - lastRawDeg);
        lastRawDeg = raw;

        double newAngle = servoUnwrappedDeg / SERVO_REVS_PER_TURRET_REV;
        double dt = loopClock.seconds();
        loopClock.reset();
        if (dt > 1e-4 && dt < 0.25) {
            double v = (newAngle - angleDeg) / dt;
            velDegPerSec += 0.4 * (v - velDegPerSec);
        }
        angleDeg = newAngle;

        if (!locked) {
            seenMinDeg = Math.min(seenMinDeg, angleDeg);
            seenMaxDeg = Math.max(seenMaxDeg, angleDeg);
        }
    }

    public double angleDeg() {
        return angleDeg;
    }

    // ======================= LIMIT CAPTURE =======================

    /**
     * Drive the turret slowly to find its ends. stick -1..1, positive = counter-clockwise. Ignored once locked.
     * Power is always stick * CAPTURE_POWER, there is no way to go faster here.
     */
    public void captureDrive(double stick) {
        if (locked) return;
        if (fault != null || limp || Math.abs(stick) < CAPTURE_DEADBAND) {
            blockedSign = 0; // releasing the stick re-arms a blocked direction
            setOff();
            return;
        }
        int sign = (int) Math.signum(stick);
        if (sign == blockedSign) {
            setOff();
            return;
        }
        blockedSign = 0;
        double p = Range.clip(stick, -1, 1) * CAPTURE_POWER;
        apply(p);

        // End-stop check: being driven but not moving.
        if (watchSign == sign && Math.abs(p) >= CAPTURE_WATCH_MIN_POWER
                && watchClock.seconds() > END_STOP_TIME_SEC
                && Math.abs(angleDeg - watchStartAngle) < END_STOP_MIN_MOVE_DEG) {
            blockedSign = sign;
            if (sign > 0) endStopPosDeg = angleDeg;
            else endStopNegDeg = angleDeg;
            setOff();
        }
    }

    /** True while a direction is cut off because the turret hit its end; release the stick to clear. */
    public boolean atEndStop() {
        return blockedSign != 0;
    }

    /** Angle where the counter-clockwise end stop was felt, NaN if not yet. */
    public double endStopPosDeg() {
        return endStopPosDeg;
    }

    /** Angle where the clockwise end stop was felt, NaN if not yet. */
    public double endStopNegDeg() {
        return endStopNegDeg;
    }

    public boolean limitsLocked() {
        return locked;
    }

    public double seenMinDeg() {
        return seenMinDeg;
    }

    public double seenMaxDeg() {
        return seenMaxDeg;
    }

    public double minDeg() {
        return minDeg;
    }

    public double maxDeg() {
        return maxDeg;
    }

    /** Restart the capture from the current position. Only works before the limits are locked. */
    public void resetCapture() {
        if (locked) return;
        seenMinDeg = angleDeg;
        seenMaxDeg = angleDeg;
        endStopPosDeg = Double.NaN;
        endStopNegDeg = Double.NaN;
        blockedSign = 0;
    }

    /**
     * Freezes the captured range (minus the margin) as the hard limits. One-shot: returns false and changes nothing
     * if already locked or if the range is too small.
     */
    public boolean lockLimits() {
        if (locked) return false;
        double lo = seenMinDeg + LIMIT_MARGIN_DEG;
        double hi = seenMaxDeg - LIMIT_MARGIN_DEG;
        if (hi - lo < MIN_SPAN_DEG) return false;
        minDeg = lo;
        maxDeg = hi;
        locked = true;
        setOff();
        return true;
    }

    // ======================= SPEED CAP =======================

    public double maxPower() {
        return maxPower;
    }

    public void changeMaxPower(double delta) {
        maxPower = Range.clip(maxPower + delta, MIN_MAX_POWER, ABSOLUTE_MAX_POWER);
    }

    // ======================= COMMANDS (after lock) =======================

    /** Hold the turret at a turret angle (continuous degrees). The target is clipped into the limits. */
    public void goTo(double targetDeg) {
        if (!locked) {
            output(0);
            return;
        }
        double target = Range.clip(targetDeg, minDeg, maxDeg);
        double error = target - angleDeg;
        double power = KP * error - KD * velDegPerSec;
        if (Math.abs(error) > TOLERANCE_DEG) power += Math.signum(error) * KS;
        else power = 0;
        output(power);
    }

    /** stick -1..1, positive = counter-clockwise, scaled by the current speed cap. */
    public void manual(double stick) {
        output(stick * maxPower);
    }

    /** Make the turret limp (PWM off) or take it back. Faults stay limp regardless. */
    public void setLimp(boolean limp) {
        this.limp = limp;
    }

    public boolean isLimp() {
        return limp;
    }

    public void stop() {
        setOff();
    }

    /**
     * Of all equivalent angles (desired + k*360) inside the hard limits, the one closest to the current angle.
     * If none fit (target in the dead zone), the limit the turret is already nearer, so it never swings across.
     */
    public double chooseTarget(double desiredWrappedDeg) {
        if (!locked) return angleDeg;
        double best = Double.NaN;
        for (int k = -3; k <= 3; k++) {
            double c = desiredWrappedDeg + 360.0 * k;
            if (c < minDeg || c > maxDeg) continue;
            if (Double.isNaN(best) || Math.abs(c - angleDeg) < Math.abs(best - angleDeg)) best = c;
        }
        if (!Double.isNaN(best)) return best;
        return Math.abs(angleDeg - minDeg) < Math.abs(angleDeg - maxDeg) ? minDeg : maxDeg;
    }

    public boolean targetInDeadZone(double desiredWrappedDeg) {
        if (!locked) return true;
        for (int k = -3; k <= 3; k++) {
            double c = desiredWrappedDeg + 360.0 * k;
            if (c >= minDeg && c <= maxDeg) return false;
        }
        return true;
    }

    public String fault() {
        return fault;
    }

    public double lastPower() {
        return lastPower;
    }

    // ======================= OUTPUT =======================

    /** After lock: the only way power reaches the servo. Enforces the hard limits. */
    private void output(double requested) {
        if (!locked || fault != null || limp) {
            setOff();
            return;
        }

        double p = Range.clip(requested, -maxPower, maxPower);

        // Hard limits: no power toward a limit once at/past it, tapering inside the slow zone.
        if (p > 0) p = Math.min(p, allowedToward(maxDeg - angleDeg));
        if (p < 0) p = -Math.min(-p, allowedToward(angleDeg - minDeg));

        apply(p);
        if (fault == null && watchSign != 0 && watchClock.seconds() > FAULT_TIME_SEC
                && Math.abs(angleDeg - watchStartAngle) < FAULT_MIN_MOVE_DEG
                && Math.abs(lastPower) >= FAULT_MIN_POWER) {
            fault = "NO MOVEMENT: power applied but feedback did not change. Check the feedback wire / jam.";
            setOff();
        }
    }

    /** Slew limit, direction check, then send to the servo. */
    private void apply(double p) {
        double dt = Math.min(slewClock.seconds(), 0.1);
        slewClock.reset();
        double step = MAX_POWER_SLEW_PER_SEC * dt;
        // Speeding up is rate-limited; slowing down / stopping is immediate.
        if (Math.abs(p) > Math.abs(lastPower) || Math.signum(p) != Math.signum(lastPower)) {
            p = Range.clip(p, lastPower - step, lastPower + step);
        }

        double watchMin = locked ? FAULT_MIN_POWER : CAPTURE_WATCH_MIN_POWER;
        int sign = Math.abs(p) >= watchMin ? (int) Math.signum(p) : 0;
        if (sign == 0 || sign != watchSign) {
            watchSign = sign;
            watchStartAngle = angleDeg;
            watchClock.reset();
        } else if ((angleDeg - watchStartAngle) * sign < -FAULT_MIN_MOVE_DEG) {
            fault = "WRONG DIRECTION: power " + (sign > 0 ? "+" : "-")
                    + " moved the turret the other way. Flip SERVO_REVERSED (or ENCODER_REVERSED).";
            setOff();
            return;
        }

        enablePwm();
        lastPower = p;
        servo.setPower(p);
    }

    /** Max power allowed toward a limit that is distLeft degrees away. */
    private double allowedToward(double distLeft) {
        if (distLeft <= 0) return 0;
        double frac = Math.min(1.0, distLeft / SLOW_ZONE_DEG);
        return Math.max(maxPower * frac, Math.min(maxPower, KS + 0.02));
    }

    private void setOff() {
        lastPower = 0;
        watchSign = 0;
        servo.setPower(0);
        disablePwm();
    }

    private void enablePwm() {
        if (!pwmOn) {
            servo.setPwmEnable();
            pwmOn = true;
        }
    }

    private void disablePwm() {
        if (pwmOn) {
            servo.setPwmDisable();
            pwmOn = false;
        }
    }
}
