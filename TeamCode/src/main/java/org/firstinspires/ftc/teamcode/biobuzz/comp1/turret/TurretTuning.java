package org.firstinspires.ftc.teamcode.biobuzz.comp1.turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.util.BulkReader;

import java.util.ArrayDeque;

/**
 * Live tuning for TurretController (the MOTOR turret), robot standing still. Robot rotation is NOT accounted for:
 * updateHeading() is never called, so the KV feedforward stays at zero and only KP / KD / KS / tolerance / max power /
 * slew are tuned. Nothing but the turret motor is powered.
 *
 * Every loop starts with a bulk read, so the angle and velocity the controller sees and the numbers measured here
 * come from the same snapshot. Resolution limit: one encoder tick = 360 / TICKS_PER_TURRET_REV degrees
 * (shown on telemetry). Errors are shown in degrees AND ticks; nothing finer than 1 tick is real.
 *
 * BEFORE INIT: turret pointing straight out the robot FRONT (encoder zeroes at INIT = 0 deg).
 *
 * gamepad1
 *   A              - auto step test on/off: 0 -> +size -> 0 -> -size -> 0 ... every DWELL_SEC (both sides),
 *                    measuring each step
 *   left stick button - SLOW LIMIT SWEEP: target creeps at SWEEP_DEG_PER_SEC to MAX_DEG, then MIN_DEG, then 0,
 *                    recording the furthest angle reached each way. Any other mode button stops it.
 *   Y              - cycle step size (2, 5, 15, 45, 90, 180 deg)
 *   B              - stop the test and hold 0
 *   X              - limp on/off (power 0)
 *   START          - KS finder: ramps power slowly each way until the turret moves 2 ticks
 *   left bumper    - set KS to the KS finder's suggestion
 *   dpad left/right - select gain       dpad up/down - gain up/down by the current factor
 *   right bumper   - cycle the factor (1%, 5%, 10%, 25%)
 *   BACK           - clear the step results
 *
 * SUGGESTED ORDER
 *   1. START (KS finder), then left bumper to use it.
 *   2. Small steps (2 and 5 deg): raise KP until it settles quickly; if it never gets inside the band, KS is low.
 *   3. Big steps (45, 90, 180): raise KD until the overshoot is ~0-1 tick and there are no zero crossings.
 *      If it rings at small steps after adding KD, lower KP a little.
 *   4. TOLERANCE: no smaller than 1 tick, or it hunts back and forth. MAX_POWER / SLEW: limit for big swings.
 *   5. Copy the final numbers into TurretController.
 */
@TeleOp(name = "Turret Tuning (motor)", group = "BioBuzz Tests")
public class TurretTuning extends LinearOpMode {

    // ======================= TEST SETTINGS =======================
    private static final double[] STEP_SIZES_DEG = {2, 5, 15, 45, 90, 180};
    private static final double DWELL_SEC = 2.5;          // time per step
    private static final double SETTLE_HOLD_SEC = 0.25;   // must stay in the band this long to count as settled
    private static final double STEADY_WINDOW_SEC = 0.5;  // steady-state error = last part of each step
    private static final int HISTORY = 8;
    private static final double[] FACTORS = {0.01, 0.05, 0.10, 0.25};

    // KS finder
    private static final double KS_RAMP_PER_SEC = 0.03;
    private static final double KS_MAX_POWER = 0.5;
    private static final int KS_MOVE_TICKS = 2;
    private static final double KS_PAUSE_SEC = 0.6;

    private static final String[] GAIN_NAMES = {"KP", "KD", "KS", "TOLERANCE_DEG", "MAX_POWER", "SLEW/s"};

    private static final double TICK_DEG = 360.0 / TurretConstants.TICKS_PER_TURRET_REV;

    private TurretController turret;

    private enum Mode { HOLD, STEP_TEST, LIMP, KS_FIND, LIMIT_SWEEP }

    // Slow limit sweep
    private static final double SWEEP_DEG_PER_SEC = 15.0;
    private static final double SWEEP_PAUSE_SEC = 1.0;
    /** Sweep legs: 0 = to MAX_DEG, 1 = to MIN_DEG, 2 = back to 0, 3 = done. */
    private int sweepLeg = 0;
    private double sweepPauseUntil = 0;
    private double sweepArrivedAt = Double.NaN;
    private String sweepNote = ""; // problems seen during the sweep (kept after it moves on)
    private double sweepMaxSeen = 0, sweepMinSeen = 0;
    private String sweepStatus = "not run (left stick button)";

    private Mode mode = Mode.HOLD;
    private double target = 0;
    private int stepIndex = 2;
    private int gainIndex = 0;
    private int factorIndex = 2;

    // Step test
    private Step current = null;
    private final ArrayDeque<Step> history = new ArrayDeque<>();
    /** Step test sequence as multiples of the step size: +1, 0, -1, 0, repeat. */
    private static final int[] STEP_SEQUENCE = {1, 0, -1, 0};
    private int stepPhase = 0;

    // KS finder
    private int ksDir = 0;            // direction being ramped (0 = idle)
    private double ksPower = 0;
    private int ksStartTicks = 0;
    private boolean ksPausing = false;
    private final ElapsedTime ksTimer = new ElapsedTime();
    private double ksPos = Double.NaN, ksNeg = Double.NaN;
    private String ksStatus = "not run (START)";

    // Loop timing
    private double loopMsAvg = 0, loopMsMax = 0;

    @Override
    public void runOpMode() {
        turret = new TurretController(hardwareMap); // zeroes the encoder: turret must face FRONT
        BulkReader bulk = new BulkReader(hardwareMap);
        ElapsedTime clock = new ElapsedTime();

        while (opModeInInit()) {
            bulk.update();
            telemetry.addLine("Turret facing robot FRONT? Robot must stay still. Press START to begin.");
            telemetry.addData("Turret", "%.2f°  (%d ticks)", turret.angleDeg(), turret.ticks());
            telemetry.addData("Resolution", "1 tick = %.3f°", TICK_DEG);
            telemetry.update();
        }

        clock.reset();
        double lastTime = 0;
        while (opModeIsActive()) {
            bulk.update(); // FIRST: fresh encoder + velocity snapshot for this loop

            double now = clock.seconds();
            double dt = now - lastTime;
            lastTime = now;
            double ms = dt * 1000.0;
            loopMsAvg += 0.05 * (ms - loopMsAvg);
            loopMsMax = Math.max(loopMsMax * 0.999, ms);

            handleButtons(now);

            double angle = turret.angleDeg();
            double vel = turret.velocityDeg();

            switch (mode) {
                case LIMP:
                    turret.stop();
                    break;
                case KS_FIND:
                    runKsFinder(dt);
                    break;
                case STEP_TEST:
                    if (current != null && now - current.t0 >= DWELL_SEC) {
                        finishStep();
                        stepPhase = (stepPhase + 1) % STEP_SEQUENCE.length;
                        startStep(STEP_SEQUENCE[stepPhase] * STEP_SIZES_DEG[stepIndex], angle, now);
                    }
                    turret.driveTo(target);
                    if (current != null) current.update(angle, vel, now);
                    break;
                case LIMIT_SWEEP:
                    runSweep(angle, dt, now);
                    break;
                case HOLD:
                default:
                    turret.driveTo(target);
                    break;
            }

            addTelemetry(angle, vel);
        }
        turret.stop();
    }

    // ======================= BUTTONS =======================

    private void handleButtons(double now) {
        if (gamepad1.aWasPressed()) {
            if (mode == Mode.STEP_TEST) {
                mode = Mode.HOLD;
                current = null;
            } else {
                mode = Mode.STEP_TEST;
                stepPhase = 0;
                startStep(STEP_SIZES_DEG[stepIndex], turret.angleDeg(), now);
            }
        }
        if (gamepad1.bWasPressed()) {
            mode = Mode.HOLD;
            current = null;
            target = 0;
        }
        if (gamepad1.xWasPressed()) {
            mode = mode == Mode.LIMP ? Mode.HOLD : Mode.LIMP;
            current = null;
            target = turret.angleDeg(); // un-limping holds where it is now
        }
        if (gamepad1.leftStickButtonWasPressed()) {
            if (mode == Mode.LIMIT_SWEEP) {
                mode = Mode.HOLD;
                target = turret.angleDeg();
                sweepStatus = "stopped";
            } else {
                mode = Mode.LIMIT_SWEEP;
                current = null;
                target = turret.angleDeg(); // creep from where it is now
                sweepLeg = 0;
                sweepArrivedAt = Double.NaN;
                sweepNote = "";
                sweepPauseUntil = 0;
                sweepMaxSeen = sweepMinSeen = turret.angleDeg();
                sweepStatus = "to + limit";
            }
        }
        if (gamepad1.yWasPressed()) stepIndex = (stepIndex + 1) % STEP_SIZES_DEG.length;
        if (gamepad1.startWasPressed()) startKsFinder();
        if (gamepad1.backWasPressed()) history.clear();
        if (gamepad1.leftBumperWasPressed() && !Double.isNaN(suggestedKs())) TurretController.KS = suggestedKs();

        if (gamepad1.rightBumperWasPressed()) factorIndex = (factorIndex + 1) % FACTORS.length;
        if (gamepad1.dpadRightWasPressed()) gainIndex = (gainIndex + 1) % GAIN_NAMES.length;
        if (gamepad1.dpadLeftWasPressed()) gainIndex = (gainIndex + GAIN_NAMES.length - 1) % GAIN_NAMES.length;
        if (gamepad1.dpadUpWasPressed()) scaleGain(1.0 + FACTORS[factorIndex]);
        if (gamepad1.dpadDownWasPressed()) scaleGain(1.0 / (1.0 + FACTORS[factorIndex]));
    }

    private double getGain(int i) {
        switch (i) {
            case 0: return TurretController.KP;
            case 1: return TurretController.KD;
            case 2: return TurretController.KS;
            case 3: return TurretController.TOLERANCE_DEG;
            case 4: return TurretController.MAX_POWER;
            default: return TurretController.MAX_POWER_SLEW_PER_SEC;
        }
    }

    /** Multiplies the selected gain. A gain at 0 jumps to a small seed value instead (0 * anything = 0). */
    private void scaleGain(double f) {
        double[] seeds = {0.005, 0.0001, 0.01, 0.25, 0.1, 0.5};
        double v = getGain(gainIndex);
        v = (v <= 0 && f > 1) ? seeds[gainIndex] : v * f;
        switch (gainIndex) {
            case 0: TurretController.KP = v; break;
            case 1: TurretController.KD = v; break;
            case 2: TurretController.KS = Math.min(v, 0.5); break;
            case 3: TurretController.TOLERANCE_DEG = v; break;
            case 4: TurretController.MAX_POWER = Math.min(v, 1.0); break;
            default: TurretController.MAX_POWER_SLEW_PER_SEC = v; break;
        }
    }

    // ======================= STEP TEST =======================

    private void startStep(double newTarget, double angle, double now) {
        // Same clip driveTo applies, so the measured target is the one it is really driving to.
        newTarget = Math.max(TurretConstants.MIN_DEG, Math.min(TurretConstants.MAX_DEG, newTarget));
        target = newTarget;
        current = new Step(angle, newTarget, now);
    }

    private void finishStep() {
        if (current == null) return;
        current.finish();
        history.addFirst(current);
        while (history.size() > HISTORY) history.removeLast();
    }

    /** One measured step response. */
    private static class Step {
        final double start, target, t0, size;
        double rise10 = Double.NaN, rise90 = Double.NaN;
        double enteredBand = Double.NaN; // time (since t0) it last entered the band, NaN if outside
        double settle = Double.NaN;      // final settle time, set in finish()
        double maxOvershoot = 0;         // deg past the target
        int crossings = 0;               // error sign flips outside the band (ringing)
        int lastSign = 0;
        double ssSum = 0, ssMax = 0;
        int ssN = 0;
        double peakVel = 0;
        double lastT = 0;

        Step(double start, double target, double t0) {
            this.start = start;
            this.target = target;
            this.t0 = t0;
            this.size = target - start;
        }

        int dir() {
            return size >= 0 ? 1 : -1;
        }

        void update(double angle, double vel, double now) {
            double t = now - t0;
            lastT = t;
            double band = band();
            double err = target - angle;
            peakVel = Math.max(peakVel, Math.abs(vel));

            if (Math.abs(size) > 1e-6) {
                double progress = (angle - start) / size;
                if (Double.isNaN(rise10) && progress >= 0.1) rise10 = t;
                if (Double.isNaN(rise90) && progress >= 0.9) rise90 = t;
                maxOvershoot = Math.max(maxOvershoot, (angle - target) * dir());
            }

            int s = Math.abs(err) > band ? (int) Math.signum(err) : 0;
            if (s != 0) {
                if (lastSign != 0 && s != lastSign) crossings++;
                lastSign = s;
                enteredBand = Double.NaN;
            } else if (Double.isNaN(enteredBand)) {
                enteredBand = t;
            }

            if (t >= DWELL_SEC - STEADY_WINDOW_SEC) {
                ssSum += Math.abs(err);
                ssMax = Math.max(ssMax, Math.abs(err));
                ssN++;
            }
        }

        void finish() {
            if (!Double.isNaN(enteredBand) && lastT - enteredBand >= SETTLE_HOLD_SEC) settle = enteredBand;
        }

        double ssMean() {
            return ssN == 0 ? Double.NaN : ssSum / ssN;
        }

        /** Settled = within the tolerance, but never tighter than 1 encoder tick. */
        static double band() {
            return Math.max(TurretController.TOLERANCE_DEG, TICK_DEG);
        }
    }

    // ======================= SLOW LIMIT SWEEP =======================

    /**
     * Moves the TARGET (not the power) toward each limit at SWEEP_DEG_PER_SEC, so the turret follows slowly and the
     * normal controller + hard-limit checks stay in charge the whole time.
     */
    private void runSweep(double angle, double dt, double now) {
        sweepMaxSeen = Math.max(sweepMaxSeen, angle);
        sweepMinSeen = Math.min(sweepMinSeen, angle);

        if (sweepLeg >= 3) {
            mode = Mode.HOLD;
            target = 0;
            sweepStatus = "done";
            turret.driveTo(target);
            return;
        }
        double[] goals = {TurretConstants.MAX_DEG, TurretConstants.MIN_DEG, 0};
        String[] names = {"to + limit", "to - limit", "back to 0"};
        double goal = goals[sweepLeg];

        if (now < sweepPauseUntil) {
            sweepStatus = "pause at " + String.format("%.1f°", angle);
        } else {
            double step = SWEEP_DEG_PER_SEC * Math.min(dt, 0.1);
            target += Math.max(-step, Math.min(step, goal - target));
            sweepStatus = names[sweepLeg];
            // Target arrived and the turret caught up (or gave up after 3 s, e.g. KS too low) -> pause, next leg.
            if (target != goal) {
                sweepArrivedAt = Double.NaN;
            } else if (Double.isNaN(sweepArrivedAt)) {
                sweepArrivedAt = now;
            }
            boolean caughtUp = Math.abs(goal - angle) <= Step.band();
            if (target == goal && (caughtUp || now - sweepArrivedAt > 3.0)) {
                if (!caughtUp) sweepNote += String.format(" did not reach %.0f° (stuck at %.1f°).", goal, angle);
                sweepLeg++;
                sweepPauseUntil = now + SWEEP_PAUSE_SEC;
                sweepArrivedAt = Double.NaN;
            }
        }
        turret.driveTo(target);
    }

    // ======================= KS FINDER =======================

    private void startKsFinder() {
        mode = Mode.KS_FIND;
        current = null;
        ksPos = ksNeg = Double.NaN;
        ksDir = turret.angleDeg() > 0 ? -1 : 1; // first push toward the middle of the travel
        beginKsRamp();
        ksStatus = "ramping " + (ksDir > 0 ? "+" : "-");
    }

    private void beginKsRamp() {
        turret.stop();
        ksPower = 0;
        ksStartTicks = turret.ticks();
        ksPausing = false;
    }

    private void runKsFinder(double dt) {
        if (ksPausing) {
            turret.stop();
            if (ksTimer.seconds() < KS_PAUSE_SEC) return;
            if (Double.isNaN(ksPos) || Double.isNaN(ksNeg)) {
                ksDir = -ksDir;
                beginKsRamp();
                ksStatus = "ramping " + (ksDir > 0 ? "+" : "-");
            } else {
                finishKs("done");
            }
            return;
        }
        ksPower += KS_RAMP_PER_SEC * Math.min(dt, 0.1);
        if (ksPower > KS_MAX_POWER) {
            finishKs(String.format("FAILED: no movement %s up to %.2f power", ksDir > 0 ? "+" : "-", KS_MAX_POWER));
            return;
        }
        turret.openLoop(ksDir * ksPower);
        if (Math.abs(turret.ticks() - ksStartTicks) >= KS_MOVE_TICKS) {
            if (ksDir > 0) ksPos = ksPower;
            else ksNeg = ksPower;
            turret.stop();
            ksPausing = true;
            ksTimer.reset();
            ksStatus = "pausing";
        }
    }

    private void finishKs(String status) {
        turret.stop();
        ksDir = 0;
        ksStatus = status;
        mode = Mode.HOLD;
        target = turret.angleDeg();
    }

    /** Breakaway power seen in the harder direction, a touch under so it doesn't jump on its own. */
    private double suggestedKs() {
        if (Double.isNaN(ksPos) || Double.isNaN(ksNeg)) return Double.NaN;
        return 0.9 * Math.max(ksPos, ksNeg);
    }

    // ======================= TELEMETRY =======================

    private void addTelemetry(double angle, double vel) {
        double err = target - angle;
        telemetry.addData("Mode", mode == Mode.STEP_TEST ? "STEP TEST (A stop)" : mode.toString());
        telemetry.addData("Turret", "%.2f°  (%d ticks)   target %.2f°", angle, turret.ticks(), target);
        telemetry.addData("Error", "%+.2f°  (%+.1f ticks)   band ±%.2f°", err, err / TICK_DEG, Step.band());
        telemetry.addData("Speed / power", "%.1f °/s   %.3f", vel, turret.power());
        telemetry.addData("Loop", "%.1f ms avg, %.1f ms max   (1 tick = %.3f°)", loopMsAvg, loopMsMax, TICK_DEG);
        telemetry.addData("Step size (Y)", "%.0f°", STEP_SIZES_DEG[stepIndex]);

        telemetry.addLine("--- GAINS (dpad L/R select, U/D change, RB factor) ---");
        telemetry.addData("Factor", "%.0f%%", FACTORS[factorIndex] * 100);
        for (int i = 0; i < GAIN_NAMES.length; i++) {
            telemetry.addData((i == gainIndex ? "> " : "   ") + GAIN_NAMES[i], "%.5f", getGain(i));
        }

        telemetry.addLine("--- SLOW LIMIT SWEEP (left stick button) ---");
        telemetry.addData("Sweep", "%s   reached %+.1f° .. %+.1f°   (limits %+.0f / %+.0f, hard %+.0f / %+.0f)",
                sweepStatus, sweepMinSeen, sweepMaxSeen, TurretConstants.MIN_DEG, TurretConstants.MAX_DEG,
                TurretConstants.HARD_MIN_DEG, TurretConstants.HARD_MAX_DEG);
        if (!sweepNote.isEmpty()) telemetry.addLine("Sweep:" + sweepNote);

        telemetry.addLine("--- KS FINDER (START, LB apply) ---");
        telemetry.addData("KS finder", "%s   + %s   - %s   suggest %s", ksStatus,
                fmt(ksPos, "%.3f"), fmt(ksNeg, "%.3f"), fmt(suggestedKs(), "%.3f"));

        telemetry.addLine("--- STEPS (newest first, BACK clear) ---");
        telemetry.addLine("size | rise90 | settle | over | rings | ss mean/max | peak");
        for (Step s : history) {
            telemetry.addLine(String.format("%+6.1f° | %s | %s | %.2f°(%.1ft) | %d | %.2f/%.2f° | %.0f°/s",
                    s.size, fmt(s.rise90, "%.2fs"), fmt(s.settle, "%.2fs"), s.maxOvershoot,
                    s.maxOvershoot / TICK_DEG, s.crossings, s.ssMean(), s.ssMax, s.peakVel));
        }
        addAverages();
        telemetry.update();
    }

    /** Average of the stored steps in each direction (they can differ: gravity, cable drag, gear backlash). */
    private void addAverages() {
        for (int dir = 1; dir >= -1; dir -= 2) {
            double settle = 0, over = 0, ss = 0;
            int n = 0, settledN = 0;
            for (Step s : history) {
                if (s.dir() != dir) continue;
                n++;
                over += s.maxOvershoot;
                ss += s.ssMean();
                if (!Double.isNaN(s.settle)) {
                    settle += s.settle;
                    settledN++;
                }
            }
            if (n == 0) continue;
            telemetry.addData(dir > 0 ? "Avg + steps" : "Avg - steps",
                    "settle %s (%d/%d settled)  over %.2f°  ss %.2f°",
                    settledN == 0 ? "--" : String.format("%.2fs", settle / settledN), settledN, n, over / n, ss / n);
        }
    }

    private static String fmt(double v, String f) {
        return Double.isNaN(v) ? "--" : String.format(f, v);
    }
}
