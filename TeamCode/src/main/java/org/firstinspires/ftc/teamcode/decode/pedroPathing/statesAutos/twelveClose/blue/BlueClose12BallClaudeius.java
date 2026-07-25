package org.firstinspires.ftc.teamcode.decode.pedroPathing.statesAutos.twelveClose.blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.teleOp.tests.CustomMecanumDrive;

/*
 * ============================================================================
 *  CHANGES vs. previous version
 * ----------------------------------------------------------------------------
 *  SPINDEXER INTAKE (the "fried last row" bug):
 *   - Rewrote intake() into a single clean state machine (intakeStep()).
 *   - LAST-BALL FIX: when the 3rd ball is captured, findNextEmpty() returns -1,
 *     so we now flag the carousel FULL, stop advancing, and cut the intake
 *     motor. No more re-reading the same ball / parking on a live slot / jamming
 *     a 4th ball into a full spindexer.
 *   - Added a "slot cleared" gate: a ball can't be captured again until the
 *     sensor has read EMPTY once (i.e. the previous ball physically rotated
 *     away). Kills double-latching on one ball.
 *   - Added a small debounce (DETECT_FRAMES) so a single noisy frame can't
 *     trigger a phantom advance.
 *   - primeIntake() detects an already-full carousel (used for the gate pass)
 *     and just keeps the intake motor spinning without touching slots.
 *
 *  DELAYS (all trimmed + hoisted to named constants so you can tune them):
 *   - Flywheel is held hot at LAUNCH_VELOCITY the entire auto, so shoot spin-up
 *     waits are now just "let the flick engage" (200 -> 150 ms).
 *   - Pre-shoot settle 400 -> 150 ms (PRESHOOT_MS).
 *   - Gate settle 750 -> 100 ms (GATE_SETTLE_MS).
 *   - Removed the pointless 2 ms COLOR_DELAY_MS mechanism entirely.
 *
 *  CLEANUP:
 *   - Removed dead code (scanBallsInSlots, aimClosest, indexAimToPattern,
 *     findGreen, findClosestLoaded, shootAllPattern, waitForSpindexer) and the
 *     unused fields/imports they pulled in. Restore any if you still need them.
 *   - Collapsed the repeated "reset + rotate to 0" boilerplate into prepIntake().
 *   - Dropped the redundant double slot hardcodes around aimToPattern().
 * ============================================================================
 */

//@Disabled
@Autonomous(name = "!!!!!!!! CLAUDE MM STATES Blue Close 12 Ball CLAUDEIUS THE FIFTH JR")
public class BlueClose12BallClaudeius extends LinearOpMode {

    private int index = 0;
    private float gain = 20;

    private enum Ball { EMPTY, PURPLE, GREEN }
    private Ball[] slots = {Ball.EMPTY, Ball.EMPTY, Ball.EMPTY};

    // ---------------- DRIVE ----------------
    private Follower follower;
    private GeneratedPathsBlue12BallCloseV3 paths;
    private CustomMecanumDrive drivetrain;
    private Limelight3A limelight;

    // ---------------- INTAKE, TRANSFER, SCORING ----------------
    private DcMotor spinMotor;
    private DcMotorEx launchMotor;
    private DcMotor intakeMotor;
    private DcMotorEx flickMotor;
    private Servo hoodServo;
    private NormalizedColorSensor intakeColor;
    private NormalizedColorSensor intakeColor2;

    private static final int[] OUTTAKE_POS = {500, 0, 250};
    private static final int[] INTAKE_POS  = {125, 375, 625};

    // ---------------- TUNABLES ----------------
    private static final double INTAKE_POWER     = -0.6;
    private static final double SPIN_POWER       = 0.38;
    private static final double SHOOT_SPIN_POWER = 0.30;
    private static final int    LAUNCH_VELOCITY  = 1700;

    private static final int SETTLE_TICKS  = 8;   // spinAtTarget tolerance
    private static final int DETECT_FRAMES  = 2;  // debounce; drop to 1 if you miss fast balls

    // settle delays (ms) -- keep small, raise only if balls don't clear
    private static final int SHOOT_SPINUP_MS = 150; // was 200
    private static final int SHOOT_FLICK_MS  = 150; // was 200
    private static final int PRESHOOT_MS     = 150; // was 400
    private static final int GATE_SETTLE_MS  = 100; // was 750

    // ---------------- INTAKE STATE ----------------
    private boolean intakeActive   = false;
    private boolean spindexerFull  = false;
    private boolean slotCleared    = true;      // sensor saw EMPTY since last capture
    private int     colorStable    = 0;         // debounce counter
    private Ball    lastReading     = Ball.EMPTY;

    // ---------------- RUN ----------------
    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();
        resetSlots();

        // preload arrangement
        slots[0] = Ball.PURPLE;
        slots[1] = Ball.GREEN;
        slots[2] = Ball.PURPLE;

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(GeneratedPathsBlue12BallCloseV3.START_POSE);
        paths = new GeneratedPathsBlue12BallCloseV3(follower);
        hoodServo.setPosition(0.80);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        launchMotor.setVelocity(LAUNCH_VELOCITY); // keep flywheel hot the whole auto

        // ---- PRELOAD SHOT ----
        Ball[] pattern = getPatternFromTag();
        aimToPattern(pattern);
        telemetry.addData("pattern", pattern[0] + " " + pattern[1] + " " + pattern[2]);
        telemetry.update();

        runPath(paths.shootOld(), PRESHOOT_MS, 1);
        shootAll();

        // ---- INTAKE ROW 1 ----
        prepIntake();
        runPath(paths.toIntake1(), 0, 1);
        runPathWithIntake(paths.intake1(), 0, 0.21);
        runPathWithIntake(paths.gate(), GATE_SETTLE_MS, 1);

        slots[0] = Ball.PURPLE; slots[1] = Ball.PURPLE; slots[2] = Ball.GREEN; // known row-1 layout
        aimToPattern(pattern);
        runPath(paths.shoot2(), 0, 1);
        shootAll();

        // ---- INTAKE ROW 2 ----
        prepIntake();
        runPath(paths.toIntake2(), 0, 1);
        runPathWithIntake(paths.intake2(), 0, 0.21);

        slots[0] = Ball.PURPLE; slots[1] = Ball.GREEN; slots[2] = Ball.PURPLE;
        aimToPattern(pattern);
        runPath(paths.shoot3(), 0, 1);
        shootAll();

        // ---- INTAKE ROW 3 (the last row) ----
        prepIntake();
        runPath(paths.toIntake3(), 0, 1);
        runPathWithIntake(paths.intake3(), 0, 0.21);

        slots[0] = Ball.GREEN; slots[1] = Ball.PURPLE; slots[2] = Ball.PURPLE;
        aimToPattern(pattern);
        runPath(paths.shoot4(), 0, 1);
        shootAll();

        // ---- LEAVE ----
        runPath(paths.leave(), 0, 1);
        intakeMotor.setPower(0);

        telemetry.addLine("Finished");
        telemetry.update();
    }

    // ---------------- SPINDEXER ----------------
    private void rotateToIndex(int target) {
        index = target;
        int base = intakeActive ? INTAKE_POS[target] : OUTTAKE_POS[target];
        int targetPos = closestModular(base, spinMotor.getCurrentPosition());

        spinMotor.setTargetPosition(targetPos);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(SPIN_POWER);
    }

    private int closestModular(int mod, int current) {
        int best = mod;
        int minDiff = Integer.MAX_VALUE;
        for (int k = -2; k <= 2; k++) {
            int candidate = mod + 750 * k;
            int diff = Math.abs(candidate - current);
            if (diff < minDiff) {
                minDiff = diff;
                best = candidate;
            }
        }
        return best;
    }

    private void aimToPattern(Ball[] pattern) {
        intakeActive = false;
        int idx = findBestStartIndex(pattern);
        if (idx == -1) idx = index; // safety: no green found, don't crash
        index = idx;
        rotateToIndex(idx);
    }

    private int findNextEmpty() {
        for (int i = 0; i < 3; i++) {
            int idx = (index + i) % 3;
            if (slots[idx] == Ball.EMPTY) return idx;
        }
        return -1;
    }

    private boolean spinAtTarget() {
        return Math.abs(spinMotor.getTargetPosition() - spinMotor.getCurrentPosition()) <= SETTLE_TICKS;
    }

    private int findBestStartIndex(Ball[] pattern) {
        int greenIndex = -1;
        for (int i = 0; i < 3; i++) {
            if (slots[i] == Ball.GREEN) { greenIndex = i; break; }
        }
        if (greenIndex == -1) return -1;

        // Patterns: GPP / PGP / PPG
        if (pattern[0] == Ball.GREEN) return greenIndex;             // GPP
        if (pattern[1] == Ball.GREEN) return (greenIndex + 2) % 3;   // PGP
        if (pattern[2] == Ball.GREEN) return (greenIndex + 1) % 3;   // PPG
        return -1;
    }

    // ---------------- INTAKE STATE MACHINE ----------------
    /** Reset the per-cycle intake state and start pulling. Slots are NOT reset here. */
    private void primeIntake() {
        colorStable = 0;
        lastReading = Ball.EMPTY;
        slotCleared = true;
        intakeActive = true;
        intakeMotor.setPower(INTAKE_POWER);

        int firstEmpty = findNextEmpty();
        spindexerFull = (firstEmpty == -1);
        if (!spindexerFull) rotateToIndex(firstEmpty);
    }

    /** Call every loop while a path is following. */
    private void intakeStep() {
        if (!intakeActive) return;
        if (spindexerFull) return; // gate pass: just keep the motor spinning

        Ball reading = detectColor(intakeColor, intakeColor2);

        // Slot must clear before we accept another ball -> no double-latch.
        if (reading == Ball.EMPTY) {
            slotCleared = true;
            colorStable = 0;
            lastReading = Ball.EMPTY;
            return;
        }

        // Only capture when the carousel is settled AND the last ball has cleared.
        if (!spinAtTarget() || !slotCleared) return;

        // Debounce.
        if (reading == lastReading) colorStable++;
        else { colorStable = 1; lastReading = reading; }
        if (colorStable < DETECT_FRAMES) return;

        // ---- capture ----
        slots[index] = reading;
        slotCleared = false;
        colorStable = 0;
        lastReading = Ball.EMPTY;

        int nextEmpty = findNextEmpty();
        if (nextEmpty != -1) {
            rotateToIndex(nextEmpty);      // advance, keep intaking
        } else {
            // LAST ROW: full. Stop cleanly so nothing jams and we don't re-read.
            spindexerFull = true;
            intakeActive = false;
            intakeMotor.setPower(0);
        }
    }

    // ---------------- PATH HELPERS ----------------
    private void prepIntake() {
        resetSlots();
        intakeActive = true;
        rotateToIndex(0); // start at slot 0 so intake fills 0,1,2 in order
    }

    private void runPath(PathChain path, int stopDelay, double speed) {
        follower.setMaxPower(speed);
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) follower.update();
        follower.breakFollowing();
        if (stopDelay > 0) sleep(stopDelay);
    }

    private void runPathWithIntake(PathChain path, int stopDelay, double speed) {
        primeIntake();
        follower.setMaxPower(speed);
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            intakeStep();
        }
        follower.breakFollowing();
        if (stopDelay > 0) sleep(stopDelay);

        intakeActive = false;
        intakeMotor.setPower(0);
    }

    // ---------------- APRILTAG AND COLOR SENSORS ----------------
    private Ball[] getPatternFromTag() {
        int id = detectAprilTag(200);
        if (id == 21) return new Ball[]{Ball.GREEN, Ball.PURPLE, Ball.PURPLE};
        if (id == 23) return new Ball[]{Ball.PURPLE, Ball.PURPLE, Ball.GREEN};
        return new Ball[]{Ball.PURPLE, Ball.GREEN, Ball.PURPLE};
    }

    private int detectAprilTag(long timeoutMs) {
        long start = System.currentTimeMillis();
        int id = 0;
        while (opModeIsActive() && System.currentTimeMillis() - start < timeoutMs) {
            LLResult r = limelight.getLatestResult();
            if (r != null && r.isValid() && !r.getFiducialResults().isEmpty())
                for (LLResultTypes.FiducialResult res : r.getFiducialResults()) {
                    id = res.getFiducialId();
                    if (21 <= id && id <= 23) return id;
                }
            sleep(15);
        }
        return 22;
    }

    private void enableLight(NormalizedColorSensor s) {
        if (s instanceof SwitchableLight) ((SwitchableLight) s).enableLight(true);
    }

    private Ball detectColor(NormalizedColorSensor sensor1, NormalizedColorSensor sensor2) {
        Ball ball1 = detectSingleSensor(sensor1);
        Ball ball2 = detectSingleSensor(sensor2);
        if (ball1 == Ball.PURPLE || ball2 == Ball.PURPLE) return Ball.PURPLE;
        if (ball1 == Ball.GREEN  || ball2 == Ball.GREEN)  return Ball.GREEN;
        return Ball.EMPTY;
    }

    private Ball detectSingleSensor(NormalizedColorSensor sensor) {
        NormalizedRGBA c = sensor.getNormalizedColors();
        float r = c.red, g = c.green, b = c.blue;

        float total = r + g + b;
        if (total < 0.03f) return Ball.EMPTY;

        // PURPLE: blue-dominant
        if (b > r * 1.2f && b > g * 1.1f && b > 0.12f) return Ball.PURPLE;
        // GREEN: looser dominance + absolute floor
        if (g > r * 1.15f && g > b * 1.15f && g > 0.15f) return Ball.GREEN;

        return Ball.EMPTY;
    }

    // ---------------- SHOOTING ----------------
    private void shootAll() {
        intakeMotor.setPower(0);

        flickMotor.setPower(1);
        launchMotor.setVelocity(LAUNCH_VELOCITY); // already hot; just engage the flick
        sleep(SHOOT_SPINUP_MS);

        int endPosition = spinMotor.getCurrentPosition() + 500;
        spinMotor.setTargetPosition(endPosition);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(SHOOT_SPIN_POWER);

        while (opModeIsActive() && spinMotor.isBusy()) {
            // sweep fires balls naturally
        }
        spinMotor.setPower(0);

        flickMotor.setPower(1);
        sleep(SHOOT_FLICK_MS); // clear the last ball
        flickMotor.setPower(0);

        resetSlots();
        index = (index + 2) % 3;

        intakeMotor.setPower(INTAKE_POWER); // pre-spin for the next travel leg
    }

    // ---------------- INIT ----------------
    private void initHardware() {
        drivetrain = new CustomMecanumDrive(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3); // APRILTAG PIPELINE
        limelight.start();

        spinMotor = hardwareMap.get(DcMotor.class, "spinMotor");
        intakeColor = hardwareMap.get(NormalizedColorSensor.class, "intakeColor");
        intakeColor2 = hardwareMap.get(NormalizedColorSensor.class, "intakeColor2");

        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeColor.setGain(gain);
        enableLight(intakeColor);
        intakeColor2.setGain(gain);
        enableLight(intakeColor2);

        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        launchMotor.setDirection(DcMotorEx.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(300, 0, 0, 12.9);
        launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        launchMotor.setVelocity(0);

        hoodServo = hardwareMap.servo.get("hoodServo");

        flickMotor = hardwareMap.get(DcMotorEx.class, "flickMotor");
        flickMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flickMotor.setPower(0);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void resetSlots() {
        for (int i = 0; i < 3; i++) slots[i] = Ball.EMPTY;
    }
}