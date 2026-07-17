package org.firstinspires.ftc.teamcode.decode.AarushImprovements.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.decode.AarushImprovements.util.MagicConstants;

/**
 * Spindexer: rotating 3-slot ball storage with two color sensors at the
 * intake window. Handles detection, slot tracking, and rotation.
 *
 * <p>Default hardware names: {@code spinMotor}, {@code intakeColor},
 * {@code intakeColor2}.</p>
 */
public class Spindexer {

    public enum Ball {
        EMPTY, PURPLE, GREEN
    }

    public enum Mode {
        INTAKE, OUTTAKE
    }

    // ---------------- HARDWARE ----------------
    private final DcMotor spinMotor;
    private final NormalizedColorSensor colorSensor1;
    private final NormalizedColorSensor colorSensor2;

    // ---------------- CONSTANTS ----------------
    private static final int[] OUTTAKE_POSITIONS = {504, 2, 252};
    private static final int[] INTAKE_POSITIONS = {125, 375, 625};

    private static final int SPIN_TOLERANCE_TICKS = 5;
    private static final long SPIN_TIMEOUT_MS = 10_000;

    private static final float COLOR_SENSOR_GAIN = 20f;
    private static final float MIN_COLOR_TOTAL = 0.07f;
    private static final float PURPLE_BLUE_THRESHOLD = 0.12f;
    private static final float GREEN_THRESHOLD = 0.15f;
    private static final float PURPLE_RATIO = 1.35f;
    private static final float PURPLE_GREEN_RATIO = 1.25f;
    private static final float GREEN_RATIO = 1.15f;

    // ---------------- STATE ----------------
    private final Ball[] slots = {Ball.EMPTY, Ball.EMPTY, Ball.EMPTY};
    private int currentIndex = 0;
    private Mode currentMode = Mode.INTAKE;
    private double motorSpeed = 0.35;
    private int lastTargetPosition = 0;

    private boolean waitingToRotate = false;
    private boolean waitingForBall = false;
    private long colorDetectedTime = 0;
    private int nextIndexAfterDelay = -1;
    private long colorDelayMs = 50;

    // ---------------- CONSTRUCTOR ----------------

    public Spindexer(HardwareMap hardwareMap, String spinMotorName,
                     String colorSensor1Name, String colorSensor2Name) {
        spinMotor = hardwareMap.get(DcMotor.class, spinMotorName);
        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinMotor.setDirection(DcMotor.Direction.REVERSE);

        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, colorSensor1Name);
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, colorSensor2Name);
        colorSensor1.setGain(COLOR_SENSOR_GAIN);
        colorSensor2.setGain(COLOR_SENSOR_GAIN);
        enableLight(colorSensor1);
        enableLight(colorSensor2);
    }

    public Spindexer(HardwareMap hardwareMap) {
        this(hardwareMap, "spinMotor", "intakeColor", "intakeColor2");
    }

    // ---------------- ROTATION ----------------

    public void rotateToIndex(int targetIndex) {
        currentIndex = targetIndex;
        int[] positions = currentMode == Mode.INTAKE ? INTAKE_POSITIONS : OUTTAKE_POSITIONS;
        int basePosition = positions[targetIndex];
        int targetPosition = closestModularPosition(
                basePosition, spinMotor.getCurrentPosition());
        lastTargetPosition = targetPosition;
        spinMotor.setTargetPosition(targetPosition);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(motorSpeed);
    }

    private int closestModularPosition(int targetMod, int currentPosition) {
        int bestPosition = targetMod;
        int minDifference = Integer.MAX_VALUE;
        int rev = MagicConstants.SPINDEXER_TICKS_PER_REV;
        for (int k = -2; k <= 2; k++) {
            int candidate = targetMod + rev * k;
            int difference = Math.abs(candidate - currentPosition);
            if (difference < minDifference) {
                minDifference = difference;
                bestPosition = candidate;
            }
        }
        return bestPosition;
    }

    public boolean isAtTarget() {
        return Math.abs(spinMotor.getCurrentPosition() - lastTargetPosition)
                <= SPIN_TOLERANCE_TICKS;
    }

    public boolean isBusy() {
        return spinMotor.isBusy();
    }

    // ---------------- MODE ----------------

    public void setMode(Mode mode) {
        this.currentMode = mode;
    }

    public Mode getMode() {
        return currentMode;
    }

    // ---------------- SLOTS ----------------

    public void resetSlots() {
        for (int i = 0; i < 3; i++) slots[i] = Ball.EMPTY;
    }

    public void setSlot(int index, Ball ball) {
        if (index >= 0 && index < 3) slots[index] = ball;
    }

    public Ball getSlot(int index) {
        if (index >= 0 && index < 3) return slots[index];
        return Ball.EMPTY;
    }

    public Ball[] getSlots() {
        return slots.clone();
    }

    public int getCurrentIndex() {
        return currentIndex;
    }

    public void preload(Ball slot0, Ball slot1, Ball slot2) {
        slots[0] = slot0;
        slots[1] = slot1;
        slots[2] = slot2;
    }

    public int findNextEmpty() {
        for (int i = 0; i < 3; i++) {
            int idx = (currentIndex + i) % 3;
            if (slots[idx] == Ball.EMPTY) return idx;
        }
        return -1;
    }

    public int findClosestBall(Ball target) {
        for (int i = 0; i < 3; i++) {
            int idx = (currentIndex + i) % 3;
            if (slots[idx] == target) return idx;
        }
        return -1;
    }

    public int findClosestLoaded() {
        for (int i = 0; i < 3; i++) {
            int idx = (currentIndex + i) % 3;
            if (slots[idx] != Ball.EMPTY) return idx;
        }
        return -1;
    }

    public boolean contains(Ball ball) {
        for (Ball slot : slots) {
            if (slot == ball) return true;
        }
        return false;
    }

    public boolean isEmpty() {
        for (Ball slot : slots) {
            if (slot != Ball.EMPTY) return false;
        }
        return true;
    }

    public boolean isFull() {
        for (Ball slot : slots) {
            if (slot == Ball.EMPTY) return false;
        }
        return true;
    }

    // ---------------- AIMING ----------------

    public void aimClosest(Ball target) {
        setMode(Mode.OUTTAKE);
        int idx = findClosestBall(target);
        if (idx != -1) rotateToIndex(idx);
    }

    public void markCurrentEmpty() {
        slots[currentIndex] = Ball.EMPTY;
    }

    // ---------------- COLOR DETECTION ----------------

    public Ball detectBall() {
        Ball ball1 = detectSingleSensor(colorSensor1);
        Ball ball2 = detectSingleSensor(colorSensor2);
        if (ball1 == Ball.PURPLE || ball2 == Ball.PURPLE) return Ball.PURPLE;
        if (ball1 == Ball.GREEN || ball2 == Ball.GREEN) return Ball.GREEN;
        return Ball.EMPTY;
    }

    private Ball detectSingleSensor(NormalizedColorSensor sensor) {
        NormalizedRGBA colors = sensor.getNormalizedColors();
        float r = colors.red, g = colors.green, b = colors.blue;
        if (r + g + b < MIN_COLOR_TOTAL) return Ball.EMPTY;
        if (b > r * PURPLE_RATIO && b > g * PURPLE_GREEN_RATIO && b > PURPLE_BLUE_THRESHOLD) {
            return Ball.PURPLE;
        }
        if (g > r * GREEN_RATIO && g > b * GREEN_RATIO && g > GREEN_THRESHOLD) {
            return Ball.GREEN;
        }
        return Ball.EMPTY;
    }

    private void enableLight(NormalizedColorSensor sensor) {
        if (sensor instanceof SwitchableLight) {
            ((SwitchableLight) sensor).enableLight(true);
        }
    }

    // ---------------- INTAKE LOOP ----------------

    public boolean updateIntake() {
        if (waitingToRotate) {
            if (System.currentTimeMillis() - colorDetectedTime >= colorDelayMs) {
                if (nextIndexAfterDelay != -1) {
                    rotateToIndex(nextIndexAfterDelay);
                    waitingForBall = true;
                } else {
                    waitingForBall = true;
                }
                waitingToRotate = false;
            }
            return false;
        }

        if (waitingForBall && currentMode == Mode.INTAKE && !spinMotor.isBusy()) {
            Ball detected = detectBall();
            if (detected != Ball.EMPTY) {
                slots[currentIndex] = detected;
                waitingForBall = false;
                nextIndexAfterDelay = findNextEmpty();
                colorDetectedTime = System.currentTimeMillis();
                waitingToRotate = true;
                return true;
            }
        }
        return false;
    }

    public void startWaitingForBall() {
        waitingForBall = true;
        waitingToRotate = false;
    }

    public void stopWaitingForBall() {
        waitingForBall = false;
        waitingToRotate = false;
    }

    public boolean isWaitingForBall() {
        return waitingForBall;
    }

    // ---------------- SWEEP SHOOTING ----------------

    private boolean sweepActive = false;
    private int sweepTargetPosition = 0;

    public void startShootAllSweep() {
        sweepActive = true;
        currentMode = Mode.OUTTAKE;

        int current = spinMotor.getCurrentPosition();
        int rev = MagicConstants.SPINDEXER_TICKS_PER_REV;
        int currentMod = ((current % rev) + rev) % rev;
        int targetInRev = OUTTAKE_POSITIONS[2];
        if (targetInRev <= currentMod) targetInRev += rev;
        sweepTargetPosition = current - currentMod + targetInRev;

        spinMotor.setTargetPosition(sweepTargetPosition);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(0.35);
    }

    public boolean isSweepComplete() {
        if (!sweepActive) return true;
        if (!spinMotor.isBusy()) {
            sweepActive = false;
            resetSlots();
            currentIndex = 2;
            return true;
        }
        return false;
    }

    public boolean isSweepActive() {
        return sweepActive;
    }

    public void cancelSweep() {
        sweepActive = false;
        spinMotor.setPower(0);
    }

    // ---------------- CONFIG ----------------

    public void setMotorSpeed(double speed) {
        this.motorSpeed = speed;
    }

    public void setColorDelayMs(long delayMs) {
        this.colorDelayMs = delayMs;
    }

    // ---------------- TELEMETRY ----------------

    public String getSlotsString() {
        return String.format("[%s, %s, %s]", slots[0], slots[1], slots[2]);
    }

    public int getCurrentPosition() {
        return spinMotor.getCurrentPosition();
    }
}
