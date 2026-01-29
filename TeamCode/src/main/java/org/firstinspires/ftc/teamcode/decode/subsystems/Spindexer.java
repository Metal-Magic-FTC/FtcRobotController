package org.firstinspires.ftc.teamcode.decode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

/**
 * Unified Spindexer class for managing the rotating ball storage mechanism.
 * Handles ball detection via color sensors, slot management, and rotation control.
 */
public class Spindexer {

    // ==================== ENUMS ====================
    public enum Ball {
        EMPTY,
        PURPLE,
        GREEN
    }

    public enum Mode {
        INTAKE,   // Positions for receiving balls
        OUTTAKE   // Positions for shooting balls
    }

    // ==================== HARDWARE ====================
    private final DcMotor spinMotor;
    private final NormalizedColorSensor colorSensor1;
    private final NormalizedColorSensor colorSensor2;

    // ==================== CONSTANTS ====================
    private static final int[] OUTTAKE_POSITIONS = {504, 2, 252};
    private static final int[] INTAKE_POSITIONS = {125, 375, 625};
    private static final int ENCODER_TICKS_PER_REV = 750;

    private static final int SPIN_TOLERANCE_TICKS = 5;
    private static final long SPIN_TIMEOUT_MS = 10000;

    private static final float COLOR_SENSOR_GAIN = 20f;
    private static final float MIN_COLOR_TOTAL = 0.07f;
    private static final float PURPLE_BLUE_THRESHOLD = 0.12f;
    private static final float GREEN_THRESHOLD = 0.15f;

    // ==================== STATE ====================
    private final Ball[] slots = {Ball.EMPTY, Ball.EMPTY, Ball.EMPTY};
    private int currentIndex = 0;
    private Mode currentMode = Mode.INTAKE;
    private double motorSpeed = 0.35;
    private int lastTargetPosition = 0;

    // Delayed rotation state (for color detection timing)
    private boolean waitingToRotate = false;
    private boolean waitingForBall = false;
    private long colorDetectedTime = 0;
    private int nextIndexAfterDelay = -1;
    private long colorDelayMs = 50;

    // ==================== CONSTRUCTOR ====================
    /**
     * Creates a new Spindexer with the specified hardware components.
     *
     * @param hardwareMap The hardware map from the OpMode
     * @param spinMotorName Name of the spin motor in hardware config
     * @param colorSensor1Name Name of the first color sensor
     * @param colorSensor2Name Name of the second color sensor
     */
    public Spindexer(HardwareMap hardwareMap, String spinMotorName,
                     String colorSensor1Name, String colorSensor2Name) {
        // Initialize motor
        spinMotor = hardwareMap.get(DcMotor.class, spinMotorName);
        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinMotor.setDirection(DcMotor.Direction.REVERSE);

        // Initialize color sensors
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, colorSensor1Name);
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, colorSensor2Name);

        colorSensor1.setGain(COLOR_SENSOR_GAIN);
        colorSensor2.setGain(COLOR_SENSOR_GAIN);

        enableLight(colorSensor1);
        enableLight(colorSensor2);
    }

    /**
     * Creates a new Spindexer with default hardware names.
     */
    public Spindexer(HardwareMap hardwareMap) {
        this(hardwareMap, "spinMotor", "intakeColor", "intakeColor2");
    }

    // ==================== ROTATION CONTROL ====================
    /**
     * Rotates the spindexer to the specified slot index.
     *
     * @param targetIndex The slot index (0, 1, or 2) to rotate to
     */
    public void rotateToIndex(int targetIndex) {
        currentIndex = targetIndex;
        int[] positions = (currentMode == Mode.INTAKE) ? INTAKE_POSITIONS : OUTTAKE_POSITIONS;
        int basePosition = positions[targetIndex];
        int targetPosition = closestModularPosition(basePosition, spinMotor.getCurrentPosition());

        lastTargetPosition = targetPosition;

        spinMotor.setTargetPosition(targetPosition);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(motorSpeed);
    }

    /**
     * Finds the closest equivalent modular position to minimize rotation distance.
     */
    private int closestModularPosition(int targetMod, int currentPosition) {
        int bestPosition = targetMod;
        int minDifference = Integer.MAX_VALUE;

        for (int k = -2; k <= 2; k++) {
            int candidate = targetMod + ENCODER_TICKS_PER_REV * k;
            int difference = Math.abs(candidate - currentPosition);
            if (difference < minDifference) {
                minDifference = difference;
                bestPosition = candidate;
            }
        }
        return bestPosition;
    }

    /**
     * Waits for the spindexer to reach its target position.
     * Call this in a loop that checks opModeIsActive().
     *
     * @return true if the spindexer has reached its target
     */
    public boolean isAtTarget() {
        int error = Math.abs(spinMotor.getCurrentPosition() - lastTargetPosition);
        return error <= SPIN_TOLERANCE_TICKS;
    }

    /**
     * Checks if the spin motor is currently busy.
     */
    public boolean isBusy() {
        return spinMotor.isBusy();
    }

    // ==================== MODE CONTROL ====================
    /**
     * Sets the spindexer mode (INTAKE or OUTTAKE).
     */
    public void setMode(Mode mode) {
        this.currentMode = mode;
    }

    /**
     * Gets the current spindexer mode.
     */
    public Mode getMode() {
        return currentMode;
    }

    // ==================== SLOT MANAGEMENT ====================
    /**
     * Resets all slots to empty.
     */
    public void resetSlots() {
        for (int i = 0; i < 3; i++) {
            slots[i] = Ball.EMPTY;
        }
    }

    /**
     * Sets the ball in a specific slot.
     */
    public void setSlot(int index, Ball ball) {
        if (index >= 0 && index < 3) {
            slots[index] = ball;
        }
    }

    /**
     * Gets the ball in a specific slot.
     */
    public Ball getSlot(int index) {
        if (index >= 0 && index < 3) {
            return slots[index];
        }
        return Ball.EMPTY;
    }

    /**
     * Gets the current slot index.
     */
    public int getCurrentIndex() {
        return currentIndex;
    }

    /**
     * Preloads the spindexer with balls (for autonomous start).
     */
    public void preload(Ball slot0, Ball slot1, Ball slot2) {
        slots[0] = slot0;
        slots[1] = slot1;
        slots[2] = slot2;
    }

    /**
     * Finds the next empty slot starting from the current index.
     *
     * @return The index of the next empty slot, or -1 if all slots are full
     */
    public int findNextEmpty() {
        for (int i = 0; i < 3; i++) {
            int idx = (currentIndex + i) % 3;
            if (slots[idx] == Ball.EMPTY) {
                return idx;
            }
        }
        return -1;
    }

    /**
     * Finds the closest slot containing the target ball type.
     *
     * @return The index of the closest matching slot, or -1 if not found
     */
    public int findClosestBall(Ball target) {
        for (int i = 0; i < 3; i++) {
            int idx = (currentIndex + i) % 3;
            if (slots[idx] == target) {
                return idx;
            }
        }
        return -1;
    }

    /**
     * Finds the closest loaded (non-empty) slot.
     */
    public int findClosestLoaded() {
        for (int i = 0; i < 3; i++) {
            int idx = (currentIndex + i) % 3;
            if (slots[idx] != Ball.EMPTY) {
                return idx;
            }
        }
        return -1;
    }

    /**
     * Checks if the spindexer contains a specific ball type.
     */
    public boolean contains(Ball ball) {
        for (Ball slot : slots) {
            if (slot == ball) return true;
        }
        return false;
    }

    /**
     * Checks if all slots are empty.
     */
    public boolean isEmpty() {
        for (Ball slot : slots) {
            if (slot != Ball.EMPTY) return false;
        }
        return true;
    }

    /**
     * Checks if all slots are full.
     */
    public boolean isFull() {
        for (Ball slot : slots) {
            if (slot == Ball.EMPTY) return false;
        }
        return true;
    }

    // ==================== AIMING ====================
    /**
     * Aims at the closest slot containing the target ball type.
     * Switches to OUTTAKE mode automatically.
     */
    public void aimClosest(Ball target) {
        setMode(Mode.OUTTAKE);
        int idx = findClosestBall(target);
        if (idx != -1) {
            rotateToIndex(idx);
        }
    }

    /**
     * Marks the current slot as empty (after shooting).
     */
    public void markCurrentEmpty() {
        slots[currentIndex] = Ball.EMPTY;
    }

    // ==================== COLOR DETECTION ====================
    /**
     * Detects what ball (if any) is currently at the intake using both color sensors.
     */
    public Ball detectBall() {
        Ball ball1 = detectSingleSensor(colorSensor1);
        Ball ball2 = detectSingleSensor(colorSensor2);

        // Prioritize detected balls
        if (ball1 == Ball.PURPLE || ball2 == Ball.PURPLE) return Ball.PURPLE;
        if (ball1 == Ball.GREEN || ball2 == Ball.GREEN) return Ball.GREEN;

        return Ball.EMPTY;
    }

    private Ball detectSingleSensor(NormalizedColorSensor sensor) {
        NormalizedRGBA colors = sensor.getNormalizedColors();
        float r = colors.red, g = colors.green, b = colors.blue;

        // Reject far / floor readings
        float total = r + g + b;
        if (total < MIN_COLOR_TOTAL) return Ball.EMPTY;

        // PURPLE: blue-dominant
        if (b > r * 1.35f && b > g * 1.25f && b > PURPLE_BLUE_THRESHOLD) {
            return Ball.PURPLE;
        }

        // GREEN: green-dominant
        if (g > r * 1.15f && g > b * 1.15f && g > GREEN_THRESHOLD) {
            return Ball.GREEN;
        }

        return Ball.EMPTY;
    }

    private void enableLight(NormalizedColorSensor sensor) {
        if (sensor instanceof SwitchableLight) {
            ((SwitchableLight) sensor).enableLight(true);
        }
    }

    // ==================== INTAKE LOOP HELPERS ====================
    /**
     * Call this in your intake loop. Handles automatic ball detection and slot rotation.
     *
     * @return true if a ball was just stored
     */
    public boolean updateIntake() {
        // Handle delayed rotation after color detection
        if (waitingToRotate) {
            if (System.currentTimeMillis() - colorDetectedTime >= colorDelayMs) {
                if (nextIndexAfterDelay != -1) {
                    rotateToIndex(nextIndexAfterDelay);
                    waitingForBall = true;
                } else {
                    waitingForBall = true; // Keep waiting even if full
                }
                waitingToRotate = false;
            }
            return false;
        }

        // Detect ball if waiting and motor isn't busy
        if (waitingForBall && currentMode == Mode.INTAKE && !spinMotor.isBusy()) {
            Ball detected = detectBall();

            if (detected != Ball.EMPTY) {
                slots[currentIndex] = detected;
                waitingForBall = false;

                // Prepare for delayed rotation
                nextIndexAfterDelay = findNextEmpty();
                colorDetectedTime = System.currentTimeMillis();
                waitingToRotate = true;

                return true; // Ball was stored
            }
        }

        return false;
    }

    /**
     * Starts waiting for a ball at the current position.
     */
    public void startWaitingForBall() {
        waitingForBall = true;
        waitingToRotate = false;
    }

    /**
     * Stops waiting for balls.
     */
    public void stopWaitingForBall() {
        waitingForBall = false;
        waitingToRotate = false;
    }

    /**
     * Checks if currently waiting for a ball.
     */
    public boolean isWaitingForBall() {
        return waitingForBall;
    }

    // ==================== SHOOT ALL SWEEP ====================
    private boolean sweepActive = false;
    private int sweepTargetPosition = 0;

    /**
     * Starts a sweep through all slots for rapid-fire shooting.
     * The spindexer will rotate from current position through all slots.
     * Use this with the flick motor running continuously.
     */
    public void startShootAllSweep() {
        sweepActive = true;
        currentMode = Mode.OUTTAKE;

        // Calculate sweep target - full rotation through all slots
        int current = spinMotor.getCurrentPosition();
        int currentMod = ((current % ENCODER_TICKS_PER_REV) + ENCODER_TICKS_PER_REV) % ENCODER_TICKS_PER_REV;

        // Target the last outtake position, ensuring we sweep forward
        int targetInRev = OUTTAKE_POSITIONS[2];
        if (targetInRev <= currentMod) {
            targetInRev += ENCODER_TICKS_PER_REV;
        }

        sweepTargetPosition = current - currentMod + targetInRev;

        spinMotor.setTargetPosition(sweepTargetPosition);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(0.35); // Slower speed for consistent shooting
    }

    /**
     * Checks if the shoot-all sweep is complete.
     *
     * @return true if the sweep is finished
     */
    public boolean isSweepComplete() {
        if (!sweepActive) {
            return true;
        }

        if (!spinMotor.isBusy()) {
            sweepActive = false;
            // Clear all slots after sweep
            resetSlots();
            currentIndex = 2; // End at last position
            return true;
        }

        return false;
    }

    /**
     * Checks if a sweep is currently active.
     */
    public boolean isSweepActive() {
        return sweepActive;
    }

    /**
     * Cancels any active sweep.
     */
    public void cancelSweep() {
        sweepActive = false;
        spinMotor.setPower(0);
    }

    // ==================== CONFIGURATION ====================
    /**
     * Sets the motor speed for rotation.
     */
    public void setMotorSpeed(double speed) {
        this.motorSpeed = speed;
    }

    /**
     * Sets the delay after color detection before rotating.
     */
    public void setColorDelayMs(long delayMs) {
        this.colorDelayMs = delayMs;
    }

    // ==================== TELEMETRY ====================
    /**
     * Gets a string representation of the current slots for telemetry.
     */
    public String getSlotsString() {
        return String.format("[%s, %s, %s]", slots[0], slots[1], slots[2]);
    }

    /**
     * Gets the current encoder position.
     */
    public int getEncoderPosition() {
        return spinMotor.getCurrentPosition();
    }

    /**
     * Gets the target encoder position.
     */
    public int getTargetPosition() {
        return lastTargetPosition;
    }
}