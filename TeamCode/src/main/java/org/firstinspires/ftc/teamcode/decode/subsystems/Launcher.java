package org.firstinspires.ftc.teamcode.decode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Unified Launcher class for managing the ball launching mechanism.
 * Controls the main flywheel motor, hood servo, and flick motor (mini flywheel) for shooting balls.
 *
 * The flick motor is a continuously rotating mini flywheel that launches balls when spun up.
 */
public class Launcher {

    // ==================== HARDWARE ====================
    private final DcMotorEx launchMotor;    // Main flywheel
    private final DcMotorEx flickMotor;      // Mini flywheel for flicking balls
    private final Servo hoodServo;

    // ==================== CONSTANTS ====================
    // Hood servo positions
    private static final double HOOD_DEFAULT_POSITION = 0.77;

    // Main flywheel velocities (ticks/sec for velocity control)
    private static final double LAUNCH_VELOCITY_HIGH = 2500;    // Full power shooting
    private static final double LAUNCH_VELOCITY_DEFAULT = 2000; // Standard shooting
    private static final double LAUNCH_VELOCITY_IDLE = 900;     // Idle spin (keeps flywheel warm)

    // Flick motor power
    private static final double FLICK_POWER_ON = 1.0;   // Flick motor on (launching)
    private static final double FLICK_POWER_OFF = 0.0;  // Flick motor off

    // PIDF coefficients for launch motor velocity control
    private static final double LAUNCH_P = 200;
    private static final double LAUNCH_I = 0;
    private static final double LAUNCH_D = 0;
    private static final double LAUNCH_F = 17.4;

    // ==================== STATE ====================
    private boolean isFlywheelRunning = false;
    private boolean isFlickMotorRunning = false;
    private double currentLaunchVelocity = LAUNCH_VELOCITY_DEFAULT;

    // ==================== CONSTRUCTOR ====================
    /**
     * Creates a new Launcher with the specified hardware components.
     *
     * @param hardwareMap The hardware map from the OpMode
     * @param launchMotorName Name of the main launch motor in hardware config
     * @param flickMotorName Name of the flick motor (mini flywheel)
     * @param hoodServoName Name of the hood servo
     */
    public Launcher(HardwareMap hardwareMap, String launchMotorName,
                    String flickMotorName, String hoodServoName) {
        // Initialize main launch motor with velocity control
        launchMotor = hardwareMap.get(DcMotorEx.class, launchMotorName);
        launchMotor.setDirection(DcMotorEx.Direction.REVERSE);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PIDF coefficients for velocity control
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(LAUNCH_P, LAUNCH_I, LAUNCH_D, LAUNCH_F);
        launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Initialize flick motor (mini flywheel)
        flickMotor = hardwareMap.get(DcMotorEx.class, flickMotorName);
        flickMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flickMotor.setPower(0);

        // Initialize hood servo
        hoodServo = hardwareMap.servo.get(hoodServoName);
        hoodServo.setPosition(HOOD_DEFAULT_POSITION);
    }

    /**
     * Creates a new Launcher with default hardware names.
     */
    public Launcher(HardwareMap hardwareMap) {
        this(hardwareMap, "launchMotor", "flickMotor", "hoodServo");
    }

    // ==================== MAIN FLYWHEEL CONTROL ====================
    /**
     * Starts the main flywheel at the default velocity.
     */
    public void startFlywheel() {
        launchMotor.setVelocity(currentLaunchVelocity);
        isFlywheelRunning = true;
    }

    /**
     * Starts the main flywheel at a specific velocity.
     *
     * @param velocity The target velocity in ticks per second
     */
    public void startFlywheel(double velocity) {
        currentLaunchVelocity = velocity;
        launchMotor.setVelocity(velocity);
        isFlywheelRunning = true;
    }

    /**
     * Sets the flywheel to high velocity for powerful shots.
     */
    public void setFlywheelHigh() {
        launchMotor.setVelocity(LAUNCH_VELOCITY_HIGH);
        currentLaunchVelocity = LAUNCH_VELOCITY_HIGH;
        isFlywheelRunning = true;
    }

    /**
     * Sets the flywheel to idle velocity (keeps it warm without full power).
     */
    public void setFlywheelIdle() {
        launchMotor.setVelocity(LAUNCH_VELOCITY_IDLE);
        isFlywheelRunning = true;
    }

    /**
     * Stops the main flywheel completely.
     */
    public void stopFlywheel() {
        launchMotor.setVelocity(0);
        launchMotor.setPower(0);
        isFlywheelRunning = false;
    }

    /**
     * Checks if the main flywheel is currently running.
     */
    public boolean isFlywheelRunning() {
        return isFlywheelRunning;
    }

    /**
     * Gets the current flywheel velocity.
     */
    public double getFlywheelVelocity() {
        return launchMotor.getVelocity();
    }

    /**
     * Sets the default launch velocity for future startFlywheel() calls.
     */
    public void setDefaultLaunchVelocity(double velocity) {
        this.currentLaunchVelocity = velocity;
    }

    // ==================== FLICK MOTOR CONTROL ====================
    /**
     * Starts the flick motor (mini flywheel) to launch balls.
     */
    public void startFlickMotor() {
        flickMotor.setPower(FLICK_POWER_ON);
        isFlickMotorRunning = true;
    }

    /**
     * Starts the flick motor at a specific power.
     *
     * @param power The motor power (0.0 to 1.0)
     */
    public void startFlickMotor(double power) {
        flickMotor.setPower(power);
        isFlickMotorRunning = power > 0;
    }

    /**
     * Stops the flick motor.
     */
    public void stopFlickMotor() {
        flickMotor.setPower(FLICK_POWER_OFF);
        isFlickMotorRunning = false;
    }

    /**
     * Checks if the flick motor is currently running.
     */
    public boolean isFlickMotorRunning() {
        return isFlickMotorRunning;
    }

    /**
     * Gets the current flick motor power.
     */
    public double getFlickMotorPower() {
        return flickMotor.getPower();
    }

    // ==================== HOOD CONTROL ====================
    /**
     * Sets the hood servo position.
     *
     * @param position Servo position (0.0 to 1.0)
     */
    public void setHoodPosition(double position) {
        hoodServo.setPosition(position);
    }

    /**
     * Resets the hood to the default position.
     */
    public void resetHood() {
        hoodServo.setPosition(HOOD_DEFAULT_POSITION);
    }

    /**
     * Gets the current hood position.
     */
    public double getHoodPosition() {
        return hoodServo.getPosition();
    }

    // ==================== COMBINED SHOOTING OPERATIONS ====================
    /**
     * Prepares the launcher for shooting (starts main flywheel, stops flick motor).
     */
    public void prepareToShoot() {
        stopFlickMotor();
        startFlywheel();
    }

    /**
     * Prepares with high power for longer shots.
     */
    public void prepareToShootHigh() {
        stopFlickMotor();
        setFlywheelHigh();
    }

    /**
     * Starts shooting by activating both the main flywheel and flick motor.
     * Call this when ready to launch balls.
     */
    public void shoot() {
        startFlywheel();
        startFlickMotor();
    }

    /**
     * Starts shooting at high velocity.
     */
    public void shootHigh() {
        setFlywheelHigh();
        startFlickMotor();
    }

    /**
     * Stops shooting (stops flick motor but keeps main flywheel running).
     */
    public void stopShooting() {
        stopFlickMotor();
    }

    /**
     * Completely stops the launcher (both motors).
     */
    public void stopAll() {
        stopFlickMotor();
        stopFlywheel();
    }

    /**
     * Resets the launcher to idle state.
     */
    public void reset() {
        stopFlickMotor();
        setFlywheelIdle();
        resetHood();
    }

    // ==================== TIMED SHOOTING ====================
    /**
     * Shoots for a specified duration (blocking).
     * Useful for shooting a known number of balls.
     *
     * @param durationMs How long to run the flick motor in milliseconds
     */
    public void shootTimed(long durationMs) {
        startFlickMotor();
        try {
            Thread.sleep(durationMs);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        stopFlickMotor();
    }

    // ==================== NON-BLOCKING TIMED SHOOT ====================
    private boolean timedShootActive = false;
    private long timedShootStartTime = 0;
    private long timedShootDuration = 0;

    /**
     * Starts a non-blocking timed shoot sequence.
     *
     * @param durationMs How long to run the flick motor
     */
    public void startTimedShoot(long durationMs) {
        timedShootDuration = durationMs;
        timedShootStartTime = System.currentTimeMillis();
        timedShootActive = true;
        startFlickMotor();
    }

    /**
     * Updates the non-blocking timed shoot sequence.
     * Call this in your main loop.
     *
     * @return true if the timed shoot is complete (or not active)
     */
    public boolean updateTimedShoot() {
        if (!timedShootActive) {
            return true;
        }

        if (System.currentTimeMillis() - timedShootStartTime >= timedShootDuration) {
            stopFlickMotor();
            timedShootActive = false;
            return true;
        }

        return false;
    }

    /**
     * Checks if a timed shoot is currently in progress.
     */
    public boolean isTimedShootActive() {
        return timedShootActive;
    }

    /**
     * Cancels any active timed shoot.
     */
    public void cancelTimedShoot() {
        timedShootActive = false;
        stopFlickMotor();
    }

    // ==================== TELEMETRY ====================
    /**
     * Gets the current launch motor power.
     */
    public double getLaunchPower() {
        return launchMotor.getPower();
    }

    /**
     * Gets information for telemetry.
     */
    public String getTelemetryString() {
        return String.format("Flywheel: %.0f vel, Flick: %.2f, Hood: %.2f",
                launchMotor.getVelocity(), flickMotor.getPower(), hoodServo.getPosition());
    }

    // ==================== STATIC VELOCITY CONSTANTS (for external use) ====================
    public static double getDefaultVelocity() {
        return LAUNCH_VELOCITY_DEFAULT;
    }

    public static double getHighVelocity() {
        return LAUNCH_VELOCITY_HIGH;
    }

    public static double getIdleVelocity() {
        return LAUNCH_VELOCITY_IDLE;
    }
}