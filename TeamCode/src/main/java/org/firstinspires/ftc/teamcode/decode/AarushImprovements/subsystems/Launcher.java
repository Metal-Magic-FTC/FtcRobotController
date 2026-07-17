package org.firstinspires.ftc.teamcode.decode.AarushImprovements.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Launcher: main flywheel, mini-flywheel (flick), and hood servo.
 *
 * <p>Default hardware names: {@code launchMotor}, {@code flickMotor},
 * {@code hoodServo}.</p>
 */
public class Launcher {

    // ---------------- HARDWARE ----------------
    private final DcMotorEx launchMotor;
    private final DcMotorEx flickMotor;
    private final Servo hoodServo;

    // ---------------- CONSTANTS ----------------
    private static final double HOOD_DEFAULT_POSITION = 0.77;
    private static final double LAUNCH_VELOCITY_HIGH = 2500;
    private static final double LAUNCH_VELOCITY_DEFAULT = 2000;
    private static final double LAUNCH_VELOCITY_IDLE = 900;
    private static final double FLICK_POWER_ON = 1.0;

    private static final double LAUNCH_P = 200;
    private static final double LAUNCH_I = 0;
    private static final double LAUNCH_D = 0;
    private static final double LAUNCH_F = 17.4;

    // ---------------- STATE ----------------
    private boolean isFlywheelRunning = false;
    private boolean isFlickMotorRunning = false;
    private double currentLaunchVelocity = LAUNCH_VELOCITY_DEFAULT;

    // ---------------- CONSTRUCTOR ----------------

    public Launcher(HardwareMap hardwareMap, String launchMotorName,
                    String flickMotorName, String hoodServoName) {
        launchMotor = hardwareMap.get(DcMotorEx.class, launchMotorName);
        launchMotor.setDirection(DcMotorEx.Direction.REVERSE);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(LAUNCH_P, LAUNCH_I, LAUNCH_D, LAUNCH_F));

        flickMotor = hardwareMap.get(DcMotorEx.class, flickMotorName);
        flickMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flickMotor.setPower(0);

        hoodServo = hardwareMap.servo.get(hoodServoName);
        hoodServo.setPosition(HOOD_DEFAULT_POSITION);
    }

    public Launcher(HardwareMap hardwareMap) {
        this(hardwareMap, "launchMotor", "flickMotor", "hoodServo");
    }

    // ---------------- MAIN FLYWHEEL ----------------

    public void startFlywheel() {
        launchMotor.setVelocity(currentLaunchVelocity);
        isFlywheelRunning = true;
    }

    public void startFlywheel(double velocity) {
        currentLaunchVelocity = velocity;
        launchMotor.setVelocity(velocity);
        isFlywheelRunning = true;
    }

    public void setFlywheelHigh() {
        launchMotor.setVelocity(LAUNCH_VELOCITY_HIGH);
        currentLaunchVelocity = LAUNCH_VELOCITY_HIGH;
        isFlywheelRunning = true;
    }

    public void setFlywheelIdle() {
        launchMotor.setVelocity(LAUNCH_VELOCITY_IDLE);
        isFlywheelRunning = true;
    }

    public void stopFlywheel() {
        launchMotor.setVelocity(0);
        launchMotor.setPower(0);
        isFlywheelRunning = false;
    }

    public boolean isFlywheelRunning() {
        return isFlywheelRunning;
    }

    public double getFlywheelVelocity() {
        return launchMotor.getVelocity();
    }

    public void setDefaultLaunchVelocity(double velocity) {
        this.currentLaunchVelocity = velocity;
    }

    // ---------------- FLICK MOTOR ----------------

    public void startFlickMotor() {
        flickMotor.setPower(FLICK_POWER_ON);
        isFlickMotorRunning = true;
    }

    public void startFlickMotor(double power) {
        flickMotor.setPower(power);
        isFlickMotorRunning = power > 0;
    }

    public void stopFlickMotor() {
        flickMotor.setPower(0);
        isFlickMotorRunning = false;
    }

    public boolean isFlickMotorRunning() {
        return isFlickMotorRunning;
    }

    public double getFlickMotorPower() {
        return flickMotor.getPower();
    }

    // ---------------- HOOD ----------------

    public void setHoodPosition(double position) {
        hoodServo.setPosition(position);
    }

    public void resetHood() {
        hoodServo.setPosition(HOOD_DEFAULT_POSITION);
    }

    public double getHoodPosition() {
        return hoodServo.getPosition();
    }

    // ---------------- COMPOSITE OPERATIONS ----------------

    public void prepareToShoot() {
        stopFlickMotor();
        startFlywheel();
    }

    public void prepareToShootHigh() {
        stopFlickMotor();
        setFlywheelHigh();
    }

    public void shoot() {
        startFlywheel();
        startFlickMotor();
    }

    public void shootHigh() {
        setFlywheelHigh();
        startFlickMotor();
    }

    public void stopShooting() {
        stopFlickMotor();
    }

    public void stopAll() {
        stopFlickMotor();
        stopFlywheel();
    }

    public void reset() {
        stopFlickMotor();
        setFlywheelIdle();
        resetHood();
    }

    // ---------------- TIMED SHOOTING ----------------

    public void shootTimed(long durationMs) {
        startFlickMotor();
        try {
            Thread.sleep(durationMs);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        stopFlickMotor();
    }

    private boolean timedShootActive = false;
    private long timedShootStartTime = 0;
    private long timedShootDuration = 0;

    public void startTimedShoot(long durationMs) {
        timedShootDuration = durationMs;
        timedShootStartTime = System.currentTimeMillis();
        timedShootActive = true;
        startFlickMotor();
    }

    public boolean updateTimedShoot() {
        if (!timedShootActive) return true;
        if (System.currentTimeMillis() - timedShootStartTime >= timedShootDuration) {
            stopFlickMotor();
            timedShootActive = false;
            return true;
        }
        return false;
    }

    public boolean isTimedShootActive() {
        return timedShootActive;
    }

    public void cancelTimedShoot() {
        timedShootActive = false;
        stopFlickMotor();
    }

    // ---------------- TELEMETRY ----------------

    public double getLaunchPower() {
        return launchMotor.getPower();
    }

    public String getTelemetryString() {
        return String.format("Flywheel: %.0f vel, Flick: %.2f, Hood: %.2f",
                launchMotor.getVelocity(), flickMotor.getPower(), hoodServo.getPosition());
    }

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
