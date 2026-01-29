package org.firstinspires.ftc.teamcode.decode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Unified Intake class for managing the ball intake mechanism.
 * Controls the intake motor for collecting and ejecting balls.
 */
public class Intake {

    // ==================== HARDWARE ====================
    private final DcMotor intakeMotor;

    // ==================== CONSTANTS ====================
    // Motor power levels
    private static final double INTAKE_POWER_DEFAULT = -0.8;   // Negative for intaking
    private static final double INTAKE_POWER_SLOW = -0.6;      // Slower intake
    private static final double OUTTAKE_POWER_DEFAULT = 0.6;   // Positive for ejecting
    private static final double OUTTAKE_POWER_FAST = 0.8;      // Faster eject

    // ==================== STATE ====================
    private boolean isRunning = false;
    private boolean isIntaking = false;  // true = intaking, false = outtaking

    // ==================== CONSTRUCTOR ====================
    /**
     * Creates a new Intake with the specified hardware components.
     *
     * @param hardwareMap The hardware map from the OpMode
     * @param intakeMotorName Name of the intake motor in hardware config
     */
    public Intake(HardwareMap hardwareMap, String intakeMotorName) {
        intakeMotor = hardwareMap.get(DcMotor.class, intakeMotorName);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Creates a new Intake with default hardware names.
     */
    public Intake(HardwareMap hardwareMap) {
        this(hardwareMap, "intakeMotor");
    }

    // ==================== BASIC CONTROL ====================
    /**
     * Starts intaking at the default power.
     */
    public void intake() {
        intakeMotor.setPower(INTAKE_POWER_DEFAULT);
        isRunning = true;
        isIntaking = true;
    }

    /**
     * Starts intaking at a specific power.
     *
     * @param power The power level (should be negative for intaking)
     */
    public void intake(double power) {
        // Ensure power is negative for intaking
        double actualPower = power > 0 ? -power : power;
        intakeMotor.setPower(actualPower);
        isRunning = true;
        isIntaking = true;
    }

    /**
     * Starts slow intake.
     */
    public void intakeSlow() {
        intakeMotor.setPower(INTAKE_POWER_SLOW);
        isRunning = true;
        isIntaking = true;
    }

    /**
     * Starts outtaking (ejecting) at the default power.
     */
    public void outtake() {
        intakeMotor.setPower(OUTTAKE_POWER_DEFAULT);
        isRunning = true;
        isIntaking = false;
    }

    /**
     * Starts outtaking at a specific power.
     *
     * @param power The power level (should be positive for outtaking)
     */
    public void outtake(double power) {
        // Ensure power is positive for outtaking
        double actualPower = power < 0 ? -power : power;
        intakeMotor.setPower(actualPower);
        isRunning = true;
        isIntaking = false;
    }

    /**
     * Starts fast outtake.
     */
    public void outtakeFast() {
        intakeMotor.setPower(OUTTAKE_POWER_FAST);
        isRunning = true;
        isIntaking = false;
    }

    /**
     * Sets the motor power directly.
     * Negative values intake, positive values outtake.
     *
     * @param power The motor power (-1.0 to 1.0)
     */
    public void setPower(double power) {
        intakeMotor.setPower(power);
        isRunning = power != 0;
        isIntaking = power < 0;
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        intakeMotor.setPower(0);
        isRunning = false;
    }

    // ==================== STATE QUERIES ====================
    /**
     * Checks if the intake motor is currently running.
     */
    public boolean isRunning() {
        return isRunning;
    }

    /**
     * Checks if currently intaking (vs outtaking).
     */
    public boolean isIntaking() {
        return isIntaking && isRunning;
    }

    /**
     * Checks if currently outtaking.
     */
    public boolean isOuttaking() {
        return !isIntaking && isRunning;
    }

    /**
     * Gets the current motor power.
     */
    public double getPower() {
        return intakeMotor.getPower();
    }

    // ==================== DIRECTION CONTROL ====================
    /**
     * Reverses the intake motor direction.
     */
    public void reverseDirection() {
        if (intakeMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    /**
     * Sets the motor direction.
     *
     * @param direction The motor direction
     */
    public void setDirection(DcMotorSimple.Direction direction) {
        intakeMotor.setDirection(direction);
    }

    // ==================== TELEMETRY ====================
    /**
     * Gets information for telemetry.
     */
    public String getTelemetryString() {
        String state = isRunning ? (isIntaking ? "INTAKING" : "OUTTAKING") : "STOPPED";
        return String.format("Intake: %s (%.2f)", state, intakeMotor.getPower());
    }
}